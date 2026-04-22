#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
监控底盘（或多节点）TPDO1 中的 CiA402 状态字 0x6041 变化。

默认假设
--------
1) TPDO1 COB-ID = 0x180 + node_id
2) 0x6041 映射在 TPDO1 的前两个字节（LE）

注意
----
这两个都是“默认配置”假设：
- PDO COB-ID 可能被改
- PDO 映射也可能被改

因此脚本支持：
- --ids         自定义监听 COB-ID
- --sw-offset   指定 0x6041 在 TPDO1 数据中的字节偏移（默认 0）

MDX+ / CiA402 说明
------------------
0x6041 的低位可按 CiA402 / PDS 解码：
- bit0 rtso  Ready to switch on
- bit1 so    Switched on
- bit2 oe    Operation enabled
- bit3 f     Fault
- bit4 ve    Voltage enabled
- bit5 qs    Quick stop
- bit6 sod   Switch on disabled
- bit7 warn  Warning

其中 bit10~bit15 与控制模式相关（PP/PV/TQ/CSV/HM 等含义不同），
本脚本仅显示原始 6 位值，不直接做模式相关解释。

用法
----
# 采集 15s，仅在状态字变化时打印
python3 candump_tpdo1_statusword.py --capture can0 --duration 15

# 指定节点 ID（自动监听 0x180+n）
python3 candump_tpdo1_statusword.py --capture can0 --duration 10 --nodes 1,2,3

# 状态字不在 TPDO1 字节0~1，而在字节2~3
python3 candump_tpdo1_statusword.py --capture can0 --duration 10 --nodes 1,2,3 --sw-offset 2

# 自定义 COB-ID
python3 candump_tpdo1_statusword.py --capture can0 --duration 10 --ids 181,182,183

# 读日志
python3 candump_tpdo1_statusword.py /tmp/can.log

# 实时跟随
python3 candump_tpdo1_statusword.py --follow can0 --nodes 1,2,3

# 跟随 + 告警时读 SDO（需 EDS，独占 can0）
python3 candump_tpdo1_statusword.py --follow can0 --nodes 1,2,3 \
    --query-sdo-on-alert \
    --eds robot_base_ctl/hardware/canopen/eds/CANOPEN-EDS-MBDV-Servo-DulAxes-V1.0.eds

# 每帧都打印
python3 candump_tpdo1_statusword.py --capture can0 --duration 5 --all-frames
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import time
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_EDS = (
    SCRIPT_DIR.parent
    / "hardware"
    / "canopen"
    / "eds"
    / "CANOPEN-EDS-MBDV-Servo-DulAxes-V1.0.eds"
)

# candump -ta:
# (1775481418.697398) can0 182 [8] 37 16 xx xx ...
LINE_DATA = re.compile(
    r"\((?P<ts>\d+\.\d+)\)\s+\S+\s+(?P<id>[0-9A-Fa-f]+)\s+\[(?P<dlc>\d+)\]\s*(?P<rest>.*?)\s*$"
)


class Ds402State:
    NOT_READY = "NOT_READY"
    SWITCH_ON_DISABLED = "SWITCH_ON_DISABLED"
    READY_TO_SWITCH_ON = "READY_TO_SWITCH_ON"
    SWITCHED_ON = "SWITCHED_ON"
    OPERATION_ENABLED = "OPERATION_ENABLED"
    QUICK_STOP_ACTIVE = "QUICK_STOP_ACTIVE"
    FAULT_REACTION_ACTIVE = "FAULT_REACTION_ACTIVE"
    FAULT = "FAULT"
    UNKNOWN = "UNKNOWN"


def decode_ds402_state(statusword: int) -> str:
    """
    用 CiA402 主状态掩码解码 PDS 主状态。
    只看低位组合；bit10~15 不参与主状态判断。
    """
    masked = statusword & 0x006F

    if masked == 0x0000:
        return Ds402State.NOT_READY
    if masked == 0x0040:
        return Ds402State.SWITCH_ON_DISABLED
    if masked == 0x0021:
        return Ds402State.READY_TO_SWITCH_ON
    if masked == 0x0023:
        return Ds402State.SWITCHED_ON
    if masked == 0x0027:
        return Ds402State.OPERATION_ENABLED
    if masked == 0x0007:
        return Ds402State.QUICK_STOP_ACTIVE
    if masked == 0x000F:
        return Ds402State.FAULT_REACTION_ACTIVE
    if masked == 0x0008:
        return Ds402State.FAULT

    return Ds402State.UNKNOWN


def pds_bits_line(sw: int) -> str:
    """
    输出 MDX+/CiA402 常用状态位：
    bit0~7 + 原始 bit10~15
    """
    parts = [
        f"rtso={int(bool(sw & 0x0001))}",
        f"so={int(bool(sw & 0x0002))}",
        f"oe={int(bool(sw & 0x0004))}",
        f"f={int(bool(sw & 0x0008))}",
        f"ve={int(bool(sw & 0x0010))}",
        f"qs={int(bool(sw & 0x0020))}",
        f"sod={int(bool(sw & 0x0040))}",
        f"warn={int(bool(sw & 0x0080))}",
    ]
    hi = (sw >> 10) & 0x3F
    parts.append(f"bits10-15=0x{hi:02X}")
    return " ".join(parts)


def tpdo1_cob_id(node_id: int) -> int:
    return 0x180 + int(node_id)


def cob_to_node_id(cob: int) -> int:
    return int(cob) - 0x180


def parse_data_bytes(rest: str, dlc: int) -> list[int]:
    tokens = [t for t in rest.split() if re.fullmatch(r"[0-9A-Fa-f]{1,2}", t)]
    out: list[int] = []
    for t in tokens:
        out.append(int(t, 16))
        if len(out) >= dlc:
            break
    return out[:dlc]


def statusword_from_tpdo1(data: list[int], offset: int = 0) -> int | None:
    """
    从 TPDO1 数据中按 little-endian 读取 16bit 状态字 0x6041。
    默认 offset=0，即 data[0], data[1]。
    """
    if offset < 0:
        return None
    if len(data) < offset + 2:
        return None
    return int(data[offset]) | (int(data[offset + 1]) << 8)


def statusword_triggers_sdo_query(sw: int) -> tuple[bool, str]:
    """
    Warning/Fault/异常 PDS 时，建议读 SDO 诊断对象。
    """
    reasons: list[str] = []

    if sw & 0x0080:
        reasons.append("Warning")
    if sw & 0x0008:
        reasons.append("FaultBit")

    st = decode_ds402_state(sw)
    if st == Ds402State.FAULT:
        reasons.append("PDS_FAULT")
    elif st == Ds402State.FAULT_REACTION_ACTIVE:
        reasons.append("PDS_FAULT_REACTION")
    elif st == Ds402State.UNKNOWN:
        reasons.append("PDS_UNKNOWN")

    return (len(reasons) > 0, "+".join(reasons))


def _safe_sdo_read(node, idx: int, sub=None, default=None):
    try:
        if sub is None:
            return node.sdo[idx].raw
        return node.sdo[idx][sub].raw
    except Exception:
        return default


def format_mdx_sdo_diagnostics(node) -> str:
    """
    常见诊断对象：
    - 0x603F 错误代码
    - 0x1001 错误寄存器
    - 0x200F 厂商报警/诊断对象（若设备支持）
    """
    parts: list[str] = []

    v = _safe_sdo_read(node, 0x603F, default=None)
    parts.append(f"0x603F=0x{v:04X}" if v is not None else "0x603F=?")

    v = _safe_sdo_read(node, 0x1001, default=None)
    if v is None:
        parts.append("0x1001=?")
    else:
        if isinstance(v, int):
            parts.append(f"0x1001=0x{v:02X}")
        else:
            parts.append(f"0x1001={v}")

    v = _safe_sdo_read(node, 0x200F, default=None)
    if v is None:
        parts.append("0x200F=?")
    else:
        if isinstance(v, int):
            parts.append(f"0x200F=0x{v:08X}")
        else:
            parts.append(f"0x200F={v}")

    return "  ".join(parts)


def print_statusword_line(ts: float, cid_hex: str, nid: int, sw: int) -> None:
    node_s = f"node{nid}" if nid >= 0 else f"COB0x{cid_hex.upper()}"
    st = decode_ds402_state(sw)
    print(
        f"{ts:>14.6f}  0x{cid_hex.upper():>3s}  {node_s:>8s}  "
        f"SW=0x{sw:04X}  {st:22s}  |  {pds_bits_line(sw)}"
    )


def handle_frame(
    ts: float,
    cid_hex: str,
    node_by_cob: dict[str, int],
    data: list[int],
    *,
    all_frames: bool,
    last_sw: dict[str, int],
    sw_offset: int,
) -> None:
    sw = statusword_from_tpdo1(data, sw_offset)
    if sw is None:
        return

    key = cid_hex.lower()
    if not all_frames and last_sw.get(key) == sw:
        return

    last_sw[key] = sw
    nid = node_by_cob.get(key, -1)
    print_statusword_line(ts, key, nid, sw)


def iter_candump_lines(text: str):
    for line in text.splitlines():
        line = line.strip()
        if line:
            yield line


def process_line(
    line: str,
    watch_ids: set[str],
    node_by_cob: dict[str, int],
    *,
    all_frames: bool,
    last_sw: dict[str, int],
    sw_offset: int,
) -> None:
    m = LINE_DATA.match(line)
    if not m:
        return

    cid = m.group("id").lower()
    if cid not in watch_ids:
        return

    dlc = int(m.group("dlc"))
    if dlc < sw_offset + 2:
        return

    data = parse_data_bytes(m.group("rest") or "", dlc)
    if len(data) < sw_offset + 2:
        return

    ts = float(m.group("ts"))
    handle_frame(
        ts,
        cid,
        node_by_cob,
        data,
        all_frames=all_frames,
        last_sw=last_sw,
        sw_offset=sw_offset,
    )


def load_capture_text(iface: str, duration: float) -> str:
    if duration <= 0:
        print("--duration must be positive.", file=sys.stderr)
        sys.exit(2)

    cmd = ["timeout", str(duration), "candump", "-ta", iface]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, errors="replace")
    except FileNotFoundError as e:
        print(f"Need can-utils (candump) and timeout: {e}", file=sys.stderr)
        sys.exit(1)

    if proc.returncode not in (0, 124):
        print(proc.stderr or proc.stdout or "", file=sys.stderr)
        sys.exit(1)

    return proc.stdout


def load_file(path: str) -> str:
    if path == "-":
        return sys.stdin.read()

    p = Path(path)
    if not p.is_file():
        print(f"Not a file: {p}", file=sys.stderr)
        sys.exit(1)

    return p.read_text(errors="replace")


def _build_watch_set(args: argparse.Namespace) -> set[str]:
    if args.ids.strip():
        return {
            x.strip().lower().removeprefix("0x")
            for x in args.ids.split(",")
            if x.strip()
        }

    ids: set[str] = set()
    for part in args.nodes.split(","):
        part = part.strip()
        if not part:
            continue
        nid = int(part, 10)
        ids.add(f"{tpdo1_cob_id(nid):X}".lower())

    return ids


def _node_map(watch_cobs: set[str], args: argparse.Namespace) -> dict[str, int]:
    if args.ids.strip():
        return {}

    mapping: dict[str, int] = {}
    for part in args.nodes.split(","):
        part = part.strip()
        if not part:
            continue
        nid = int(part, 10)
        mapping[f"{tpdo1_cob_id(nid):X}".lower()] = nid

    return mapping


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Monitor CiA402 statusword 0x6041 from TPDO1/candump -ta."
    )

    p.add_argument(
        "logfile",
        nargs="?",
        default=None,
        help="candump -ta log path, or '-' for stdin",
    )

    p.add_argument(
        "--capture",
        metavar="IFACE",
        default="",
        help="Run candump -ta IFACE for --duration s",
    )
    p.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Seconds with --capture",
    )

    p.add_argument(
        "--follow",
        metavar="IFACE",
        default="",
        help="Stream candump -ta IFACE until Ctrl+C",
    )

    p.add_argument(
        "--nodes",
        type=str,
        default="1,2,3",
        help="Comma-separated node IDs; watches default TPDO1 COB-ID 0x180+node",
    )

    p.add_argument(
        "--ids",
        type=str,
        default="",
        help="Override watch IDs: comma-separated hex IDs as in candump, e.g. 181,182,183",
    )

    p.add_argument(
        "--sw-offset",
        type=int,
        default=0,
        help="Byte offset of 0x6041 within TPDO1 payload (little-endian), default 0",
    )

    p.add_argument(
        "--all-frames",
        action="store_true",
        help="Print every matching frame, not only on statusword change",
    )

    p.add_argument(
        "--query-sdo-on-alert",
        action="store_true",
        help=(
            "Only with --follow: use python-canopen to subscribe TPDO1 and query "
            "SDO 0x603F / 0x1001 / 0x200F when Warning/Fault/abnormal PDS is seen"
        ),
    )

    p.add_argument(
        "--eds",
        type=str,
        default="",
        help=f"EDS path for --query-sdo-on-alert; default tries {DEFAULT_EDS}",
    )

    p.add_argument(
        "--bitrate",
        type=int,
        default=500000,
        help="SocketCAN bitrate for --query-sdo-on-alert",
    )

    p.add_argument(
        "--sdo-cooldown",
        type=float,
        default=0.5,
        help="Minimum interval between SDO diagnostics on same node",
    )

    p.add_argument(
        "--sdo-timeout",
        type=float,
        default=1.0,
        help="Single SDO timeout in seconds",
    )

    return p.parse_args()


def follow_canopen_query_sdo(args: argparse.Namespace) -> int:
    """
    独占 SocketCAN，用 python-canopen subscribe 收 TPDO1，
    Warning/Fault/异常 PDS 时读诊断对象。
    """
    try:
        import canopen
    except ImportError as e:
        print("需要 python-canopen: pip install canopen", file=sys.stderr)
        print(e, file=sys.stderr)
        return 1

    eds_arg = (args.eds or "").strip()
    eds_path = Path(eds_arg).expanduser().resolve() if eds_arg else DEFAULT_EDS.resolve()
    if not eds_path.is_file():
        print(
            f"请指定有效 --eds（当前: {eds_path}）。默认尝试: {DEFAULT_EDS}",
            file=sys.stderr,
        )
        return 1

    watch_cobs = _build_watch_set(args)
    node_by_cob = _node_map(watch_cobs, args)

    network = canopen.Network()
    seen_nodes: set[int] = set()

    for cid_hex in watch_cobs:
        cob = int(cid_hex, 16)
        nid = node_by_cob.get(cid_hex.lower(), cob_to_node_id(cob))
        if nid in seen_nodes:
            continue
        seen_nodes.add(nid)

        node = canopen.BaseNode402(nid, str(eds_path))
        node.sdo.RESPONSE_TIMEOUT = float(args.sdo_timeout)
        network.add_node(node)

    try:
        network.connect(
            interface="socketcan",
            channel=args.follow,
            bitrate=int(args.bitrate),
        )
    except Exception as e:
        print(f"CANopen connect failed on {args.follow}: {e}", file=sys.stderr)
        return 1

    print(
        "Following TPDO1 via CANopen subscribe; "
        "Warning/Fault/abnormal PDS => SDO read 0x603F / 0x1001 / 0x200F. Ctrl+C to stop.\n"
    )
    print(f"EDS: {eds_path}")
    print(
        f"{'time(s)':>14s}  {'COB':>5s}  {'node':>8s}  "
        f"SW=hex   {'PDS_state':22s}  |  PDS bits"
    )

    last_sw: dict[str, int] = {}
    last_sdo_ts: dict[int, float] = {}
    cooldown = float(args.sdo_cooldown)
    all_frames = bool(args.all_frames)
    sw_offset = int(args.sw_offset)

    def on_tpdo(can_id: int, data: bytearray, bus_ts: float) -> None:
        if len(data) < sw_offset + 2:
            return

        sw = int(data[sw_offset]) | (int(data[sw_offset + 1]) << 8)
        cid_hex = f"{can_id:x}"

        if not all_frames and last_sw.get(cid_hex) == sw:
            return

        last_sw[cid_hex] = sw
        nid = node_by_cob.get(cid_hex, cob_to_node_id(can_id))
        ts = float(bus_ts) if bus_ts is not None else time.time()

        print_statusword_line(ts, cid_hex, nid, sw)

        need, why = statusword_triggers_sdo_query(sw)
        if not need:
            return

        now = time.time()
        if now - last_sdo_ts.get(nid, 0.0) < cooldown:
            return
        last_sdo_ts[nid] = now

        node = network.nodes.get(nid)
        if node is None:
            return

        diag = format_mdx_sdo_diagnostics(node)
        print(f"{'':>14s}           -> SDO node{nid} ({why})  {diag}")

    for cid_hex in watch_cobs:
        network.subscribe(int(cid_hex, 16), on_tpdo)

    try:
        while True:
            time.sleep(0.2)
            network.check()
    except KeyboardInterrupt:
        print("\n[stopped]", file=sys.stderr)
    finally:
        network.disconnect()

    return 0


def main() -> int:
    args = parse_args()

    if args.sw_offset < 0:
        print("--sw-offset must be >= 0", file=sys.stderr)
        return 2

    if args.query_sdo_on_alert and not args.follow:
        print("--query-sdo-on-alert must be used with --follow IFACE", file=sys.stderr)
        return 2

    if args.follow:
        if args.capture or args.logfile:
            print("Use --follow alone, or logfile/--capture without --follow.", file=sys.stderr)
            return 2

        if args.query_sdo_on_alert:
            return follow_canopen_query_sdo(args)

        watch_cobs = _build_watch_set(args)
        node_by_cob = _node_map(watch_cobs, args)

        print(
            "Following TPDO1 statusword 0x6041. "
            "Low bits are decoded as CiA402/PDS; bit7=Warning; "
            "bit10~15 are printed raw only. Ctrl+C to stop.\n"
        )
        print(
            f"{'time(s)':>14s}  {'COB':>5s}  {'node':>8s}  "
            f"SW=hex   {'PDS_state':22s}  |  PDS bits"
        )

        proc = None
        last_sw: dict[str, int] = {}

        try:
            proc = subprocess.Popen(
                ["candump", "-ta", args.follow],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                bufsize=1,
            )
            assert proc.stdout is not None

            for line in proc.stdout:
                process_line(
                    line.rstrip("\n"),
                    watch_cobs,
                    node_by_cob,
                    all_frames=args.all_frames,
                    last_sw=last_sw,
                    sw_offset=args.sw_offset,
                )

        except KeyboardInterrupt:
            print("\n[stopped]", file=sys.stderr)
        finally:
            if proc is not None and proc.poll() is None:
                proc.terminate()

        return 0

    if args.capture:
        if args.logfile:
            print("Use either logfile or --capture, not both.", file=sys.stderr)
            return 2
        text = load_capture_text(args.capture, args.duration)
        label = f"capture {args.capture} {args.duration}s"
    else:
        if not args.logfile:
            print("Need: logfile / '-' / --capture IFACE / --follow IFACE", file=sys.stderr)
            return 2
        text = load_file(args.logfile)
        label = args.logfile

    watch_cobs = _build_watch_set(args)
    node_by_cob = _node_map(watch_cobs, args)

    print(f"Source: {label}")
    print(f"Watching COB-IDs (hex): {', '.join(sorted(watch_cobs, key=lambda x: int(x, 16)))}")
    print(
        "\nDecode rule: low bits use CiA402/PDS main-state decode; "
        "bit7=Warning; bit10~15 are shown raw only, not mode-specific decoded.\n"
    )
    print(
        f"{'time(s)':>14s}  {'COB':>5s}  {'node':>8s}  "
        f"SW=hex   {'PDS_state':22s}  |  PDS bits"
    )

    last_sw: dict[str, int] = {}
    for line in iter_candump_lines(text):
        process_line(
            line,
            watch_cobs,
            node_by_cob,
            all_frames=args.all_frames,
            last_sw=last_sw,
            sw_offset=args.sw_offset,
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
