#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
监控底盘（或多节点）**TPDO1 状态字 0x6041** 的变化。

CiA301 默认 TxPDO1 COB-ID = **0x180 + node_id**（candump 第三列常为 181、182、183 对应 node 1/2/3）。

MDX+ 用户手册：0x6041 为 **PDS 状态字**，位 0~6 与 CiA402 状态机相关（手册表头常见：
rtso/bit0、so/bit1、oe/bit2、f/bit3、ve/bit4、qs/bit5、sod/bit6）；bit10~15 随控制模式变化。

用法
----
  # 采集 15s，仅在状态字变化时打印
  python3 candump_tpdo1_statusword.py --capture can0 --duration 15

  # 指定节点 ID（自动生成 0x180+n）
  python3 candump_tpdo1_statusword.py --capture can0 --duration 10 --nodes 1,2,3

  # 读日志
  python3 candump_tpdo1_statusword.py /tmp/can.log

  # 实时跟随（Ctrl+C 结束）
  python3 candump_tpdo1_statusword.py --follow can0 --nodes 1,2,3

  # 跟随 + 告警时读 SDO（MDX+ 手册：0x603F 错误码、0x200F DSP 报警；需 EDS，独占 can0）
  python3 candump_tpdo1_statusword.py --follow can0 --nodes 1,2,3 \\
      --query-sdo-on-alert --eds robot_base_ctl/hardware/canopen/eds/CANOPEN-EDS-MBDV-Servo-DulAxes-V1.0.eds

  # 每一帧都打印（变化检测关闭）
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


# candump -ta: (ts) ifname ID [dlc] dd dd ...
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
    """CiA402 PDS 主状态（与 Ds402_ctl / 手册 0x6041 低有效位组合一致）。"""
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
    MDX+ 手册 PDS 状态位命名（bit0~6 常用缩写）：
    rtso=Ready to switch on, so=Switched on, oe=Operation enabled, f=Fault,
    ve=Voltage enabled, qs=Quick stop, sod=Switch on disabled
    """
    parts = [
        f"rtso={int(bool(sw & 1))}",
        f"so={int(bool(sw & 2))}",
        f"oe={int(bool(sw & 4))}",
        f"f={int(bool(sw & 8))}",
        f"ve={int(bool(sw & 16))}",
        f"qs={int(bool(sw & 32))}",
        f"sod={int(bool(sw & 64))}",
    ]
    hi = (sw >> 10) & 0x3F
    parts.append(f"bits10-15=0x{hi:02X}")
    return " ".join(parts)


def tpdo1_cob_id(node_id: int) -> int:
    return 0x180 + int(node_id)


def parse_data_bytes(rest: str, dlc: int) -> list[int]:
    tok = [t for t in rest.split() if re.fullmatch(r"[0-9A-Fa-f]{1,2}", t, re.I)]
    out: list[int] = []
    for t in tok:
        out.append(int(t, 16))
        if len(out) >= dlc:
            break
    return out[:dlc]


def statusword_from_tpdo1(data: list[int]) -> int | None:
    """默认映射：0x6041 为 TPDO1 第一个对象，小端 16 位占字节 0~1。"""
    if len(data) < 2:
        return None
    return int(data[0]) | (int(data[1]) << 8)


def statusword_triggers_sdo_query(sw: int) -> tuple[bool, str]:
    """
    MDX+ 手册：0x6041 bit7=Warning（报警）；bit3=Fault；bit10~15 随模式变化。
    在 Warning / Fault / 非预期 PDS 时建议读 0x603F（手册 6.1 与 CiA402）及厂商 0x200F。
    """
    reasons: list[str] = []
    if sw & 0x0080:
        reasons.append("bit7_Warning")
    if sw & 0x0008:
        reasons.append("bit3_Fault")
    st = decode_ds402_state(sw)
    if st == Ds402State.FAULT:
        reasons.append("PDS_FAULT")
    if st == Ds402State.FAULT_REACTION_ACTIVE:
        reasons.append("PDS_FAULT_REACTION")
    if st == Ds402State.UNKNOWN:
        reasons.append("PDS_UNKNOWN")
    if not reasons:
        return False, ""
    return True, "+".join(reasons)


def _safe_sdo_read(node, idx: int, sub=None, default=None):
    try:
        if sub is None:
            return node.sdo[idx].raw
        return node.sdo[idx][sub].raw
    except Exception:
        return default


def format_mdx_sdo_diagnostics(node) -> str:
    """
    MDX+ 手册：错误代码 0x603F；DSP 报警 0x200F；标准错误寄存器 0x1001。
    """
    parts: list[str] = []
    v = _safe_sdo_read(node, 0x603F, default=None)
    if v is not None:
        parts.append(f"0x603F=0x{v:04X}")
    else:
        parts.append("0x603F=?")
    v = _safe_sdo_read(node, 0x1001, default=None)
    if v is not None:
        parts.append(f"0x1001=0x{v:08X}")
    else:
        parts.append("0x1001=?")
    v = _safe_sdo_read(node, 0x200F, default=None)
    if v is not None:
        parts.append(f"0x200F=0x{v:08X}")
    else:
        parts.append("0x200F=?")
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
) -> None:
    sw = statusword_from_tpdo1(data)
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
        if not line:
            continue
        yield line


def process_line(
    line: str,
    watch_ids: set[str],
    node_by_cob: dict[str, int],
    *,
    all_frames: bool,
    last_sw: dict[str, int],
) -> None:
    m = LINE_DATA.match(line)
    if not m:
        return
    cid = m.group("id").lower()
    if cid not in watch_ids:
        return
    dlc = int(m.group("dlc"))
    if dlc < 2:
        return
    data = parse_data_bytes(m.group("rest") or "", dlc)
    if len(data) < 2:
        return
    ts = float(m.group("ts"))
    handle_frame(ts, cid, node_by_cob, data, all_frames=all_frames, last_sw=last_sw)


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


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Monitor CiA402 statusword (0x6041) on TPDO1 (COB 0x180+node) from candump -ta."
    )
    p.add_argument(
        "logfile",
        nargs="?",
        default=None,
        help="candump -ta log path, or '-' for stdin",
    )
    p.add_argument("--capture", metavar="IFACE", default="", help="Run candump -ta IFACE for --duration s")
    p.add_argument("--duration", type=float, default=10.0, help="Seconds with --capture")
    p.add_argument(
        "--nodes",
        type=str,
        default="1,2,3",
        help="Comma-separated node IDs; watches COB 0x180+id (default 1,2,3 base wheels)",
    )
    p.add_argument(
        "--ids",
        type=str,
        default="",
        help="Override: comma-separated hex IDs as in candump (e.g. 181,182,183), ignores --nodes",
    )
    p.add_argument(
        "--all-frames",
        action="store_true",
        help="Print every matching frame, not only on statusword change",
    )
    p.add_argument(
        "--follow",
        metavar="IFACE",
        default="",
        help="Stream candump -ta IFACE until Ctrl+C (do not use with --capture)",
    )
    p.add_argument(
        "--query-sdo-on-alert",
        action="store_true",
        help=(
            "仅与 --follow 合用：用 CANopen（python-canopen）独占接口监听 TPDO1 COB，"
            "当状态字含 Warning(bit7)/Fault(bit3) 或 PDS 为 Fault/未知时，"
            "SDO 读取 MDX+ 手册中的 0x603F、0x1001、0x200F（需 --eds）"
        ),
    )
    p.add_argument(
        "--eds",
        type=str,
        default="",
        help=f"对象字典 EDS 路径（--query-sdo-on-alert 时必填；默认尝试 {DEFAULT_EDS}）",
    )
    p.add_argument(
        "--bitrate",
        type=int,
        default=500000,
        help="SocketCAN 比特率（--query-sdo-on-alert，默认 500000）",
    )
    p.add_argument(
        "--sdo-cooldown",
        type=float,
        default=0.5,
        help="同一 node 两次 SDO 查询最小间隔（秒），避免 0x06B7 等快速翻转刷总线",
    )
    p.add_argument(
        "--sdo-timeout",
        type=float,
        default=1.0,
        help="单次 SDO 超时（秒）",
    )
    return p.parse_args()


def cob_to_node_id(cob: int) -> int:
    """CiA301 默认 TPDO1 COB = 0x180 + node_id。"""
    return int(cob) - 0x180


def follow_canopen_query_sdo(args: argparse.Namespace) -> int:
    """独占 SocketCAN，用 network.subscribe 收 TPDO1，告警时 SDO 读诊断对象。"""
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
            f"请指定有效 --eds（当前: {eds_path}）。"
            f"默认路径: {DEFAULT_EDS}",
            file=sys.stderr,
        )
        return 1

    watch_cobs = _build_watch_set(args)
    node_by_cob = _node_map(watch_cobs, args)

    network = canopen.Network()
    seen: set[int] = set()
    for cid_hex in watch_cobs:
        cob = int(cid_hex, 16)
        nid = node_by_cob.get(cid_hex.lower(), cob_to_node_id(cob))
        if nid in seen:
            continue
        seen.add(nid)
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
        print(f"CANopen 连接失败 ({args.follow}): {e}", file=sys.stderr)
        return 1

    print(
        "Following TPDO1 (CANopen subscribe); "
        "MDX+：Warning(bit7)/Fault(bit3)/异常 PDS 时 SDO 读 0x603F / 0x1001 / 0x200F。Ctrl+C 结束。\n"
    )
    print(f"EDS: {eds_path}")
    print(
        f"{'time(s)':>14s}  {'COB':>5s}  {'node':>8s}  "
        f"SW=hex   {'PDS_state':22s}  |  PDS bits (MDX 缩写)"
    )

    last_sw: dict[str, int] = {}
    last_sdo_ts: dict[int, float] = {}
    cooldown = float(args.sdo_cooldown)
    all_frames = bool(args.all_frames)

    def on_tpdo(can_id: int, data: bytearray, bus_ts: float) -> None:
        if len(data) < 2:
            return
        sw = int(data[0]) | (int(data[1]) << 8)
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
    if args.query_sdo_on_alert and not args.follow:
        print("--query-sdo-on-alert 必须与 --follow IFACE 同时使用。", file=sys.stderr)
        return 2
    if args.follow:
        if args.capture or args.logfile:
            print("Use --follow alone, or log/--capture without --follow.", file=sys.stderr)
            return 2
        if args.query_sdo_on_alert:
            return follow_canopen_query_sdo(args)
        watch_cobs = _build_watch_set(args)
        node_by_cob = _node_map(watch_cobs, args)
        print(
            "Following TPDO1 statusword (0x6041 @ bytes 0-1 LE). "
            "MDX+ / CiA402: PDS bits0-6 + mode bits10-15. Ctrl+C to stop.\n"
        )
        print(
            f"{'time(s)':>14s}  {'COB':>5s}  {'node':>8s}  "
            f"SW=hex   {'PDS_state':22s}  |  PDS bits (MDX 缩写)"
        )
        last_sw: dict[str, int] = {}
        proc = None
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
            print(
                "Need: logfile / '-' / --capture IFACE / --follow IFACE",
                file=sys.stderr,
            )
            return 2
        text = load_file(args.logfile)
        label = args.logfile

    watch_cobs = _build_watch_set(args)
    node_by_cob = _node_map(watch_cobs, args)
    print(f"Source: {label}")
    print(f"Watching TPDO1 COB-IDs (hex): {', '.join(sorted(watch_cobs, key=lambda x: int(x, 16)))}")
    print(
        "\nMDX+ 手册: 0x6041 状态字反映 CiA402 PDS；下列解码使用标准 mask 0x006F。"
        " bit10~15 随模式变化，仅显示原始 6 位。\n"
    )
    print(
        f"{'time(s)':>14s}  {'COB':>5s}  {'node':>8s}  "
        f"SW=hex   {'PDS_state':22s}  |  PDS bits (MDX 缩写)"
    )
    last_sw: dict[str, int] = {}
    for line in iter_candump_lines(text):
        process_line(
            line,
            watch_cobs,
            node_by_cob,
            all_frames=args.all_frames,
            last_sw=last_sw,
        )
    return 0


def _build_watch_set(args: argparse.Namespace) -> set[str]:
    if args.ids.strip():
        return {x.strip().lower().removeprefix("0x") for x in args.ids.split(",") if x.strip()}
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
    m: dict[str, int] = {}
    for part in args.nodes.split(","):
        part = part.strip()
        if not part:
            continue
        nid = int(part, 10)
        m[f"{tpdo1_cob_id(nid):X}".lower()] = nid
    return m


if __name__ == "__main__":
    raise SystemExit(main())
