#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从 candump -ta 文本统计各 CAN ID 的平均帧率与间隔。

方式 A — 直接在 Python 里采集并计算（无需先写文件）
----------------------------------------------
  python3 candump_analyze_freq.py --capture can0 --duration 10 --ids 481,482,483

方式 B — 读已有日志
------------------
  python3 candump_analyze_freq.py /tmp/can0.log
  python3 candump_analyze_freq.py /tmp/can0.log --ids 481,482,483

方式 C — 管道
------------
  timeout 10 candump -ta can0 | python3 candump_analyze_freq.py -

说明
----
- --ids 与 candump 第三列一致（十六进制、无 0x，如 481 = TPDO4 节点 1）。
- 需要系统中有 candump、timeout（方式 A）；can0 勿与其它进程同时占用。
- 总线负载：与 canbusload 默认类似，采用 can-utils canframelen **CFL_WORSTCASE**
  （11 位 ID: 55+10*dlc bit/帧；29 位 ID: 80+10*dlc；含 IFS；见 canframelen.h）。
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path
from statistics import mean, median


LINE_RE = re.compile(
    r"\((?P<ts>\d+\.\d+)\)\s+\S+\s+(?P<id>[0-9A-Fa-f]+)\s+\["
)

# 含 DLC，用于总线负载（与 canbusload 统计范围一致：日志里出现的每一行经典帧）
LINE_DLC_RE = re.compile(
    r"\((?P<ts>\d+\.\d+)\)\s+\S+\s+(?P<id>[0-9A-Fa-f]+)\s+\[(?P<dlc>\d+)\]\s*"
)


def frame_bits_worstcase(can_id_hex: str, dlc: int) -> tuple[int, int]:
    """
    返回 (bits_total, bits_payload)。
    与 can-utils can_frame_length(..., CFL_WORSTCASE, CAN_MTU) 对经典帧一致。
    """
    dlc = max(0, min(8, dlc))
    try:
        cid = int(can_id_hex, 16)
    except ValueError:
        cid = 0
    eff = cid > 0x7FF
    if eff:
        bits_total = 80 + dlc * 10
    else:
        bits_total = 55 + dlc * 10
    return bits_total, dlc * 8


def bus_load_from_candump_text(text: str, bitrate: int) -> dict | None:
    """整段日志时间跨度内平均帧率、bit/s、负载%%；忽略无法解析 DLC 的行。"""
    events: list[tuple[float, int, int]] = []
    for m in LINE_DLC_RE.finditer(text):
        ts = float(m.group("ts"))
        bid, dlc_s = m.group("id"), m.group("dlc")
        try:
            dlc = int(dlc_s, 10)
        except ValueError:
            continue
        bt, _bp = frame_bits_worstcase(bid, dlc)
        events.append((ts, bt, dlc * 8))

    if len(events) < 2:
        return None

    events.sort(key=lambda e: e[0])
    t0, t1 = events[0][0], events[-1][0]
    span = t1 - t0
    if span <= 0:
        return None

    n = len(events)
    sum_total = sum(e[1] for e in events)
    sum_payload = sum(e[2] for e in events)
    fps = n / span
    bits_s_total = sum_total / span
    bits_s_payload = sum_payload / span
    load_pct = 100.0 * bits_s_total / float(bitrate)
    return {
        "frames": n,
        "span_s": span,
        "fps": fps,
        "bits_s_total": bits_s_total,
        "bits_s_payload": bits_s_payload,
        "load_pct": load_pct,
    }


def norm_hex(s: str) -> str:
    s = s.strip().lower()
    return s[2:] if s.startswith("0x") else s


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Analyze candump -ta output: frame rates per CAN ID."
    )
    p.add_argument(
        "logfile",
        nargs="?",
        default=None,
        help="Path to candump log, or '-' to read stdin",
    )
    p.add_argument(
        "--capture",
        metavar="IFACE",
        default="",
        help="Run candump -ta on IFACE for --duration seconds (no log file needed)",
    )
    p.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Seconds to capture when using --capture (default 10)",
    )
    p.add_argument(
        "--ids",
        type=str,
        default="",
        help="Comma-separated IDs as in candump (hex, no 0x), e.g. 481,482,483",
    )
    p.add_argument("--min-count", type=int, default=2)
    p.add_argument("--top", type=int, default=0)
    p.add_argument(
        "--bitrate",
        type=int,
        default=1_000_000,
        help="Bus bitrate (bit/s) for load %%, default 1000000 (1 Mbit/s)",
    )
    p.add_argument(
        "--iface",
        type=str,
        default="can0",
        help="Interface label for bus-load line (default can0)",
    )
    p.add_argument(
        "--no-bus-load",
        action="store_true",
        help="Skip canbusload-style bus utilization summary",
    )
    return p.parse_args()


def load_text(args: argparse.Namespace) -> tuple[str, str]:
    if args.capture:
        if args.logfile is not None:
            print(
                "Use either a log file (or '-') OR --capture, not both.",
                file=sys.stderr,
            )
            sys.exit(2)
        if args.duration <= 0:
            print("--duration must be positive.", file=sys.stderr)
            sys.exit(2)
        cmd = [
            "timeout",
            str(args.duration),
            "candump",
            "-ta",
            args.capture,
        ]
        try:
            proc = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                errors="replace",
            )
        except FileNotFoundError as e:
            print(
                f"Command not found ({e}). Install can-utils and coreutils (timeout).",
                file=sys.stderr,
            )
            sys.exit(1)
        if proc.returncode not in (0, 124):
            print(proc.stderr or proc.stdout or "(no output)", file=sys.stderr)
            print(
                f"candump failed (exit {proc.returncode}). Is {args.capture} up?",
                file=sys.stderr,
            )
            sys.exit(1)
        label = f"capture {args.capture} {args.duration}s ({len(proc.stdout)} bytes)"
        return proc.stdout, label

    if args.logfile is None:
        print(
            "Specify a log file, '-' for stdin, or use --capture can0 --duration N.",
            file=sys.stderr,
        )
        sys.exit(2)

    if args.logfile == "-":
        return sys.stdin.read(), "stdin"

    path = Path(args.logfile)
    if not path.is_file():
        print(f"Not a file: {path}", file=sys.stderr)
        print(
            "Tip: capture with  python3 candump_analyze_freq.py --capture can0 --duration 10",
            file=sys.stderr,
        )
        sys.exit(1)
    text = path.read_text(errors="replace")
    return text, f"{path} ({path.stat().st_size} bytes)"


def analyze(
    text: str,
    filter_ids: set[str] | None,
    min_count: int,
    top: int,
) -> list[tuple]:
    per: dict[str, list[float]] = defaultdict(list)
    for m in LINE_RE.finditer(text):
        cid = m.group("id")
        if filter_ids is not None and cid.lower() not in filter_ids:
            continue
        per[cid].append(float(m.group("ts")))

    rows = []
    for cid, ts in per.items():
        ts.sort()
        n = len(ts)
        if n < min_count:
            continue
        span = ts[-1] - ts[0]
        hz = (n - 1) / span if span > 0 else float("nan")
        dts = [ts[i + 1] - ts[i] for i in range(n - 1)]
        med_dt = median(dts)
        med_hz = 1.0 / med_dt if med_dt > 0 else float("nan")
        rows.append(
            (n, cid, span, hz, min(dts), max(dts), med_dt, med_hz, mean(dts))
        )

    rows.sort(key=lambda r: -r[0])
    if top > 0:
        rows = rows[:top]
    return rows


def main() -> int:
    args = parse_args()
    text, label = load_text(args)

    filter_ids: set[str] | None = None
    if args.ids.strip():
        filter_ids = {norm_hex(x) for x in args.ids.split(",") if x.strip()}

    print(f"Source: {label}")

    if not args.no_bus_load:
        bl = bus_load_from_candump_text(text, args.bitrate)
        if bl:
            br = args.bitrate
            # 列顺序对齐 canbusload: 帧/s、总 bit/s、payload bit/s、负载%%
            print(
                f" {args.iface}@{br}  {bl['fps']:.0f}  "
                f"{bl['bits_s_total']:.0f}  {bl['bits_s_payload']:.0f}  "
                f"{bl['load_pct']:.0f}%"
                f"   (CFL_WORSTCASE, {bl['span_s']:.2f}s / {bl['frames']} fr)"
            )
        else:
            print(
                "Bus load: skipped (need >=2 candump lines with [dlc] and time span >0)."
            )

    rows = analyze(text, filter_ids, args.min_count, args.top)

    if not rows:
        print("No matching frames. Check log format (candump -ta) or --ids filter.")
        return 0

    print(
        "id(hex)  frames  span_s    avg_Hz  min_dt_ms  max_dt_ms  med_dt_ms  med_Hz  mean_dt_ms"
    )
    for n, cid, span, hz, mindt, maxdt, meddt, medhz, meandt in rows:
        try:
            hx = f"0x{int(cid, 16):X}"
        except ValueError:
            hx = cid
        print(
            f"{hx:>7}  {n:6d}  {span:8.4f}  {hz:8.1f}  "
            f"{mindt*1000:9.3f}  {maxdt*1000:9.3f}  {meddt*1000:9.3f}  "
            f"{medhz:6.1f}  {meandt*1000:10.3f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
