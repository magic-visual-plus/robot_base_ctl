#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
进程监控；日志默认写入 tests/monitor.log，单文件超过 --max-mb 后自动轮转为 monitor.log.1、.2 ...

推荐启动（无需再 shell 重定向到 monitor.log）:
  nohup python /path/to/tests/monitor.py &
指定日志路径与 10MB 轮转:
  nohup python /path/to/tests/monitor.py --log /path/to/monitor.log --max-mb 10 &
"""

import argparse
import logging
import sys
import time
from datetime import datetime
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Optional

import psutil

TOP_N = 5
INTERVAL = 2  # 秒
DEFAULT_MAX_MB = 10
DEFAULT_BACKUP_COUNT = 5


def bytes_to_human(n):
    units = ["B", "KB", "MB", "GB", "TB"]
    size = float(n)
    for unit in units:
        if size < 1024:
            return f"{size:.2f} {unit}"
        size /= 1024
    return f"{size:.2f} PB"


def get_memory_info():
    mem = psutil.virtual_memory()
    return {
        "total": mem.total,
        "used": mem.used,
        "available": mem.available,
        "percent": mem.percent,
    }


def collect_processes():
    processes = []

    for proc in psutil.process_iter([
        "pid", "name", "username", "memory_info"
    ]):
        try:
            info = proc.info
            processes.append({
                "pid": info["pid"],
                "name": info["name"] or "N/A",
                "username": info["username"] or "N/A",
                "memory_rss": info["memory_info"].rss if info["memory_info"] else 0,
                "cpu_percent": proc.cpu_percent(interval=None),
            })
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            continue

    return processes


def format_report(
    processes,
    mem_info,
    peak_used: Optional[int] = None,
    peak_time: Optional[str] = None,
) -> str:
    lines = []
    lines.append("=" * 100)
    now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    lines.append(f"Time: {now_str}")
    lines.append(
        f"Memory: used {bytes_to_human(mem_info['used'])} / total {bytes_to_human(mem_info['total'])} "
        f"({mem_info['percent']}%), available {bytes_to_human(mem_info['available'])}"
    )
    if peak_used is not None and peak_time:
        lines.append(
            f"Memory peak (historical max): used {bytes_to_human(peak_used)} at {peak_time}"
        )
    lines.append("-" * 100)
    lines.append(f"{'PID':<8}{'CPU%':>8}  {'MEM':>12}  {'USER':<18}  NAME")
    lines.append("-" * 100)

    for p in processes[:TOP_N]:
        lines.append(
            f"{p['pid']:<8}{p['cpu_percent']:>8.1f}  "
            f"{bytes_to_human(p['memory_rss']):>12}  "
            f"{p['username'][:18]:<18}  {p['name']}"
        )
    lines.append("")
    return "\n".join(lines)


def print_report(
    processes,
    mem_info,
    log: logging.Logger,
    peak_used: Optional[int],
    peak_time: Optional[str],
):
    text = format_report(processes, mem_info, peak_used=peak_used, peak_time=peak_time)
    if sys.stdout.isatty():
        print("\033[2J\033[H", end="")
        print(text, end="")
    log.info(text)


def main():
    parser = argparse.ArgumentParser(description="Monitor top CPU processes (optional rotating log file).")
    parser.add_argument(
        "--log",
        type=str,
        default=None,
        help="Log file path; single file grows until max-mb then rotates to .1 .2 ... (default: script dir monitor.log)",
    )
    parser.add_argument(
        "--max-mb",
        type=float,
        default=DEFAULT_MAX_MB,
        help=f"Rotate log when size exceeds this many MB (default: {DEFAULT_MAX_MB})",
    )
    parser.add_argument(
        "--backup-count",
        type=int,
        default=DEFAULT_BACKUP_COUNT,
        help=f"Number of rotated backup files to keep (default: {DEFAULT_BACKUP_COUNT})",
    )
    args = parser.parse_args()

    log_path = Path(args.log) if args.log else Path(__file__).resolve().parent / "monitor.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    max_bytes = int(max(1, args.max_mb) * 1024 * 1024)

    log = logging.getLogger("monitor")
    log.setLevel(logging.INFO)
    log.handlers.clear()
    # backupCount=0 时 Python 不会轮转，至少保留 1 个备份文件
    backup_count = max(1, int(args.backup_count))
    fh = RotatingFileHandler(
        log_path,
        maxBytes=max_bytes,
        backupCount=backup_count,
        encoding="utf-8",
    )
    fh.setFormatter(logging.Formatter("%(message)s"))
    log.addHandler(fh)
    log.propagate = False

    # 启动信息写入日志
    log.info(
        f"monitor started | log={log_path} | rotate at {max_bytes // (1024 * 1024)} MB | backups={backup_count}"
    )
    if sys.stdout.isatty():
        print(f"Logging to {log_path} (rotate at {args.max_mb} MB, keep {backup_count} backups)")

    print("Initializing process CPU counters...", file=sys.stderr if not sys.stdout.isatty() else sys.stdout)
    collect_processes()
    time.sleep(1)

    peak_used: int = 0
    peak_time: Optional[str] = None

    while True:
        processes = collect_processes()
        mem_info = get_memory_info()

        used = mem_info["used"]
        if used > peak_used:
            peak_used = used
            peak_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        top_cpu = sorted(processes, key=lambda x: x["cpu_percent"], reverse=True)

        print_report(top_cpu, mem_info, log, peak_used=peak_used, peak_time=peak_time)

        time.sleep(INTERVAL)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.", file=sys.stderr)
