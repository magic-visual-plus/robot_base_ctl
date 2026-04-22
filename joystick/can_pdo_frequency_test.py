#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CANopen PDO / SYNC 频率测试（基于本目录 Ds402_ctl.py）

用途
----
1. 在代码侧统计：
   - RPDO 发送次数（通过 monkeypatch safe_transmit，与 Ds402_ctl 写速度路径一致）
   - TPDO1（statusword）与 TPDO4（pos/vel 反馈）回调次数（每轴独立计数）
2. 与 candump / cansend 对照，确认总线上真实报文周期。

运行示例
--------
  cd /opt/project/robot_base_ctl/joystick
  python3 can_pdo_frequency_test.py --duration 5 --cmd-hz 50

  # 不启动主站 SYNC（connect 后立刻 stop）
  python3 can_pdo_frequency_test.py --no-sync --duration 5 --cmd-hz 50

  # 自定义 SYNC 周期（秒），例如 50ms -> 20Hz
  python3 can_pdo_frequency_test.py --sync-period 0.05 --duration 5 --cmd-hz 100

  # 底盘不要动、只测 TPDO 回调频率（测量段内不再发循环 RPDO）
  python3 can_pdo_frequency_test.py --no-cmd-loop --duration 5

命令行 CAN 工具（需另开终端，且勿与占用 can0 的进程冲突）
--------------------------------------------------------
  # 监听 SYNC：CiA301 默认 COB-ID 0x080（11-bit）
  candump can0 | grep '080'

  # 看某节点 TPDO/RPDO（COB-ID 与 PDO 映射有关，以 candump 全量为准后再 grep）
  candump can0 | grep -E '181|281|381|481'

  # 仅发一帧 SYNC（若总线上无其它 SYNC 生产者，可观察从站是否依赖 SYNC）
  cansend can0 080#

说明
----
- Ds402_ctl.CanopenBus.connect() 默认 ``net.sync.start(PDO_MAP.sync_period_s)``（现默认 5ms
  ≈200Hz），由本 Network 在 CAN 上发 SYNC。这与「应用层多少次调用 set_target_velocity」
  无直接关系；RxPDO 是否在 SYNC 边沿进应用取决于从站 RPDO transmission type（init 里
  已把速度/位置 RPDO 配成同步型）与厂家实现。
- TPDO event_timer + inhibit（init 里约 5ms）影响从站**上行**节奏；本脚本统计的是
  python-canopen 解析 TPDO 后触发回调的次数，应与 candump 中对应 TPDO 频率一致
  （在总线不丢帧时）。

底盘不动 / 安全测试
------------------
- 默认 ``--velocity 0``：目标速度为 0，正常不应转动；但轴仍处于 **PV + Operation enabled**，
  可能有保持力矩，异常参数或驱动器故障时仍可能有位移。
- 硬件侧仍建议：轮子离地、急停有效、或断开动力，再测 CAN。
- 若**只关心 TPDO 反馈频率**、尽量**少发 RPDO**：使用 ``--no-cmd-loop``（见下），
  测量窗口内本程序**不再循环写速度**，RPDO 计数应接近 0。
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List

# 与同目录 Ds402_ctl 一致
import Ds402_ctl as ds402
from Ds402_ctl import CanopenBus, MotorManager, decode_ds402_state, PDO_MAP


@dataclass
class FreqStats:
    rpdo_transmit_total: int = 0
    tpdo_status: Dict[int, int] = field(default_factory=dict)
    tpdo_fb: Dict[int, int] = field(default_factory=dict)


def _default_eds_path() -> str:
    return os.path.normpath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "hardware",
            "canopen",
            "eds",
            "CANOPEN-EDS-MBDV-Servo-SingleAxis-V1.1.1.eds",
        )
    )


def _patch_safe_transmit(stats: FreqStats) -> Callable[..., None]:
    orig = ds402.safe_transmit

    def wrapped(pdo, retries: int = 80, backoff_s: float = 0.003):
        stats.rpdo_transmit_total += 1
        return orig(pdo, retries=retries, backoff_s=backoff_s)

    ds402.safe_transmit = wrapped  # type: ignore[assignment]
    return orig


def _attach_tpdo_counters(mgr: MotorManager, stats: FreqStats) -> None:
    for nid, m in mgr.motors.items():
        stats.tpdo_status[nid] = 0
        stats.tpdo_fb[nid] = 0
        cfg = m.io.cfg
        tpdo_sw = m.io.node.tpdo[cfg.tpdo_status_num]
        tpdo_fb = m.io.node.tpdo[cfg.tpdo_fb_num]

        def on_sw(_map, axis_id: int = nid) -> None:
            stats.tpdo_status[axis_id] += 1

        def on_fb(_map, axis_id: int = nid) -> None:
            stats.tpdo_fb[axis_id] += 1

        tpdo_sw.add_callback(on_sw)
        tpdo_fb.add_callback(on_fb)


def _apply_sync(bus: CanopenBus, sync_period_sec: float | None) -> None:
    """
    sync_period_sec is None -> 不改：保留 connect() 已启动的 SYNC（默认 PDO_MAP.sync_period_s）。
    sync_period_sec == 0 -> 仅 stop，本主站不再发 SYNC。
    sync_period_sec > 0 -> stop 后按给定周期 restart。
    """
    if sync_period_sec is None:
        return
    try:
        bus.net.sync.stop()
    except Exception:
        pass
    if sync_period_sec <= 0:
        return
    bus.net.sync.start(float(sync_period_sec))


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Measure RPDO transmit rate vs TPDO callback rate (joystick/Ds402_ctl stack). "
            "Default velocity=0; use --no-cmd-loop to avoid periodic RPDO during measurement."
        )
    )
    parser.add_argument("--channel", default="can0", help="SocketCAN device")
    parser.add_argument("--bitrate", type=int, default=1_000_000)
    parser.add_argument("--eds", default="", help="EDS path (default: ../hardware/.../SingleAxis eds)")
    parser.add_argument("--nodes", default="1,2,3", help="Comma-separated node IDs")
    parser.add_argument("--duration", type=float, default=5.0, help="Test loop duration (s)")
    parser.add_argument("--cmd-hz", type=float, default=50.0, help="Target set_velocities_rad_s loop rate")
    parser.add_argument(
        "--sync-period",
        type=float,
        default=-1.0,
        metavar="SEC",
        help="SYNC period in seconds after connect. Default -1: keep connect() default (PDO_MAP.sync_period_s). "
        "Use 0 with --no-sync to disable.",
    )
    parser.add_argument(
        "--no-sync",
        action="store_true",
        help="Stop master SYNC after connect (same as --sync-period 0)",
    )
    parser.add_argument("--encoder-cpr", type=int, default=2**16)
    parser.add_argument("--gear-ratio", type=float, default=10.0)
    parser.add_argument(
        "--velocity",
        type=float,
        default=0.0,
        help="rad/s per axis during cmd loop (default 0: no intentional motion; still enabled in PV)",
    )
    parser.add_argument("--skip-motion", action="store_true", help="Alias for --velocity 0")
    parser.add_argument(
        "--no-cmd-loop",
        action="store_true",
        help=(
            "After enable: send one zero-velocity command, then only sleep for --duration. "
            "Measurement window has ~no RPDO from this script (TPDO-only rate test). "
            "Best for 'chassis must not move' + minimal bus load."
        ),
    )
    args = parser.parse_args()

    eds = args.eds.strip() or _default_eds_path()
    if not os.path.isfile(eds):
        print(f"EDS not found: {eds}", file=sys.stderr)
        return 1

    node_ids: List[int] = [int(x.strip()) for x in args.nodes.split(",") if x.strip()]
    if not node_ids:
        print("No node IDs", file=sys.stderr)
        return 1

    vel = 0.0 if args.skip_motion else float(args.velocity)

    stats = FreqStats()
    orig_safe_transmit = _patch_safe_transmit(stats)

    bus = CanopenBus(
        channel=args.channel,
        bitrate=args.bitrate,
        interface="socketcan",
        sync_period_s=PDO_MAP.sync_period_s,
        auto_sync=True,
    )
    mgr = MotorManager(bus)

    if args.no_sync:
        sync_period: float | None = 0.0
    elif args.sync_period < 0:
        sync_period = None
    else:
        sync_period = float(args.sync_period)

    try:
        bus.connect(can_filters=None)
        _apply_sync(bus, sync_period)

        for nid in node_ids:
            mgr.add_motor(nid, eds, encoder_cpr=args.encoder_cpr, gear_ratio=args.gear_ratio)

        mgr.init_all(default_mode="PV", zero_on_init=True)
        mgr.enable_all(timeout_s=8.0)
        mgr.set_mode_all("PV")

        _attach_tpdo_counters(mgr, stats)

        dt_cmd = 1.0 / max(args.cmd_hz, 1.0)
        n_loops = 0

        if args.no_cmd_loop:
            # 测量窗口前先发 0 速并略等，使轴上目标为 0；不计入下方统计窗口
            mgr.set_velocities_rad_s({nid: 0.0 for nid in node_ids})
            time.sleep(0.05)

        # 仅统计稳态测试段，不含 init/enable（及上面的预置零速）
        stats.rpdo_transmit_total = 0
        for nid in node_ids:
            stats.tpdo_status[nid] = 0
            stats.tpdo_fb[nid] = 0

        t0 = time.perf_counter()
        t_end = t0 + args.duration

        mode = "no-cmd-loop (TPDO only)" if args.no_cmd_loop else f"cmd_hz={args.cmd_hz} vel={vel}"
        print(
            f"[test] nodes={node_ids} duration={args.duration}s mode={mode} "
            f"sync_period_sec={sync_period!r} (None=default 0.1 in connect)"
        )
        if args.no_cmd_loop:
            time.sleep(max(0.0, t_end - time.perf_counter()))
        else:
            while time.perf_counter() < t_end:
                cmd = {nid: vel for nid in node_ids}
                mgr.set_velocities_rad_s(cmd)
                n_loops += 1
                time.sleep(dt_cmd)

        elapsed = time.perf_counter() - t0
        mgr.set_velocities_rad_s({nid: 0.0 for nid in node_ids})
        time.sleep(0.05)

        # 报告
        print(f"\n=== Wall time (measured) ~{elapsed:.3f}s, command iterations={n_loops} ===")
        if args.no_cmd_loop:
            print("Expected RPDO tx from this script during window: ~0 (no-cmd-loop)")
        else:
            print(
                f"Expected RPDO tx if {len(node_ids)} axes/iter (library serial): "
                f"~{n_loops * len(node_ids)} (loops * axes)"
            )
        print(f"Counted safe_transmit (RPDO/vel path + any other PDO using it): {stats.rpdo_transmit_total}")
        print(f"Implied RPDO rate (total): {stats.rpdo_transmit_total / elapsed:.1f} Hz")

        for nid in node_ids:
            sw_n = stats.tpdo_status.get(nid, 0)
            fb_n = stats.tpdo_fb.get(nid, 0)
            print(
                f"  axis {nid}: TPDO status callbacks={sw_n} ({sw_n / elapsed:.1f} Hz), "
                f"TPDO fb callbacks={fb_n} ({fb_n / elapsed:.1f} Hz)"
            )

        # 可选：读一眼状态
        nid0 = node_ids[0]
        fb0 = mgr.motors[nid0].feedback()
        print(
            f"\nSample axis {nid0}: SW=0x{fb0.statusword:04X} {decode_ds402_state(fb0.statusword)}"
        )

        print("\n[candump] 提示: SYNC 常为 080；具体 TPDO/RPDO COB-ID 请全量 candump 后对照 EDS。")
        mgr.estop_quiet_all(stage_sleep_s=0.05, quiet_pdo=True)
    except KeyboardInterrupt:
        print("Interrupted", file=sys.stderr)
        try:
            mgr.estop_quiet_all(stage_sleep_s=0.05, quiet_pdo=True)
        except Exception:
            pass
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        try:
            mgr.estop_quiet_all(stage_sleep_s=0.05, quiet_pdo=True)
        except Exception:
            pass
        return 1
    finally:
        ds402.safe_transmit = orig_safe_transmit  # type: ignore[assignment]
        try:
            bus.disconnect()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
