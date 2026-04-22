#!/usr/bin/env python3
import argparse
import os
import threading
import time
import sys

from evdev import ecodes
from loguru import logger
from common.controllers import BaseControllerAdapter, TorsoControllerAdapter
from common.input_utils import find_joystick, normalize_axis
from HP_timer import HighPrecisionTimer

# loguru supports {file} and {line} placeholders
# Remove the default stderr handler, then re-add our own formatting.
logger.remove()
logger.add(
    sys.stderr,
    colorize=True,
    format="<green>{time:YYYY-MM-DD HH:mm:ss.SSS}</green> | <cyan>{file}:{line}</cyan> | {level} | {message}",
)
logger.add(
    "joystick.log",
    rotation="100 MB",
    retention="10 days",
    format="{time:YYYY-MM-DD HH:mm:ss.SSS} | {file}:{line} | {level} | {message}",
)

CUR_DIR = os.path.dirname(os.path.abspath(__file__))

# 底盘 DS402：node 1=左轮, 2=后轮, 3=右轮（与 oni_ctrl.LeKiwiBaseController 一致）
_BASE_NODE_ALLOWED = frozenset({1, 2, 3})


def parse_base_node_ids(s: str) -> tuple[int, ...]:
    """解析 --base-nodes，去重并保持首次出现顺序。"""
    seen: set[int] = set()
    out: list[int] = []
    for part in s.split(","):
        part = part.strip()
        if not part:
            continue
        n = int(part, 10)
        if n not in _BASE_NODE_ALLOWED:
            raise ValueError(
                f"Invalid base node id {n}: must be one of {sorted(_BASE_NODE_ALLOWED)} "
                "(1=left, 2=back, 3=right)"
            )
        if n not in seen:
            seen.add(n)
            out.append(n)
    if not out:
        raise ValueError("base-nodes: need at least one id, e.g. 1,2,3 or 2")
    return tuple(out)


class UnifiedJoystickTeleop:
    def __init__(
        self,
        mode: str,
        base_velocity_mode: str = "PV",
        *,
        base_active_node_ids: tuple[int, ...] = (1, 2, 3),
        apply_uniform_profile_dynamics: bool = True,
        base_profile_accel_pulses_s2: int = 600_000,
        base_profile_decel_pulses_s2: int = 600_000,
        motor_diagnostics_debug: bool = False,
    ):
        self.mode = mode
        self.base_velocity_mode = base_velocity_mode.upper()
        self.base_active_node_ids = base_active_node_ids
        self.enable_base = mode in ("base", "both")
        self.enable_torso = mode in ("torso", "both")

        self.dev = find_joystick()

        # shared joystick state
        self.lx = 0.0
        self.ly = 0.0
        self.rx = 0.0
        self.btn_tr = False
        self.hat0y = 0
        self.running = True
        # 底盘：仅在松开组合键瞬间发一次 0 速，避免未按住时每周期重复写 0
        self._base_btn_tr_prev = False

        # base params
        self.linear_scale = 0.1
        self.angular_scale = -15.0  # deg/s
        self.deadzone = 0.1
        self.base = None

        # throttle axis logging (avoid huge stdout)
        self._last_axis_log_t = 0.0
        self._axis_log_period_s = 0.5  # 2Hz

        # MDX+ 0x6041 轮询：Warning(bit7)/Fault 等时 SDO 读 0x603F/0x1001/0x200F（见 oni_ctrl）
        self._mdx_status_poll_last = 0.0
        self._mdx_status_poll_period_s = 0.5  # 10 Hz，仅读 TPDO 缓存；SDO 在 oni 内按轴节流
        self._mdx_diag_first_poll_logged = False

        # control loop period (wheel RPDO command rate); absolute-time pacing in control_loop
        self._control_period_ms = 100.0
        self.high_precision_timer = HighPrecisionTimer()

        # torso controller
        self.torso = None

        if self.enable_base:
            self.base = BaseControllerAdapter(
                CUR_DIR,
                base_velocity_mode=base_velocity_mode,
                base_active_node_ids=base_active_node_ids,
                apply_uniform_profile_dynamics=apply_uniform_profile_dynamics,
                base_profile_accel_pulses_s2=base_profile_accel_pulses_s2,
                base_profile_decel_pulses_s2=base_profile_decel_pulses_s2,
                motor_diagnostics_debug=motor_diagnostics_debug,
            )
            logger.info(
                "[BASE] connected (nodes={}, base_velocity_mode={}, motor_diagnostics_debug={})",
                list(base_active_node_ids),
                self.base_velocity_mode,
                motor_diagnostics_debug,
            )

        if self.enable_torso:
            self.torso = TorsoControllerAdapter()
            logger.info("[TORSO] initialized in PV mode")

        self.ctrl_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.ctrl_thread.start()

    def control_loop(self):
        t = self.high_precision_timer
        cycle_ns = int(round(self._control_period_ms * 1_000_000))
        dt = t.timespec()
        dt.tv_sec, rem = divmod(cycle_ns, 1_000_000_000)
        dt.tv_nsec = rem

        current_time = t.clock_gettime(t.CLOCK_REALTIME)
        next_time = current_time

        while self.running:
            try:
                if self.enable_base and self.base is not None:
                    if self.btn_tr:
                        lx = 0 if abs(self.lx) < self.deadzone else self.lx
                        ly = 0 if abs(self.ly) < self.deadzone else self.ly
                        rx = 0 if abs(self.rx) < self.deadzone else self.rx

                        x = -lx * self.linear_scale
                        y = ly * self.linear_scale
                        theta = rx * self.angular_scale
                        self.base.set_body_velocity(x, y, theta)
                    elif self._base_btn_tr_prev:
                        # 下降沿：只发一次停
                        self.base.set_body_velocity(0.0, 0.0, 0.0)
                    self._base_btn_tr_prev = self.btn_tr
            except Exception:
                pass

            try:
                if self.enable_torso and self.torso is not None:
                    self.torso.update_by_buttons(self.btn_tr, self.hat0y)
            except Exception:
                pass

            try:
                if self.enable_base and self.base is not None:
                    mt = time.monotonic()
                    if mt - self._mdx_status_poll_last >= self._mdx_status_poll_period_s:
                        self.base.poll_mdx_diagnostics_if_needed(mt)
                        self._mdx_status_poll_last = mt
                        if not self._mdx_diag_first_poll_logged:
                            self._mdx_diag_first_poll_logged = True
                            logger.info("[DIAG_POLL] first poll OK at monotonic={:.3f}", mt)
            except Exception as exc:
                logger.warning("[DIAG_POLL] exception: {}", exc)

            try:
                next_time = t.timespec_add(next_time, dt)
                t.clock_nanosleep(t.CLOCK_REALTIME, t.TIMER_ABSTIME, next_time)
            except OSError as e:
                logger.warning(
                    "clock_nanosleep failed: %s; fallback sleep and re-phase next_time", e
                )
                time.sleep(self._control_period_ms / 1000.0)
                current_time = t.clock_gettime(t.CLOCK_REALTIME)
                next_time = current_time

    def listen(self):
        logger.info("=" * 60)
        logger.info(f"Unified joystick teleop mode: {self.mode}")
        if self.enable_base:
            logger.info("Base: hold BTN_TR + ABS_X/ABS_Y/ABS_Z -> vx/vy/wz; release TR -> stop")
            logger.info(
                "Base active node_id(s): {} (1=left, 2=back, 3=right); unset wheels are not on CANopen",
                list(self.base_active_node_ids),
            )
            _6060 = "3 (PV)" if self.base_velocity_mode == "PV" else "9 (CSV)"
            logger.info(f"Base CiA402 velocity mode: {self.base_velocity_mode} (0x6060={_6060})")
        if self.enable_torso:
            logger.info("Torso: BTN_TR + HAT0Y(-1/1) -> UP/DOWN; release TR -> STOP")
        logger.info("=" * 60)

        for event in self.dev.read_loop():
            if not self.running:
                break

            if event.type == ecodes.EV_ABS:
                code, val = event.code, event.value
                if code in (ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_Z):
                    norm = normalize_axis(val)

                    now_t = time.monotonic()
                    # 与底盘组合键一致：仅按住 BTN_TR 时打印摇杆轴日志（避免未组合时刷屏）
                    if self.btn_tr and (now_t - self._last_axis_log_t) >= self._axis_log_period_s:
                        if code == ecodes.ABS_Y:
                            direction = "back" if norm >= 0 else "front"
                            axis_name = "ABS_Y"
                        elif code == ecodes.ABS_X:
                            direction = "right" if norm >= 0 else "left"
                            axis_name = "ABS_X"
                        else:  # ABS_Z
                            direction = "right_turn" if norm >= 0 else "left_turn"
                            axis_name = "ABS_Z"

                        logger.info(
                            f"[AXIS] {axis_name:<7} raw={val:<4d} norm={norm:+.3f} ({direction})"
                        )
                        self._last_axis_log_t = now_t

                    if code == ecodes.ABS_X:
                        self.ly = -norm
                    elif code == ecodes.ABS_Y:
                        
                        self.lx = norm
                    elif code == ecodes.ABS_Z:
                        self.rx = norm
                elif code == ecodes.ABS_HAT0Y:
                    self.hat0y = int(val)

            elif event.type == ecodes.EV_KEY and event.code == ecodes.BTN_TR:
                if event.value == 1:
                    self.btn_tr = True
                elif event.value == 0:
                    self.btn_tr = False

    def shutdown(self):
        self.running = False
        time.sleep(0.05)

        if self.enable_torso and self.torso is not None:
            try:
                self.torso.shutdown()
            except Exception:
                pass

        if self.enable_base and self.base is not None:
            try:
                self.base.shutdown()
            except Exception:
                pass


def parse_args():
    parser = argparse.ArgumentParser(description="Unified joystick teleop for base/torso")
    parser.add_argument(
        "--mode",
        choices=["base", "torso", "both"],
        default="base",
        help="control target: base, torso, or both",
    )
    parser.add_argument(
        "--base-velocity-mode",
        choices=["pv", "csv"],
        default="pv",
        help=(
            "Omni base CiA402 mode: pv=Profile Velocity (0x6060=3), default unchanged; "
            "csv=Cyclic Synchronous Velocity (9), use with SYNC (see PDO_MAP / bus)."
        ),
    )
    parser.add_argument(
        "--no-base-profile-dynamics",
        action="store_true",
        help="Do not SDO-write 0x6083/0x6084 on connect (keep drive-stored values).",
    )
    parser.add_argument(
        "--base-profile-accel",
        type=int,
        default=600_000,
        metavar="PULSES_S2",
        help="CiA402 0x6083 planning accel (Pulses/s^2), applied to all three base nodes. Default 3600000.",
    )
    parser.add_argument(
        "--base-profile-decel",
        type=int,
        default=600_000,
        metavar="PULSES_S2",
        help="CiA402 0x6084 planning decel (Pulses/s^2), applied to all three base nodes. Default 3600000.",
    )
    parser.add_argument(
        "--motor-diag-debug",
        action="store_true",
        help="SW 正常时也周期打印诊断 SDO (DEBUG 级别日志)。",
    )
    parser.add_argument(
        "--base-nodes",
        type=str,
        default="1,2,3",
        metavar="IDS",
        help=(
            "Comma-separated DS402 node IDs to connect for the base (subset of 1,2,3 = left,back,right). "
            "Example: --base-nodes 2 or --base-nodes 1,3. Default: all three."
        ),
    )
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        base_nodes = parse_base_node_ids(args.base_nodes)
    except ValueError as e:
        print(f"Invalid --base-nodes: {e}", file=sys.stderr)
        sys.exit(2)
    ctl = UnifiedJoystickTeleop(
        args.mode,
        base_velocity_mode=args.base_velocity_mode.upper(),
        base_active_node_ids=base_nodes,
        apply_uniform_profile_dynamics=not args.no_base_profile_dynamics,
        base_profile_accel_pulses_s2=args.base_profile_accel,
        base_profile_decel_pulses_s2=args.base_profile_decel,
        motor_diagnostics_debug=args.motor_diag_debug,
    )
    try:
        ctl.listen()
    except KeyboardInterrupt:
        logger.info("Stopping...")
    finally:
        ctl.shutdown()


if __name__ == "__main__":
    main()

