from __future__ import annotations

from loguru import logger
from dataclasses import dataclass
from typing import Dict, Literal, Optional

import math
import time
import numpy as np
import os

# ✅ 导入“我们自己的电机控制代码”（把你之前那份 DS402 多轴脚本保存成这个模块名）
# 里面需要有：CanopenBus, MotorManager
from Ds402_ctl import (
    CanopenBus,
    Ds402State,
    MotorManager,
    PDO_MAP,
    ControlModeName,
    MODE_MAP,
    decode_ds402_state,
    safe_sdo_read,
)
from motor_diagnostic_config import (
    format_motor_diagnostic_sdo_line,
    load_motor_diagnostic_profile_safe,
    resolve_motor_diagnostics_json_path,
)


def _mdx_statusword_triggers_diagnostic_sdo(sw: int) -> tuple[bool, str]:
    """
    MDX+ 手册 0x6041：bit7=Warning，bit3=Fault；异常 PDS 时读 0x603F / 0x1001 / 0x200F。
    （与 candump_tpdo1_statusword 触发条件一致。）
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


@dataclass
class LeKiwiBaseConfig:
    """Configuration parameters required to operate the base (DS402 CANopen wheels)."""

    # CANopen
    can_channel: str = "can0"
    bitrate: int = 1_000_000
    eds_path: str = os.path.normpath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "hardware",
            "canopen",
            "eds",
            "CANOPEN-EDS-MBDV-Servo-SingleAxis-V1.1.1.eds",
        )
    )

    # motor encoder/gearing (for rad/s<->counts conversion inside DS402 module)
    encoder_cpr: int = 2 ** 16
    gear_ratio: float = 10.0  # you said: output one rev = 2^16 * 10 counts

    # base geometry
    wheel_radius: float = 0.1015   # [m]
    base_radius: float = 0.203   # [m] distance from center to wheel contact

    # safety limits (output-shaft wheel angular speed limit, rad/s)
    max_wheel_rad_s: float = 2.0 * math.pi  # default 1 rev/s

    # Master SYNC (s); None -> use PDO_MAP.sync_period_s (default 5ms with RPDO sync)
    sync_period_s: Optional[float] = None
    auto_sync: bool = True

    # CiA402 0x6060 for three omni wheels: PV=3 (default, unchanged), CSV=9 (needs SYNC + sync RPDO)
    base_velocity_mode: Literal["PV", "CSV"] = "PV"

    # 三台轮电机统一动态（MDX+ 手册 0x6083 规划加速度、0x6084 规划减速度，UNSIGNED32，单位 Pulses/s^2）
    apply_uniform_profile_dynamics: bool = True
    base_profile_accel_pulses_s2: int = 3_600_000
    base_profile_decel_pulses_s2: int = 3_600_000

    # wheel direction multipliers (if some wheel is reversed, set -1)
    dir_left: int = +1
    dir_back: int = +1
    dir_right: int = +1

    # 状态字异常时批量 SDO 诊断：JSON 见 config/motor_diagnostics/（默认同川 tongchuan_mdx.json）
    motor_diagnostics_profile: str = "tongchuan_mdx"
    motor_diagnostics_config_path: Optional[str] = None

    # debug 模式：SW 正常时也周期打印诊断 SDO（与异常时共用 cooldown）
    motor_diagnostics_debug: bool = False

    # 仅连接并驱动指定 node_id：1=左轮, 2=后轮, 3=右轮。用于单电机/双电机台架测试；默认三台全开。
    base_active_node_ids: tuple[int, ...] = (1, 2, 3)


class LeKiwiBaseController:
    """
    Controller for the three-wheel omni base of LeKiwi (DS402 CANopen wheels).

    Public API:
      - set_body_velocity(x[m/s], y[m/s], theta[deg/s])
      - read_body_velocity() -> {'x.vel','y.vel','theta.vel'} (theta in deg/s)
      - stop()
      - read_wheel_positions_counts()
      - read_wheel_positions_rad()
    """

    # Node mapping (per your requirement)
    NODE_LEFT = 1
    NODE_BACK = 2
    NODE_RIGHT = 3

    def __init__(self, config: LeKiwiBaseConfig):
        self.config = config

        # name <-> node id
        self.name_to_node = {
            "base_left_wheel": self.NODE_LEFT,
            "base_back_wheel": self.NODE_BACK,
            "base_right_wheel": self.NODE_RIGHT,
        }
        self.node_to_name = {v: k for k, v in self.name_to_node.items()}
        self.base_motors = list(self.name_to_node.keys())

        _sync_s = config.sync_period_s if config.sync_period_s is not None else PDO_MAP.sync_period_s
        self._bus = CanopenBus(
            channel=config.can_channel,
            bitrate=config.bitrate,
            interface="socketcan",
            sync_period_s=_sync_s,
            auto_sync=config.auto_sync,
        )
        self._mgr = MotorManager(self._bus)
        self._connected = False
        self._mdx_diag_last_sdo_mono: Dict[int, float] = {}
        _diag_path = resolve_motor_diagnostics_json_path(
            explicit_path=config.motor_diagnostics_config_path,
            profile_id=config.motor_diagnostics_profile,
        )
        self._motor_diag_profile = load_motor_diagnostic_profile_safe(_diag_path)
        if self._motor_diag_profile.id != "empty":
            logger.info(
                "Motor diagnostics SDO profile: {} ({} entries) <- {}",
                self._motor_diag_profile.id,
                len(self._motor_diag_profile.entries),
                _diag_path,
            )

    def _validate_active_nodes(self) -> None:
        allowed = {self.NODE_LEFT, self.NODE_BACK, self.NODE_RIGHT}
        active = set(self.config.base_active_node_ids)
        if not active:
            raise ValueError("base_active_node_ids must be non-empty")
        if not active.issubset(allowed):
            raise ValueError(
                f"base_active_node_ids must be subset of {sorted(allowed)} "
                f"(1=left, 2=back, 3=right), got {sorted(active)}"
            )

    def _ordered_active_nodes(self) -> list[int]:
        """左→后→右顺序，与 config.base_active_node_ids 求交。"""
        order = [self.NODE_LEFT, self.NODE_BACK, self.NODE_RIGHT]
        want = set(self.config.base_active_node_ids)
        return [n for n in order if n in want]

    # ------------------------------------------------------------------
    # Connection & configuration
    # ------------------------------------------------------------------
    @property
    def is_connected(self) -> bool:
        return self._connected

    @staticmethod
    def _safe_sdo_write_u32(node, index: int, value: int) -> bool:
        try:
            node.sdo[index].raw = int(value) & 0xFFFFFFFF
            return True
        except Exception as e:
            logger.warning(f"SDO write 0x{index:04X} failed: {e}")
            return False

    def _apply_uniform_base_profile_dynamics_sdo(self) -> None:
        cfg = self.config
        if not cfg.apply_uniform_profile_dynamics:
            logger.info("Skipping uniform 0x6083/0x6084 (apply_uniform_profile_dynamics=False)")
            return
        acc = int(cfg.base_profile_accel_pulses_s2)
        dec = int(cfg.base_profile_decel_pulses_s2)
        axes = (
            ("left", self.NODE_LEFT),
            ("back", self.NODE_BACK),
            ("right", self.NODE_RIGHT),
        )
        for label, nid in axes:
            m = self._mgr.motors.get(nid)
            if m is None:
                continue
            node = m.io.node
            ok_a = self._safe_sdo_write_u32(node, 0x6083, acc)
            ok_d = self._safe_sdo_write_u32(node, 0x6084, dec)
            if ok_a and ok_d:
                logger.info(
                    f"Base motor node {nid} ({label}): set 0x6083={acc}, 0x6084={dec} Pulses/s^2"
                )
            else:
                logger.warning(
                    f"Base motor node {nid} ({label}): partial fail writing 0x6083/0x6084 "
                    f"(ok_6083={ok_a}, ok_6084={ok_d})"
                )

    def _log_base_profile_dynamics_sdo(self) -> None:
        """Read-back 0x6083 / 0x6084 after optional apply (MDX+ planning accel/decel)."""
        axes = (
            ("left", self.NODE_LEFT),
            ("back", self.NODE_BACK),
            ("right", self.NODE_RIGHT),
        )
        for label, nid in axes:
            m = self._mgr.motors.get(nid)
            if m is None:
                continue
            node = m.io.node
            a = safe_sdo_read(node, 0x6083, default=None)
            d = safe_sdo_read(node, 0x6084, default=None)
            if a is None or d is None:
                logger.warning(
                    f"Base motor node {nid} ({label}): SDO read 0x6083/0x6084 failed (6083={a}, 6084={d})"
                )
            else:
                logger.info(
                    f"Base motor node {nid} ({label}): readback 0x6083={a}, 0x6084={d} Pulses/s^2"
                )

    def connect(self) -> None:
        """Connect CANopen network and init/enable DS402 motors (PV or CSV per config)."""
        if self.is_connected:
            logger.info("LeKiwi base already connected")
            return

        self._bus.connect(can_filters=None)

        self._validate_active_nodes()
        active = self._ordered_active_nodes()
        for nid in active:
            self._mgr.add_motor(
                nid,
                self.config.eds_path,
                encoder_cpr=self.config.encoder_cpr,
                gear_ratio=self.config.gear_ratio,
            )
        logger.info(
            "LeKiwi base: connecting node_id(s) {} (1=left, 2=back, 3=right); full omni uses 1,2,3",
            active,
        )

        vm: ControlModeName = self.config.base_velocity_mode
        if vm not in ("PV", "CSV"):
            logger.warning("Invalid base_velocity_mode={!r}, using PV", vm)
            vm = "PV"

        # init and enable (PV path: same as before when vm == "PV")
        self._mgr.init_all(default_mode=vm, zero_on_init=True)
        self._mgr.enable_all(timeout_s=8.0)
        self._mgr.set_mode_all(vm)

        self._apply_uniform_base_profile_dynamics_sdo()
        self._log_base_profile_dynamics_sdo()

        self._connected = True
        logger.info(
            "LeKiwi base connected on {}, bitrate={}, base_velocity_mode={} (0x6060={}), active_nodes={}",
            self.config.can_channel,
            self.config.bitrate,
            vm,
            MODE_MAP[vm],
            self._ordered_active_nodes(),
        )

    def disconnect(self) -> None:
        if not self.is_connected:
            return
        try:
            self.stop()
            # best-effort emergency stop + quiet PDO
            self._mgr.estop_quiet_all(stage_sleep_s=0.10, quiet_pdo=True)
        finally:
            self._bus.disconnect()
            self._connected = False
            logger.info("LeKiwi base disconnected")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def set_body_velocity(self, x: float, y: float, theta: float) -> None:
        """Command body velocity (x, y in m/s, theta in deg/s)."""
        if not self.is_connected:
            raise RuntimeError("LeKiwi base is not connected")

        # wheel angular velocities in rad/s (output shaft)
        w = self._body_to_wheel_rad_s(x, y, theta)

        # apply per-wheel direction (if any wheel reversed)
        w["base_left_wheel"] *= self.config.dir_left
        w["base_back_wheel"] *= self.config.dir_back
        w["base_right_wheel"] *= self.config.dir_right

        cmd: Dict[int, float] = {}
        for nid in self._ordered_active_nodes():
            name = self.node_to_name[nid]
            cmd[nid] = float(w[name])
        self._mgr.set_velocities_rad_s(cmd)

    def stop(self) -> None:
        """Stop all connected wheels (rad/s -> 0)."""
        if not self.is_connected:
            return
        self._mgr.set_velocities_rad_s({nid: 0.0 for nid in self._mgr.motors})

    def poll_mdx_diagnostics_if_needed(self, now_mono: float, *, cooldown_s: float = 0.5) -> None:
        """
        轮询各轴 0x6041（经 Motor IO 缓存/TPDO）；若出现 Warning/Fault 等，则按
        motor_diagnostics_profile JSON 批量 SDO 读并打 WARNING（同一 node 受 cooldown_s 节流）。

        motor_diagnostics_debug=True 时，SW 正常也会按同样 cooldown 周期打印 DEBUG 日志。
        """
        if not self.is_connected:
            return
        debug = self.config.motor_diagnostics_debug
        for nid in (self.NODE_LEFT, self.NODE_BACK, self.NODE_RIGHT):
            m = self._mgr.motors.get(nid)
            if m is None:
                continue
            try:
                sw = m.io.read_statusword()
            except Exception:
                continue
            need, why = _mdx_statusword_triggers_diagnostic_sdo(sw)
            if not need and not debug:
                continue
            if now_mono - self._mdx_diag_last_sdo_mono.get(nid, 0.0) < cooldown_s:
                continue
            self._mdx_diag_last_sdo_mono[nid] = now_mono
            name = self.node_to_name.get(nid, f"node{nid}")
            try:
                line = format_motor_diagnostic_sdo_line(m.io.node, self._motor_diag_profile)
            except Exception as exc:
                logger.warning("[MDX_DIAG] {} node{} SDO read failed: {}", name, nid, exc)
                continue
            if need:
                logger.warning(
                    "[MDX_DIAG] {} node{} SW=0x{:04X} ({}) | {}",
                    name, nid, sw, why, line,
                )
            else:
                st = decode_ds402_state(sw)
                logger.info(
                    "[MDX_DIAG_DBG] {} node{} SW=0x{:04X} ({}) | {}",
                    name, nid, sw, st, line,
                )

    def read_body_velocity(self) -> Dict[str, float]:
        """Read wheel velocities and convert back to body-frame velocity."""
        if not self.is_connected:
            raise RuntimeError("LeKiwi base is not connected")

        def _wheel_vel(nid: int, dir_mul: int) -> float:
            m = self._mgr.motors.get(nid)
            if m is None:
                return 0.0
            fb = m.feedback()
            v = fb.velocity_rad_s or 0.0
            return float(v * dir_mul)

        wl = _wheel_vel(self.NODE_LEFT, int(self.config.dir_left))
        wb = _wheel_vel(self.NODE_BACK, int(self.config.dir_back))
        wr = _wheel_vel(self.NODE_RIGHT, int(self.config.dir_right))

        return self._wheel_rad_s_to_body(wl, wb, wr)

    def read_wheel_positions_counts(self) -> Dict[str, int]:
        """Read wheel positions (raw encoder counts, whatever drive reports)."""
        if not self.is_connected:
            raise RuntimeError("LeKiwi base is not connected")
        out: Dict[str, int] = {}
        for name, nid in self.name_to_node.items():
            m = self._mgr.motors.get(nid)
            if m is None:
                out[name] = 0
                continue
            fb = m.feedback()
            out[name] = int(fb.position_counts or 0)
        return out

    def read_wheel_positions_rad(self) -> Dict[str, float]:
        """Read wheel positions converted to output shaft radians (best effort)."""
        if not self.is_connected:
            raise RuntimeError("LeKiwi base is not connected")
        out: Dict[str, float] = {}
        for name, nid in self.name_to_node.items():
            m = self._mgr.motors.get(nid)
            if m is None:
                out[name] = 0.0
                continue
            fb = m.feedback()
            out[name] = float(fb.position_rad or 0.0)
        return out

    def drive_body_displacement(
        self,
        x: float = 0.0,
        y: float = 0.0,
        theta: float = 0.0,
        linear_speed: float = 0.2,
        angular_speed: float = 30.0,
        dt: float = 0.05,
    ) -> None:
        """Drive a nominal displacement using simple velocity integration."""
        if not self.is_connected:
            raise RuntimeError("LeKiwi base is not connected")

        def _run_segment(target: float, axis: str, speed: float) -> None:
            if abs(target) < 1e-6:
                return

            start_positions = self.read_wheel_positions_counts()
            logger.info(f"[{axis}] start_positions={start_positions}")

            direction = 1.0 if target > 0 else -1.0
            remaining = abs(target)

            if axis == "x":
                vx, vy, wz = direction * speed, 0.0, 0.0
            elif axis == "y":
                vx, vy, wz = 0.0, direction * speed, 0.0
            elif axis == "theta":
                vx, vy, wz = 0.0, 0.0, direction * speed
            else:
                raise ValueError(f"Unknown axis '{axis}'")

            self.set_body_velocity(vx, vy, wz)

            last_time = time.perf_counter()
            travelled = 0.0
            try:
                while travelled < remaining:
                    time.sleep(dt)
                    now = time.perf_counter()
                    actual_dt = now - last_time
                    last_time = now

                    feedback = self.read_body_velocity()
                    if axis == "x":
                        component = feedback["x.vel"]
                    elif axis == "y":
                        component = feedback["y.vel"]
                    else:
                        component = feedback["theta.vel"]

                    travelled += abs(component) * actual_dt

                    remaining_error = remaining - travelled
                    if remaining_error < max(speed * dt * 2.0, 0.02 if axis != "theta" else 1.0):
                        scale = remaining_error / max(remaining, 1e-6)
                        scaled_speed = max(scale, 0.2) * speed
                        if axis == "x":
                            self.set_body_velocity(direction * scaled_speed, 0.0, 0.0)
                        elif axis == "y":
                            self.set_body_velocity(0.0, direction * scaled_speed, 0.0)
                        else:
                            self.set_body_velocity(0.0, 0.0, direction * scaled_speed)
            finally:
                self.stop()

            end_positions = self.read_wheel_positions_counts()
            logger.info(f"[{axis}] end_positions={end_positions}")

        _run_segment(x, "x", linear_speed)
        _run_segment(y, "y", linear_speed)
        _run_segment(theta, "theta", angular_speed)

    def drive_forward(self, distance: float, speed: float = 0.2) -> None:
        self.drive_body_displacement(x=distance, linear_speed=abs(speed))

    def drive_linear_direction(
        self,
        distance: float,
        direction_deg: float,
        speed: float = 0.2,
        dt: float = 0.05,
    ) -> None:
        if not self.is_connected:
            raise RuntimeError("LeKiwi base is not connected")
        if abs(distance) < 1e-6:
            return

        direction_rad = math.radians(direction_deg)
        ux = math.cos(direction_rad)
        uy = math.sin(direction_rad)

        sign = 1.0 if distance > 0 else -1.0
        vx = ux * speed * sign
        vy = uy * speed * sign

        target = abs(distance)
        travelled = 0.0
        self.set_body_velocity(vx, vy, 0.0)

        last_time = time.perf_counter()
        try:
            while travelled < target:
                time.sleep(dt)
                now = time.perf_counter()
                actual_dt = now - last_time
                last_time = now

                feedback = self.read_body_velocity()
                component = feedback["x.vel"] * ux + feedback["y.vel"] * uy
                travelled += abs(component) * actual_dt

                remaining = target - travelled
                if remaining < max(speed * dt * 2.0, 0.02):
                    scale = max(remaining / target, 0.2)
                    self.set_body_velocity(vx * scale, vy * scale, 0.0)
        finally:
            self.stop()

    # ------------------------------------------------------------------
    # Internals – conversion utilities
    # ------------------------------------------------------------------
    def _body_to_wheel_rad_s(self, x: float, y: float, theta_deg_s: float) -> Dict[str, float]:
        """
        Convert body velocity to wheel angular velocities (rad/s).
        """
        theta_rad_s = math.radians(theta_deg_s)
        velocity_vector = np.array([x, y, theta_rad_s], dtype=float)

        # same wheel order as your original code: left, back, right
        angles = np.radians(np.array([240, 0, 120]) - 90.0)
        m = np.array([[np.cos(a), np.sin(a), self.config.base_radius] for a in angles], dtype=float)

        wheel_linear_speeds = m.dot(velocity_vector)                    # [m/s] along wheel rolling directions
        wheel_angular_speeds = wheel_linear_speeds / self.config.wheel_radius  # [rad/s]

        # limit / scale
        max_w = float(np.max(np.abs(wheel_angular_speeds)))
        if self.config.max_wheel_rad_s > 0 and max_w > self.config.max_wheel_rad_s:
            scale = self.config.max_wheel_rad_s / max_w
            wheel_angular_speeds = wheel_angular_speeds * scale

        return {
            "base_left_wheel": float(wheel_angular_speeds[0]),
            "base_back_wheel": float(wheel_angular_speeds[1]),
            "base_right_wheel": float(wheel_angular_speeds[2]),
        }

    def _wheel_rad_s_to_body(self, left_w: float, back_w: float, right_w: float) -> Dict[str, float]:
        """
        Convert wheel angular speeds (rad/s) to body velocity.
        Returns theta in deg/s (consistent with your original API).
        """
        wheel_radps = np.array([left_w, back_w, right_w], dtype=float)
        wheel_linear_speeds = wheel_radps * self.config.wheel_radius     # [m/s]

        angles = np.radians(np.array([240, 0, 120]) - 90.0)
        m = np.array([[np.cos(a), np.sin(a), self.config.base_radius] for a in angles], dtype=float)
        m_inv = np.linalg.inv(m)

        velocity_vector = m_inv.dot(wheel_linear_speeds)  # [vx, vy, wz(rad/s)]
        x, y, theta_rad_s = velocity_vector
        theta_deg_s = math.degrees(theta_rad_s)
        return {"x.vel": float(x), "y.vel": float(y), "theta.vel": float(theta_deg_s)}


__all__ = ["LeKiwiBaseConfig", "LeKiwiBaseController"]


if __name__ == "__main__":
    cfg = LeKiwiBaseConfig(
        can_channel="can0",
        bitrate=1_000_000,
        eds_path=os.path.normpath(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "hardware",
                "canopen",
                "eds",
                "CANOPEN-EDS-MBDV-Servo-SingleAxis-V1.1.1.eds",
            )
        ),
        encoder_cpr=2 ** 16,
        gear_ratio=10.0,
        wheel_radius=0.1015,
        base_radius=0.203,
        max_wheel_rad_s=2.0 * math.pi,  # 1 rev/s
        dir_left=+1,
        dir_back=+1,
        dir_right=+1,
    )

    base = LeKiwiBaseController(cfg)
    base.connect()

    try:
        print("Rotating 360 degrees ... (open-loop by velocity integration)")

        # 读开始位置（编码器 counts）
        start_positions = base.read_wheel_positions_counts()
        logger.info(f"start_positions={start_positions}")

        # 原地转 360 度：theta 单位是“度”
        base.drive_body_displacement(theta=360.0, angular_speed=30.0)

        # 读结束位置
        end_positions = base.read_wheel_positions_counts()
        logger.info(f"end_positions={end_positions}")

        print("DONE.")

     
         
    finally:
        base.stop()
        base.disconnect()
