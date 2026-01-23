from __future__ import annotations

from loguru import logger
from dataclasses import dataclass
from typing import Dict, Optional

import math
import time
import numpy as np

# ✅ 导入“我们自己的电机控制代码”（把你之前那份 DS402 多轴脚本保存成这个模块名）
# 里面需要有：CanopenBus, MotorManager
from Ds402_ctl import CanopenBus, MotorManager


@dataclass
class LeKiwiBaseConfig:
    """Configuration parameters required to operate the base (DS402 CANopen wheels)."""

    # CANopen
    can_channel: str = "can0"
    bitrate: int = 1_000_000
    eds_path: str = "/home/nvidia/github/robot_base_ctl/motor/moons/CANOPEN-EDS-MBDV-Servo-SingleAxis-V1.1.1.eds"

    # motor encoder/gearing (for rad/s<->counts conversion inside DS402 module)
    encoder_cpr: int = 2 ** 16
    gear_ratio: float = 10.0  # you said: output one rev = 2^16 * 10 counts

    # base geometry
    wheel_radius: float = 0.1015   # [m]
    base_radius: float = 0.203   # [m] distance from center to wheel contact

    # safety limits (output-shaft wheel angular speed limit, rad/s)
    max_wheel_rad_s: float = 2.0 * math.pi  # default 1 rev/s

    # wheel direction multipliers (if some wheel is reversed, set -1)
    dir_left: int = +1
    dir_back: int = +1
    dir_right: int = +1


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

        self._bus = CanopenBus(channel=config.can_channel, bitrate=config.bitrate, interface="socketcan")
        self._mgr = MotorManager(self._bus)
        self._connected = False

    # ------------------------------------------------------------------
    # Connection & configuration
    # ------------------------------------------------------------------
    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        """Connect CANopen network and init/enable DS402 motors in PV mode."""
        if self.is_connected:
            logger.info("LeKiwi base already connected")
            return

        self._bus.connect(can_filters=None)

        # add 3 motors with our mapping
        # NOTE: MotorManager.add_motor signature in our DS402 code is:
        #   add_motor(node_id, eds_path, encoder_cpr=..., gear_ratio=...)
        self._mgr.add_motor(self.NODE_LEFT, self.config.eds_path,
                            encoder_cpr=self.config.encoder_cpr, gear_ratio=self.config.gear_ratio)
        self._mgr.add_motor(self.NODE_BACK, self.config.eds_path,
                            encoder_cpr=self.config.encoder_cpr, gear_ratio=self.config.gear_ratio)
        self._mgr.add_motor(self.NODE_RIGHT, self.config.eds_path,
                            encoder_cpr=self.config.encoder_cpr, gear_ratio=self.config.gear_ratio)

        # init and enable
        self._mgr.init_all(default_mode="PV", zero_on_init=True)
        self._mgr.enable_all(timeout_s=8.0)
        self._mgr.set_mode_all("PV")

        self._connected = True
        logger.info(f"LeKiwi base connected on {self.config.can_channel}, bitrate={self.config.bitrate}")

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

        # send to DS402 nodes (external interface of our DS402 module is rad/s)
        self._mgr.set_velocities_rad_s({
            self.NODE_LEFT: float(w["base_left_wheel"]),
            self.NODE_BACK: float(w["base_back_wheel"]),
            self.NODE_RIGHT: float(w["base_right_wheel"]),
        })

    def stop(self) -> None:
        """Stop all wheels (rad/s -> 0)."""
        if not self.is_connected:
            return
        self._mgr.set_velocities_rad_s({self.NODE_LEFT: 0.0, self.NODE_BACK: 0.0, self.NODE_RIGHT: 0.0})

    def read_body_velocity(self) -> Dict[str, float]:
        """Read wheel velocities and convert back to body-frame velocity."""
        if not self.is_connected:
            raise RuntimeError("LeKiwi base is not connected")

        fb_l = self._mgr.motors[self.NODE_LEFT].feedback()
        fb_b = self._mgr.motors[self.NODE_BACK].feedback()
        fb_r = self._mgr.motors[self.NODE_RIGHT].feedback()

        # DS402 feedback already provides velocity_rad_s (output shaft)
        wl = fb_l.velocity_rad_s or 0.0
        wb = fb_b.velocity_rad_s or 0.0
        wr = fb_r.velocity_rad_s or 0.0

        # undo direction multipliers to get physical wheel direction back
        wl *= self.config.dir_left
        wb *= self.config.dir_back
        wr *= self.config.dir_right

        return self._wheel_rad_s_to_body(wl, wb, wr)

    def read_wheel_positions_counts(self) -> Dict[str, int]:
        """Read wheel positions (raw encoder counts, whatever drive reports)."""
        if not self.is_connected:
            raise RuntimeError("LeKiwi base is not connected")
        out: Dict[str, int] = {}
        for name, nid in self.name_to_node.items():
            fb = self._mgr.motors[nid].feedback()
            out[name] = int(fb.position_counts or 0)
        return out

    def read_wheel_positions_rad(self) -> Dict[str, float]:
        """Read wheel positions converted to output shaft radians (best effort)."""
        if not self.is_connected:
            raise RuntimeError("LeKiwi base is not connected")
        out: Dict[str, float] = {}
        for name, nid in self.name_to_node.items():
            fb = self._mgr.motors[nid].feedback()
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
        eds_path="/home/nvidia/github/robot_base_ctl/motor/moons/CANOPEN-EDS-MBDV-Servo-SingleAxis-V1.1.1.eds",
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
