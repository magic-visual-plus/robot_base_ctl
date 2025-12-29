# -*- coding: utf-8 -*-
"""Standalone controller for the LeKiwi omniwheel base.

This module extracts the chassis-related logic from ``lekiwi.py`` so the base can be
used independently from the full robot (arm + cameras).  It exposes a small API to
connect to the Feetech bus, send planar velocity commands, and read back the current
estimated base velocity.
"""

from __future__ import annotations

from loguru import logger
from dataclasses import dataclass
from typing import Dict

import math
import time

import numpy as np

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

 

@dataclass
class LeKiwiBaseConfig:
    """Configuration parameters required to operate the base."""

    port: str = "/dev/ttyUSB0"
    wheel_radius: float = 0.05  # [m] #轮半径
    base_radius: float = 0.125  # [m] #地心到轮子中间的距离
    max_raw_command: int = 3000  # maximum allowed raw speed (ticks) #最大允许的原始速度（刻度）


class LeKiwiBaseController:
    """Controller for the three-wheel omni base of LeKiwi.

    The controller exposes velocity-level commands expressed in the body frame
    (``x.vel``, ``y.vel`` in m/s and ``theta.vel`` in deg/s). Internally those are
    converted into wheel velocity set-points and sent to the Feetech STS3215 motors
    operating in velocity mode.
    """

    def __init__(self, config: LeKiwiBaseConfig):
        self.config = config
        self._bus = FeetechMotorsBus(
            port=config.port,
            motors={ # 注册了三个b电机 *左轮7 *后轮8 *右轮9
                "base_left_wheel": Motor(7, "sts3215", MotorNormMode.RANGE_M100_100),
                "base_back_wheel": Motor(8, "sts3215", MotorNormMode.RANGE_M100_100),
                "base_right_wheel": Motor(9, "sts3215", MotorNormMode.RANGE_M100_100),
            },
            calibration=None,
        )
        self.base_motors = list(self._bus.motors.keys())

    # ------------------------------------------------------------------
    # Connection & configuration
    # ------------------------------------------------------------------
    @property
    def is_connected(self) -> bool: #插线了吗？
        return self._bus.is_connected

    def connect(self) -> None: 
        """Connect to the motor bus and configure wheel motors."""

        if self.is_connected:
            logger.info("LeKiwi base already connected")
            return

        self._bus.connect()
        self._configure_wheels()
        logger.info(f"LeKiwi base connected on self.config.port")

    def disconnect(self) -> None:
        if not self.is_connected:
            return
        self.stop()
        self._bus.disconnect()
        logger.info("LeKiwi base disconnected")

    def _configure_wheels(self) -> None:
        """Put wheel motors in velocity mode and enable torque."""

        self._bus.disable_torque()
        self._bus.configure_motors()
        for name in self.base_motors:
            self._bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
        self._bus.enable_torque()
        #→ 打开串口 /dev/ttyACM0
#→ 禁用电机力矩
#→ 设置电机模式为速度控制
#→ 重新上电使能
#→ 打印 "LeKiwi base connected on /dev/ttyACM0"


    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def set_body_velocity(self, x: float, y: float, theta: float) -> None:
        """Command body velocity (x, y in m/s, theta in deg/s)."""

        if not self.is_connected:
            raise RuntimeError("LeKiwi base is not connected")

        wheel_raw = self._body_to_wheel_raw(x, y, theta)
        self._bus.sync_write("Goal_Velocity", wheel_raw)

    def stop(self) -> None:
        """Convenience helper to stop all wheels."""

        if not self.is_connected:
            return
        zero_cmd = {name: 0 for name in self.base_motors}
        self._bus.sync_write("Goal_Velocity", zero_cmd)

    def read_body_velocity(self) -> Dict[str, float]:
        """Read wheel velocities and convert them back to body-frame velocity."""

        if not self.is_connected:
            raise RuntimeError("LeKiwi base is not connected")

        wheel_raw = self._bus.sync_read("Present_Velocity", self.base_motors)# 读取三个轮子的速度
        return self._wheel_raw_to_body(
            wheel_raw["base_left_wheel"],
            wheel_raw["base_back_wheel"],
            wheel_raw["base_right_wheel"],
        )
    def read_wheel_positions(self) -> Dict[str, int]: #############读取刻度
        """Read wheel positions."""

        if not self.is_connected:
            raise RuntimeError("LeKiwi base is not connected")

        wheel_positions = self._bus.sync_read("Present_Position", self.base_motors, normalize=False)
        return wheel_positions

    def drive_body_displacement(
        self,
        x: float = 0.0, ############平移 旋转
        y: float = 0.0,
        theta: float = 0.0,
        linear_speed: float = 0.2,
        angular_speed: float = 30.0,
        dt: float = 0.05,
    ) -> None:
        """Drive a nominal displacement using simple velocity integration.

        Args:
            x: Target displacement along body X (m).
            y: Target displacement along body Y (m).
            theta: Target rotation (deg, positive CCW).
            linear_speed: Magnitude used for linear motion commands (m/s).
            angular_speed: Magnitude for angular motion commands (deg/s).
            dt: Control loop period (s).

        The function sequentially executes up to three segments (x, y, theta),
        each using constant-speed commands while integrating the measured
        velocity feedback to estimate completion. This is a simple open-loop
        integrator with feedback correction and is not a precise trajectory
        controller but suffices for small displacements on moderate grip
        surfaces.
        """

        if not self.is_connected:
            raise RuntimeError("LeKiwi base is not connected")

        def _run_segment(target: float, axis: str, speed: float) -> None:
            if abs(target) < 1e-6:
                return

            direction = 1.0 if target > 0 else -1.0
            remaining = abs(target)

            # Command velocities for the selected axis
            if axis == "x":
                vx, vy, vz = direction * speed, 0.0, 0.0
            elif axis == "y":
                vx, vy, vz = 0.0, direction * speed, 0.0
            elif axis == "theta":
                vx, vy, vz = 0.0, 0.0, direction * speed
            else:
                raise ValueError(f"Unknown axis '{axis}'")

            self.set_body_velocity(vx, vy, vz)
            
            logger.info(f"start_positions={start_positions}")
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

                    # Simple proportional reduction when close to target
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
            end_positions = self.read_wheel_positions()
            logger.info(f"end_positions={end_positions}")
        _run_segment(x, "x", linear_speed)
        _run_segment(y, "y", linear_speed)
        _run_segment(theta, "theta", angular_speed) 

    def drive_forward(self, distance: float, speed: float = 0.2) -> None:
        """Drive forward (positive X) by a given distance in meters."""

        self.drive_body_displacement(x=distance, linear_speed=abs(speed))

    def drive_linear_direction(
        self,
        distance: float,
        direction_deg: float,
        speed: float = 0.2,
        dt: float = 0.05,
    ) -> None:
        """Drive along an arbitrary planar direction for a given distance.

        Args:
            distance: Target displacement magnitude (meters). Positive moves along
                ``direction_deg``; negative moves opposite.
            direction_deg: Direction angle in body frame degrees (0 along +X, 90
                along +Y).
            speed: Commanded speed magnitude (m/s).
            dt: Control loop period (seconds).
        """

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
    @staticmethod
    def _degps_to_raw(degps: float) -> int:
        steps_per_deg = 4096.0 / 360.0
        speed_in_steps = degps * steps_per_deg
        speed_int = int(round(speed_in_steps))
        if speed_int > 0x7FFF:
            speed_int = 0x7FFF
        elif speed_int < -0x8000:
            speed_int = -0x8000
        return speed_int

    @staticmethod
    def _raw_to_degps(raw_speed: int) -> float:
        steps_per_deg = 4096.0 / 360.0
        return raw_speed / steps_per_deg

    def _body_to_wheel_raw(self, x: float, y: float, theta: float) -> Dict[str, int]:
        theta_rad = theta * (np.pi / 180.0)
        velocity_vector = np.array([x, y, theta_rad])

        angles = np.radians(np.array([240, 0, 120]) - 90)
        m = np.array([[np.cos(a), np.sin(a), self.config.base_radius] for a in angles])

        wheel_linear_speeds = m.dot(velocity_vector)
        wheel_angular_speeds = wheel_linear_speeds / self.config.wheel_radius
        wheel_degps = wheel_angular_speeds * (180.0 / np.pi)

        steps_per_deg = 4096.0 / 360.0
        raw_floats = [abs(degps) * steps_per_deg for degps in wheel_degps]
        max_raw_computed = max(raw_floats)
        if max_raw_computed > self.config.max_raw_command:
            scale = self.config.max_raw_command / max_raw_computed
            wheel_degps = wheel_degps * scale

        wheel_raw = [self._degps_to_raw(deg) for deg in wheel_degps]

        return {
            "base_left_wheel": wheel_raw[0],
            "base_back_wheel": wheel_raw[1],
            "base_right_wheel": wheel_raw[2],
        }

    def _wheel_raw_to_body(
        self,
        left_wheel_speed: int,
        back_wheel_speed: int,
        right_wheel_speed: int,
    ) -> Dict[str, float]:
        wheel_degps = np.array(
            [
                self._raw_to_degps(left_wheel_speed),
                self._raw_to_degps(back_wheel_speed),
                self._raw_to_degps(right_wheel_speed),
            ]
        )

        wheel_radps = wheel_degps * (np.pi / 180.0)
        wheel_linear_speeds = wheel_radps * self.config.wheel_radius

        angles = np.radians(np.array([240, 0, 120]) - 90)
        m = np.array([[np.cos(a), np.sin(a), self.config.base_radius] for a in angles])
        m_inv = np.linalg.inv(m)
        velocity_vector = m_inv.dot(wheel_linear_speeds)
        x, y, theta_rad = velocity_vector
        theta = theta_rad * (180.0 / np.pi)
        return {"x.vel": x, "y.vel": y, "theta.vel": theta}


__all__ = ["LeKiwiBaseConfig", "LeKiwiBaseController"]

if __name__ == "__main__":

    cfg = LeKiwiBaseConfig(port="/dev/ttyACM0")
    base = LeKiwiBaseController(cfg)

    base.connect()

    try:
        print("Driving forward 0.5 m ...")
        #base.drive_forward(-0.6, speed=0.2)
        #print(base.read_body_velocity())
        #print("Driving 0.4 m at 30 degrees ...")
        #base.drive_linear_direction(distance=0.692, direction_deg=-30.0, speed=0.5)
        #print(base.read_body_velocity())

        #print("Rotating 45 degrees ...")
        start_positions = base.read_wheel_positions()
        logger.info(f"start_positions={start_positions}")
        base.drive_body_displacement(theta=90, angular_speed=30.0)
        print(base.read_body_velocity())
        end_positions = base.read_wheel_positions()
        logger.info(f"end_positions={end_positions}")
    finally:
        base.stop()
        base.disconnect() 
