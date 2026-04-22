#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手柄 -> /cmd_vel（geometry_msgs/Twist），由唯一底盘节点 motor/moons/bridge.py 订阅并写 CANopen。

与 joystick_teleop.py 的摇杆映射一致（BTN_TR + ABS_X/Y/Z），但不打开 can0。
运行前请先启动 bridge，并 source ROS2：

  source /opt/ros/humble/setup.bash
  python3 /opt/project/robot_base_ctl/motor/moons/bridge.py   # 另终端

  cd /opt/project/robot_base_ctl/joystick
  python3 joystick_teleop_ros.py

可选参数（ROS2）：
  python3 joystick_teleop_ros.py --ros-args -p cmd_topic:=/cmd_vel -p publish_hz:=20.0
  # 摇杆接收打印：joy_log_hz 轴事件最高日志频率（Hz），0=不打印轴；BTN_TR 按下/松开始终打印
  python3 joystick_teleop_ros.py --ros-args -p joy_log_hz:=4.0
"""
from __future__ import annotations

import math
import threading
import time

import rclpy
from evdev import ecodes
from geometry_msgs.msg import Twist
from rclpy.node import Node

from common.input_utils import find_joystick, normalize_axis


class JoystickCmdVelTeleopNode(Node):
    """读 evdev，周期发布 Twist（linear.x/y m/s, angular.z rad/s）。"""

    def __init__(self) -> None:
        super().__init__("joystick_cmd_vel_teleop")

        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("linear_scale", 0.1)
        self.declare_parameter("angular_scale_deg_s", -15.0)
        self.declare_parameter("deadzone", 0.1)
        self.declare_parameter("joy_log_hz", 4.0)

        cmd_topic = str(self.get_parameter("cmd_topic").value)
        hz = float(self.get_parameter("publish_hz").value)
        self._linear_scale = float(self.get_parameter("linear_scale").value)
        self._angular_scale_deg_s = float(self.get_parameter("angular_scale_deg_s").value)
        self._deadzone = float(self.get_parameter("deadzone").value)

        if hz <= 0:
            hz = 20.0

        log_hz = float(self.get_parameter("joy_log_hz").value)
        self._joy_log_min_dt = (1.0 / max(log_hz, 0.25)) if log_hz > 0 else None
        self._last_joy_log_t = 0.0

        self._lock = threading.Lock()
        self.lx = 0.0
        self.ly = 0.0
        self.rx = 0.0
        self.btn_tr = False
        self._running = True

        self._pub = self.create_publisher(Twist, cmd_topic, 10)
        period = 1.0 / hz
        self._timer = self.create_timer(period, self._on_timer)

        self._dev = find_joystick()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        self.get_logger().info(
            f"Publishing Twist to {cmd_topic} @ {hz:.1f} Hz (bridge subscribes here; no CAN in this process)"
        )
        if self._joy_log_min_dt is not None:
            self.get_logger().info(
                f"Joystick recv log: up to {1.0 / self._joy_log_min_dt:.1f} Hz on stick axes "
                f"(set joy_log_hz:=0 to disable axis logs; BTN_TR always logged)"
            )

    def _maybe_log_axes(
        self,
        axis_name: str,
        raw: int,
        norm: float,
        lx: float,
        ly: float,
        rx: float,
        btn: bool,
    ) -> None:
        if self._joy_log_min_dt is None:
            return
        t = time.monotonic()
        if t - self._last_joy_log_t < self._joy_log_min_dt:
            return
        self._last_joy_log_t = t
        self.get_logger().info(
            f"[joy] {axis_name} raw={raw} norm={norm:+.4f} | "
            f"lx={lx:+.4f} ly={ly:+.4f} rx={rx:+.4f} BTN_TR={btn}"
        )

    def _read_loop(self) -> None:
        for event in self._dev.read_loop():
            if not self._running:
                break
            if event.type == ecodes.EV_ABS:
                code, val = event.code, event.value
                if code in (ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_Z):
                    norm = normalize_axis(val)
                    axis_name = ecodes.bytype[ecodes.EV_ABS].get(code, str(code))
                    with self._lock:
                        if code == ecodes.ABS_X:
                            self.ly = -norm
                        elif code == ecodes.ABS_Y:
                            self.lx = norm
                        elif code == ecodes.ABS_Z:
                            self.rx = norm
                        lx, ly, rx, btn = self.lx, self.ly, self.rx, self.btn_tr
                    self._maybe_log_axes(axis_name, val, norm, lx, ly, rx, btn)
            elif event.type == ecodes.EV_KEY and event.code == ecodes.BTN_TR:
                if event.value not in (0, 1):
                    continue
                with self._lock:
                    if event.value == 1:
                        self.btn_tr = True
                        state = "PRESS"
                    else:
                        self.btn_tr = False
                        state = "RELEASE"
                    lx, ly, rx = self.lx, self.ly, self.rx
                self.get_logger().info(
                    f"[joy] BTN_TR {state} | lx={lx:+.3f} ly={ly:+.3f} rx={rx:+.3f}"
                )

    def _on_timer(self) -> None:
        with self._lock:
            btn = self.btn_tr
            lx, ly, rx = self.lx, self.ly, self.rx

        tw = Twist()
        if btn:
            lx_u = 0.0 if abs(lx) < self._deadzone else lx
            ly_u = 0.0 if abs(ly) < self._deadzone else ly
            rx_u = 0.0 if abs(rx) < self._deadzone else rx
            vx = -lx_u * self._linear_scale
            vy = ly_u * self._linear_scale
            theta_deg_s = rx_u * self._angular_scale_deg_s
            tw.linear.x = float(vx)
            tw.linear.y = float(vy)
            tw.angular.z = float(math.radians(theta_deg_s))
        self._pub.publish(tw)

    def destroy_node(self) -> None:
        self._running = False
        try:
            self._pub.publish(Twist())
        except Exception:
            pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = JoystickCmdVelTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
