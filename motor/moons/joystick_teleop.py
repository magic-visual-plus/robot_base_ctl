#!/usr/bin/env python3
"""
手柄遥控底盘（精简版，仅底盘，无升降柱）。

控制逻辑来自 joystick/ 的公共库：
  - oni_ctrl.LeKiwiBaseConfig / LeKiwiBaseController
  - common/input_utils.find_joystick / normalize_axis
"""
import math
import threading
import time

from evdev import ecodes

from _paths import EDS_DUAL_AXES, JOYSTICK_DIR  # noqa: F401  (sys.path side-effect)
from oni_ctrl import LeKiwiBaseConfig, LeKiwiBaseController

import sys, os
if JOYSTICK_DIR not in sys.path:
    sys.path.insert(0, JOYSTICK_DIR)
from common.input_utils import find_joystick, normalize_axis


class JoystickController:
    def __init__(self):
        cfg = LeKiwiBaseConfig(
            can_channel="can0",
            bitrate=1_000_000,
            eds_path=EDS_DUAL_AXES,
            encoder_cpr=2**16,
            gear_ratio=10.0,
            wheel_radius=0.1015,
            base_radius=0.203,
            max_wheel_rad_s=4.0 * math.pi,
            dir_left=+1,
            dir_back=+1,
            dir_right=+1,
        )
        self.base = LeKiwiBaseController(cfg)
        self.base.connect()

        self.dev = find_joystick()

        self.lx = 0.0
        self.ly = 0.0
        self.rx = 0.0
        self.linear_scale = 0.1
        self.angular_scale = -15.0
        self.deadzone = 0.1

        threading.Thread(target=self.control_loop, daemon=True).start()

    def control_loop(self):
        while True:
            lx = 0 if abs(self.lx) < self.deadzone else self.lx
            ly = 0 if abs(self.ly) < self.deadzone else self.ly
            rx = 0 if abs(self.rx) < self.deadzone else self.rx

            x = -lx * self.linear_scale
            y = ly * self.linear_scale
            theta = rx * self.angular_scale

            try:
                self.base.set_body_velocity(x, y, theta)
            except Exception:
                pass
            time.sleep(0.02)

    def listen(self):
        print("Listening to joystick events...")
        for event in self.dev.read_loop():
            if event.type == ecodes.EV_ABS:
                code, val = event.code, event.value
                if code in (ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_Z):
                    norm = normalize_axis(val)
                    if code == ecodes.ABS_X:
                        self.lx = norm
                    elif code == ecodes.ABS_Y:
                        self.ly = norm
                    elif code == ecodes.ABS_Z:
                        self.rx = norm
            elif event.type == ecodes.EV_KEY:
                print("Button:", event.code, event.value)


if __name__ == "__main__":
    jc = JoystickController()
    try:
        jc.listen()
    except KeyboardInterrupt:
        print("Stopping...")
        jc.base.stop()
        jc.base.disconnect()
