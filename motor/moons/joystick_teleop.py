#!/usr/bin/env python3
from evdev import InputDevice, categorize, ecodes
import threading
import time
import math

from oni_ctrl import LeKiwiBaseConfig, LeKiwiBaseController
import os

# get cur_dir
cur_dir = os.path.dirname(os.path.abspath(__file__))

# ----------------------------------------------
# 找手柄设备
# ----------------------------------------------
def find_joystick():
    from evdev import list_devices, InputDevice, ecodes

    devices = [InputDevice(path) for path in list_devices()]
    for dev in devices:
        caps = dev.capabilities()

        # 只要设备提供 ABS（摇杆轴），就当成手柄
        if ecodes.EV_ABS in caps and "mouse" not in dev.name.lower() and "keyboard" not in dev.name.lower():
            print(f"Found joystick-like device: {dev.name} at {dev.path}")
            return dev

    raise RuntimeError("Joystick not found! Check /dev/input/event* permission.")


# ----------------------------------------------
# 主控制类
# ----------------------------------------------
class JoystickController:
    def __init__(self):
        # 初始化底盘
        cfg = LeKiwiBaseConfig(
            can_channel="can0",
            bitrate=1_000_000,
            # eds_path=f"{cur_dir}/CANOPEN-EDS-MBDV-Servo-SingleAxis-V1.1.1.eds",
            eds_path=f"{cur_dir}/CANOPEN-EDS-MBDV-Servo-DulAxes-V1.0.eds",
            encoder_cpr=2 ** 16,
            gear_ratio=10.0,
            wheel_radius=0.1015,
            base_radius=0.203,
            max_wheel_rad_s=4.0 * math.pi,  # 1 rev/s
            dir_left=+1,
            dir_back=+1,
            dir_right=+1,
        )

        self.base = LeKiwiBaseController(cfg)
        self.base.connect()
 

        # 找手柄
        self.dev = find_joystick()

        # 当前摇杆值
        self.lx = 0.0   # 左摇杆左右
        self.ly = 0.0   # 左摇杆上下
        self.rx = 0.0   # 右摇杆左右（旋转）

        # 最大速度
        self.linear_scale = 0.5     # m/s
        self.angular_scale = -45.0   # deg/s

        # 开线程持续驱动
        threading.Thread(target=self.control_loop, daemon=True).start()

    # -------------------------------------------------
    # 控制循环：每 20ms 给底盘发一次速度
    # -------------------------------------------------
    def control_loop(self):
        deadzone = 0.1    # 小于这个就当摇杆没动

        while True:
            # 处理死区
            lx = 0 if abs(self.lx) < deadzone else self.lx
            ly = 0 if abs(self.ly) < deadzone else self.ly
            rx = 0 if abs(self.rx) < deadzone else self.rx

            # 映射速度
            x = -lx * self.linear_scale
            y = ly * self.linear_scale
            theta = rx * self.angular_scale

            try:
                self.base.set_body_velocity(x, y, theta)
            except:
                pass
            time.sleep(0.02)  # 50Hz


    # -------------------------------------------------
    # 监听手柄事件
    # -------------------------------------------------
    def listen(self):
        print("Listening to joystick events...")

        for event in self.dev.read_loop():

            # 摇杆轴事件
            if event.type == ecodes.EV_ABS:
                code = event.code
                val  = event.value

                # 自动归一化不同手柄的范围
                if code in [ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_Z]:
                    # YICHIP / 蓝牙手柄范围 = 0~255
                    if val <= 255:
                        norm = (val - 128) / 128.0    # 中值 128
                    # 通用手柄范围 = -32768~32767
                    else:
                        norm = val / 32767.0

                    if code == ecodes.ABS_X:     # 左摇杆左右
                        self.lx = norm
                    elif code == ecodes.ABS_Y:   # 左摇杆上下
                        self.ly = norm
                    elif code == ecodes.ABS_Z:  # 右摇杆左右
                        self.rx = norm

                    #print(f"Axis {code}: raw={val}, norm={norm:.3f}")

            # 按键事件
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

