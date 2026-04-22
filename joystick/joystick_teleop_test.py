#!/usr/bin/env python3
from evdev import InputDevice, ecodes
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

        if ecodes.EV_ABS in caps and "mouse" not in dev.name.lower() and "keyboard" not in dev.name.lower():
            print(f"Found joystick-like device: {dev.name} at {dev.path}")
            return dev

    raise RuntimeError("Joystick not found! Check /dev/input/event* permission.")


# ----------------------------------------------
# 手柄测试类，支持组合键判断
# ----------------------------------------------
class JoystickTester:
    def __init__(self):
        self.dev = find_joystick()
        self.btn_tr_pressed = False  # 记录 BTN_TR 状态

    def listen(self):
        print("Listening to joystick events...")
        print("Press buttons or move sticks to see event output.\n")

        for event in self.dev.read_loop():

            # 摇杆 / 扳机 轴事件
            if event.type == ecodes.EV_ABS:
                code = event.code
                val = event.value
                code_name = ecodes.bytype[ecodes.EV_ABS].get(code, f"UNKNOWN_ABS_{code}")

                # 归一化（可选）
                if 0 <= val <= 255:
                    norm = (val - 128) / 128.0
                elif -32768 <= val <= 32767:
                    norm = val / 32767.0 if val != -32768 else -1.0
                else:
                    norm = val

                # 打印
                print(f"[AXIS] {code_name:<10} raw={val:<6} norm={norm:+.3f}")

                # 判断组合键
                if self.btn_tr_pressed and code_name == "ABS_HAT0Y":
                    if val == -1:
                        print(">>> 升降柱子上升")
                    elif val == 1:
                        print(">>> 升降柱子下降")
                    # val == 0 不处理

            # 按键事件
            elif event.type == ecodes.EV_KEY:
                code = event.code
                val = event.value
                code_name = ecodes.bytype[ecodes.EV_KEY].get(code, f"UNKNOWN_KEY_{code}")

                state = {0:"RELEASED", 1:"PRESSED", 2:"HOLD"}.get(val, f"STATE_{val}")
                print(f"[BUTTON] {code_name:<15} code={code:<4} state={state}")

                # 记录 BTN_TR 按下状态
                if code_name == "BTN_TR":
                    self.btn_tr_pressed = (val == 1)  # 按下为 True，松开为 False


if __name__ == "__main__":
    jt = JoystickTester()
    try:
        jt.listen()
    except KeyboardInterrupt:
        print("Stopping...")
