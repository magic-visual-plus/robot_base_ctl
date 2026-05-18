"""查找 evdev 手柄设备（与 joystick_teleop_test 行为一致）。"""

from evdev import InputDevice, ecodes, list_devices


def find_joystick():
    devices = [InputDevice(path) for path in list_devices()]
    for dev in devices:
        caps = dev.capabilities()
        if ecodes.EV_ABS in caps and "mouse" not in dev.name.lower() and "keyboard" not in dev.name.lower():
            return dev
    raise RuntimeError("Joystick not found! Check /dev/input/event* permission.")
