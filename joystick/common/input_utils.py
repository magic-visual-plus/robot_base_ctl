from evdev import InputDevice, ecodes, list_devices


def find_joystick():
    devices = [InputDevice(path) for path in list_devices()]
    for dev in devices:
        caps = dev.capabilities()
        if ecodes.EV_ABS in caps and "mouse" not in dev.name.lower() and "keyboard" not in dev.name.lower():
            print(f"Found joystick-like device: {dev.name} at {dev.path}")
            return dev
    raise RuntimeError("Joystick not found! Check /dev/input/event* permission.")


def normalize_axis(val: int) -> float:
    # Some gamepads report 0..255, others report signed int16.
    if val <= 255:
        return (val - 128) / 128.0
    return val / 32767.0

