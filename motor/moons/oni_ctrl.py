"""
薄代理：所有 LeKiwi 底盘控制实现统一维护在 joystick/oni_ctrl.py。
moons/ 下的脚本 `from oni_ctrl import ...` 时会走到这里，再转发到 joystick 版本。
"""
import importlib
import sys
import os

_JOYSTICK_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "joystick")
)

_self_mod = sys.modules.pop(__name__, None)
_old_path = sys.path[:]
try:
    if _JOYSTICK_DIR not in sys.path:
        sys.path.insert(0, _JOYSTICK_DIR)
    _real = importlib.import_module("oni_ctrl")
finally:
    sys.path[:] = _old_path

_g = globals()
for _attr in dir(_real):
    if not _attr.startswith("__"):
        _g[_attr] = getattr(_real, _attr)

sys.modules[__name__] = _real
