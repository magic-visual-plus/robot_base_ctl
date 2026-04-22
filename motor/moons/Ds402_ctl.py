"""
薄代理：所有 DS402 实现统一维护在 joystick/Ds402_ctl.py。
moons/ 下的脚本 `from Ds402_ctl import ...` 时会走到这里，再转发到 joystick 版本。
"""
import importlib
import sys
import os

_JOYSTICK_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "joystick")
)

# 临时把自己从 sys.modules 里拿掉，让 importlib 去 joystick/ 找真正的模块
_self_mod = sys.modules.pop(__name__, None)
_old_path = sys.path[:]
try:
    if _JOYSTICK_DIR not in sys.path:
        sys.path.insert(0, _JOYSTICK_DIR)
    _real = importlib.import_module("Ds402_ctl")
finally:
    sys.path[:] = _old_path

# 把真正模块的全部属性写入本模块（re-export）
_g = globals()
for _attr in dir(_real):
    if not _attr.startswith("__"):
        _g[_attr] = getattr(_real, _attr)

# 恢复 sys.modules 映射到真正的模块（后续 import 不会再走这个文件）
sys.modules[__name__] = _real
