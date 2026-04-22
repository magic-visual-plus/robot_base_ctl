"""
moons 包的路径配置：统一 EDS 文件路径、保证 joystick 模块可导入。

所有 moons/ 下的脚本应在 import Ds402_ctl / oni_ctrl 等之前执行：
    from _paths import JOYSTICK_DIR, EDS_SINGLE_AXIS, EDS_DUAL_AXES
这样 sys.path 里已包含 joystick/，后续 import 直接走 joystick 目录的模块。
"""
import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_BASE_CTL_DIR = os.path.normpath(os.path.join(_THIS_DIR, "..", ".."))

JOYSTICK_DIR = os.path.join(_BASE_CTL_DIR, "joystick")
HARDWARE_EDS_DIR = os.path.join(_BASE_CTL_DIR, "hardware", "canopen", "eds")

EDS_SINGLE_AXIS = os.path.join(HARDWARE_EDS_DIR, "CANOPEN-EDS-MBDV-Servo-SingleAxis-V1.1.1.eds")
EDS_DUAL_AXES = os.path.join(HARDWARE_EDS_DIR, "CANOPEN-EDS-MBDV-Servo-DulAxes-V1.0.eds")

if JOYSTICK_DIR not in sys.path:
    sys.path.insert(0, JOYSTICK_DIR)
