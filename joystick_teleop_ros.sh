#!/usr/bin/env bash
# 手柄 -> /cmd_vel；需另开终端已运行 motor/moons/bridge.py
exec python3 /opt/project/robot_base_ctl/joystick/joystick_teleop_ros.py "$@"
