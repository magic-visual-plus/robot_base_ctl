builtin_interfaces/Time stamp

# command_type 映射（参考 pymbc/taskflow/teleop/protocol.py 的 TeleopCommandType）
# 0: start
# 1: back_zero
# 2: together
# 3: body_ctl
# 4: reset
# 5: stop
uint8 command_type

# base / waist
float64 base_x
float64 base_y
float64 base_z
float64 base_pitch
float64 waist_height
float64 waist_roll

# head
float64 head_rx
float64 head_rz

# right arm pose + gripper
float64 r_x
float64 r_y
float64 r_z
float64 r_qw
float64 r_qx
float64 r_qy
float64 r_qz
float64 r_finger

# left arm pose + gripper
float64 l_x
float64 l_y
float64 l_z
float64 l_qw
float64 l_qx
float64 l_qy
float64 l_qz
float64 l_finger

