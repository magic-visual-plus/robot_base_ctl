# 当前高度（mm）
float64 current_height_mm

# 目标高度（mm）
float64 target_height_mm

# 是否有正在执行的目标
bool has_active_goal

# 是否在到位窗口内（已到目标）
bool in_position_window

# CiA402 Statusword（对象 0x6041），典型由 TPDO1（COB 180h + node_id）周期上报
uint16 status_word
# 若为 false，表示尚未从 PDO 收到且 SDO 读失败（此时 status_word 填 0）
bool status_word_valid
