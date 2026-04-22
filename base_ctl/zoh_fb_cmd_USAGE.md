# zoh_fb_cmd.py 使用说明

`zoh_fb_cmd.py` 负责：

- **订阅** `/phi/motion/teleop/whole_body`（`pymbc_msgs/WholeBodyData`），在 `command_type` 允许时把 `base_x / base_y / base_pitch` 转成 **`/ref_pose`**（`geometry_msgs/PoseStamped`）。
- **提供 Action** `nav2_msgs/action/NavigateToPose`（原生接口，goal 为 `geometry_msgs/PoseStamped`），持续发 `/ref_pose` 直到里程计反馈到达容差。

底层闭环由 **`zoh_rev.py`** 完成：订阅 `/ref_pose` 与 `/rko_lio/odometry`，发布 `/cmd_vel`。  
电机桥接按现场使用 **`motor/moons/bridge.py`** 或自有 launch。

---

## 环境准备

每次新开终端先执行：

```bash
source /opt/ros/humble/setup.bash
source /opt/project/robot_base_ctl/ros2_msg/install/setup.bash
```

（`pymbc_msgs` 含 `WholeBodyData`，必须 source 该工作区。）

---

## 推荐启动顺序（多终端）

| 顺序 | 组件 | 命令 |
|------|------|------|
| 1 | 里程计 | 保证 `/rko_lio/odometry` 有数据（如 LIO / 定位节点） |
| 2 | 参考 + Action | 见下「方式 A」或「方式 B」 |
| 3 | 跟踪控制 | `python3 /opt/project/robot_base_ctl/base_ctl/zoh_rev.py` |
| 4 | 底盘桥 | `python3 /opt/project/robot_base_ctl/motor/moons/bridge.py`（或项目内惯用方式） |

### 方式 A：直接运行脚本

```bash
python3 /opt/project/robot_base_ctl/base_ctl/zoh_fb_cmd.py
```

### 方式 B：使用 tmux 脚本（仅底盘节点）

```bash
cd /opt/project/robot_base_ctl/tosor_ctl
./start_base_only.sh start
./start_base_only.sh attach
```

---

## 测试 NavigateToPose（Action）

确保 `zoh_fb_cmd.py` 已运行，且 **`zoh_rev.py` 已运行**，否则只有 `/ref_pose`、车体不会按闭环运动。

```bash
source /opt/ros/humble/setup.bash
source /opt/project/robot_base_ctl/ros2_msg/install/setup.bash

ros2 action send_goal /phi/motion/control/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -0.3, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, behavior_tree: ''}" \
  --feedback
```

若 Action 名带命名空间，先查看：

```bash
ros2 action list | grep -i navigate
```

将命令中的 action 名与 `ros2 action list` 一致；若要用其它名称，启动节点时指定：

```bash
python3 /opt/project/robot_base_ctl/base_ctl/zoh_fb_cmd.py --ros-args -p action_server_name:=navigate_to_pose
```

---

## 遥操作话题（WholeBodyData）

有上游发布 `/phi/motion/teleop/whole_body` 时，`command_type` 为 `0/1/2/3` 会进入跟随；`4` 为 reset 里程计；`5` 为 stop。  
长时间无新消息可通过参数 `teleop_timeout_sec` 自动退出跟随。

---

## 主要 ROS 参数

| 参数 | 含义 | 默认 |
|------|------|------|
| `teleop_timeout_sec` | 遥操作无新消息超时（秒），超时后不再发 `/ref_pose` | `2.0` |
| `action_server_name` | NavigateToPose 服务名 | `/phi/motion/control/navigate_to_pose` |
| `action_timeout_sec` | Action 执行超时（秒） | `120.0` |
| `action_stable_time_sec` | 位置/航向进入容差后需保持的时间（秒） | `0.3` |
| `action_pos_tolerance_m` | 位置容差（米） | `0.05` |
| `action_yaw_tolerance_rad` | 航向容差（弧度） | 约 2° |

示例：

```bash
python3 /opt/project/robot_base_ctl/base_ctl/zoh_fb_cmd.py --ros-args \
  -p teleop_timeout_sec:=3.0 \
  -p action_pos_tolerance_m:=0.08
```

---

## 坐标与单位说明

- `base_x`、`base_y`：与 `/ref_pose` 及 **`/rko_lio/odometry` 的 `pose.pose.position`（x,y）**、`zoh_rev` 里 `get_pose()` 一致（米）。
- `base_pitch`：平面航向，**弧度**。
- NavigateToPose 的 goal 使用 **四元数**；节点内部会解算为 yaw 再发 `/ref_pose`。

---

## 快速自检

```bash
ros2 topic echo /ref_pose --once
ros2 topic echo /rko_lio/odometry --once
ros2 topic echo /cmd_vel --once
```

---

## NavigateToPose 的 Feedback 和 goal「对不上」？

**这些字段全部是 `zoh_fb_cmd.py` 里组装的**，不是 Nav2 规划器算的。

| 字段 | 含义 |
|------|------|
| `current_pose` | **直接拷贝** `/rko_lio/odometry` 的 `header` + `pose.pose`（与 `ros2 topic echo` 里看到的 **同一组数**）。**不是 goal**；到位前不会等于目标点。 |
| `distance_remaining` | 在本节点里 = **当前位置到 goal 的平面距离** \(\sqrt{(x_{goal}-x)^2+(y_{goal}-y)^2}\)，与 Nav2 里「沿路径剩余距离」不是同一概念。 |
| `navigation_time` | 从本 goal 开始执行起的累计时间。 |
| `estimated_time_remaining` | 本实现中固定为 0（未估算）。 |
| `number_of_recoveries` | 固定为 0。 |

**示例：** goal 为 `y: 0.3` 时，若 echo 里 odom 的 `y ≈ 0.33`，则 feedback 的 `current_pose.position.y` 也应约为 **0.33**；`distance_remaining` 为到 goal 的平面距离（与 `zoh_rev` 用的 `(p.x,p.y)` 误差一致）。  
若曾发现「echo 是 (0.01, 0.33) 而 feedback 却是 (0.33, -0.01)」，那是旧版 **错误地做了轴交换**；当前已与 `zoh_rev` / `echo` 对齐。  
运动过程中未进容差前位姿会变；`^C` 取消后为 **CANCELED**，属正常。

---

## 常见问题

1. **Action 无反应或一直不成功**  
   - 确认 `zoh_rev.py` 已启动，且 `/rko_lio/odometry` 有数据。  
   - 确认 Action 名称与 `ros2 action list` 一致。

2. **`ModuleNotFoundError: pymbc_msgs`**  
   - 重新编译并 source：  
     `cd /opt/project/robot_base_ctl/ros2_msg && colcon build --packages-select pymbc_msgs && source install/setup.bash`

3. **`nav2_msgs` 找不到**  
   - 安装：`sudo apt install ros-humble-nav2-msgs`

4. **与躯干/全身脚本同时跑**  
   - 使用 `start_torso_follow.sh` / `start_base_torso_follow.sh` 时注意 tmux 会话名，避免重复占同一 session。

5. **`RuntimeError: no running event loop`（已修复）**  
   - 旧版在 Action 里使用 `asyncio.sleep`，在默认 rclpy 执行器下会报错。当前实现已改为同步 `execute` + `time.sleep`，若仍见旧错误请确认已保存并重启 `zoh_fb_cmd.py`。
