# 机器人底盘控制程序

**调用关系、数据流、分层说明**：见 [`docs/CONTROL_STACK.md`](docs/CONTROL_STACK.md)。

## 使用说明
需要先启动ros 的odometry 节点


### 画图
python /opt/project/robot_base_ctl/data_20251219_201132_100/data_20251219_201132_100/compare_dataset_vs_odom.py

### 测试程序：订阅 whole_body 并发布 /ref_pose
python /opt/project/robot_base_ctl/base_ctl/zoh_fb_cmd.py

### 接收cmd_vel 速度指令，控制底盘
python /opt/project/rko_lio_ws/bridge.sh

### 接收/ref_pose 给的位置指令，控制底盘来运动     
python /opt/project/robot_base_ctl/base_ctl/zoh_rev.py

### 遥操作指令
python /opt/project/robot_base_ctl/motor/moons/joystick_teleop.py

### 手柄经 ROS 与唯一底盘节点共用 CAN（推荐架构）
**只控底盘、用手柄发 `/cmd_vel` 时，不要跑 `./start.sh start`**：里面会起 `zoh_rev.py`，也会发 `/cmd_vel`，和手柄抢话题。

任选其一：

```bash
# 方式 A：tmux 只起 bridge（推荐）
cd /opt/project/robot_base_ctl
./start_bridge_only.sh start
source /opt/ros/humble/setup.bash
cd joystick && python3 joystick_teleop_ros.py
```

```bash
# 方式 B：两个终端各起一个进程
python3 /opt/project/robot_base_ctl/motor/moons/bridge.py
python3 /opt/project/robot_base_ctl/joystick/joystick_teleop_ros.py
```

见 `joystick/joystick_teleop_ros.py` 文件头说明；`./start.sh start` 留给「里程计 + /ref_pose + zoh_rev 闭环」那条链路。


### 使用launch 同时启动 zoh_rev 和 bridge
ros2 launch robot_base_ctl launch.py