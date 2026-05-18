底盘启动核心思路：
1.把雷达启动起来，同时保证零点设置正确。
2.把底盘三个全向轮控制启动起来，注意当前版本要关掉joystick_teleop.py后启动。
3.把腰部升降电机服务启动起来。
4.在雷达，三个全向轮电机服务，腰部升降电机服务启动后，再启动joystick_workflow_513.py，这个程序会映射按键和对应的action指令，同时这个运行环境要保证和小脑之间是有达到跨主机ros2通讯的。

底盘设置ros2 跨机通讯
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.2.100:7447"]'

底盘通讯测试
ros2 run demo_nodes_cpp listener


小脑执行
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
unset ZENOH_CONFIG_OVERRIDE

小脑通讯测试
ros2 run demo_nodes_cpp talker


运行终端1，初始化升降柱子的运行环境
source /opt/project/robot_base_ctl/tosor_ctl/phi_motion_torso/ros2_ws/install/setup.bash
source /opt/project/robot_base_ctl/ros2_msg/install/setup.bash
ros2 launch phi_motion_torso torso_control.launch.py


读取状态终端 读取升降柱子高度
source /opt/project/robot_base_ctl/tosor_ctl/phi_motion_torso/ros2_ws/install/setup.bash

ros2 topic echo /phi/motion/torso/status

-------------------------------------------------
在起始点开启导航
bash /opt/project/rko_lio_ws/start_rko.sh stop
bash /opt/project/rko_lio_ws/start_rko.sh start
ros2 topic echo /rko_lio/odometry --field pose.pose


运行终端2 启动底盘运行
bash /opt/project/robot_base_ctl/start.sh stop
bash /opt/project/robot_base_ctl/start.sh start

python /opt/project/robot_base_ctl/base_ctl/zoh_fb_cmd_mbc_action.py



运行终端3 启动手柄遥取料工位和挂料工位，此终端需要ros2跨主机通讯
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.2.100:7447"]'

source /opt/project/robot_base_ctl/tosor_ctl/phi_motion_torso/ros2_ws/install/setup.bash
source /opt/project/robot_base_ctl/ros2_msg/install/setup.bash
python /opt/project/robot_base_ctl/joystick/joystick_workflow_513.py







source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
unset ZENOH_CONFIG_OVERRIDE

bash /opt/project/robot_base_ctl/start.sh stop
python /opt/project/robot_base_ctl/joystick/joystick_teleop.py