# 机器人底盘控制程序


### 使用说明
需要先启动ros 的odometry 节点

python /opt/project/robot_base_ctl/data_20251219_201132_100/data_20251219_201132_100/compare_dataset_vs_odom.py
这个是用来画图的               
python /opt/project/robot_base_ctl/base_ctl/zoh_fb.py
这个是发布/ref_pose用于给底盘，并且打印每一行生成csv        
python /opt/project/rko_lio_ws/bridge.sh
这个是订阅 cmd_vel, 控制底盘运动的      
python /opt/project/robot_base_ctl/base_ctl/zoh_rev.py
这个是接收/ref_pose用于控制底盘
<!-- 遥操作 -->
python /opt/project/robot_base_ctl/motor/moons/joystick_teleop.py