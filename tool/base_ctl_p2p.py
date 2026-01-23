#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
pid_go_to_goal_and_log.py

ROS2 node:
- Sub:  /odom (nav_msgs/Odometry)  -> read x,y,yaw
- Pub:  /cmd_vel (geometry_msgs/Twist) -> send vx, vy, wz (for omni base)
- Pub:  /pid_path (nav_msgs/Path) -> visualize trajectory in RViz
- Log:  write trajectory to CSV (time,x,y,yaw,cmd_vx,cmd_vy,cmd_wz)

Usage:
  ros2 run <your_pkg> pid_go_to_goal_and_log.py --ros-args \
    -p odom_topic:=/rko_lio/odometry -p cmd_topic:=/cmd_vel \
    -p goal_x:=1.0 -p goal_y:=0.0 -p goal_yaw:=0.0 \
    -p out_csv:=/tmp/traj.csv

Or run directly:
  python3 pid_go_to_goal_and_log.py --ros-args -p goal_x:=1.0 -p goal_y:=0.0
"""
import time
import math
import csv
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from geometry_msgs.msg import TwistStamped


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    # yaw (z-axis rotation)
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


@dataclass
class PIDGains:
    kp: float
    ki: float
    kd: float
    i_max: float


class PID:
    def __init__(self, gains: PIDGains):
        self.g = gains
        self.i = 0.0
        self.e_prev: Optional[float] = None

    def reset(self):
        self.i = 0.0
        self.e_prev = None

    def step(self, e: float, dt: float, freeze_integrator: bool = False) -> float:
        if dt <= 0.0:
            return 0.0

        if self.e_prev is None:
            de = 0.0
        else:
            de = (e - self.e_prev) / dt
        self.e_prev = e

        if not freeze_integrator:
            self.i += e * dt
            self.i = clamp(self.i, -self.g.i_max, self.g.i_max)

        return self.g.kp * e + self.g.ki * self.i + self.g.kd * de



class PIDGoToGoal(Node):
    def __init__(self):
        super().__init__("pid_go_to_goal_and_log")
        odom_topic_name = "/rko_lio/odometry"
        # Params
        self.declare_parameter("odom_topic", odom_topic_name)
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("path_topic", "/pid_path")
        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_yaw", 0.0)  # rad
        self.declare_parameter("control_hz", 50.0)

        self.declare_parameter("pos_tol", 0.03)     # m
        self.declare_parameter("yaw_tol", 0.08)     # rad
        self.declare_parameter("slow_dist", 0.01)   # m (within this, scale down vx/vy)

        # speed limits (body frame)
        self.declare_parameter("vx_max", 0.5)      # m/s
        self.declare_parameter("vy_max", 0.5)      # m/s
        self.declare_parameter("wz_max", 2.0)       # rad/s (increased for faster rotation)

        # PID gains (start conservative; tune later)
        self.declare_parameter("kp_xy", 2) #1.2
        self.declare_parameter("ki_xy", 0.0)
        self.declare_parameter("kd_xy", 0.1)
        self.declare_parameter("imax_xy", 0.3)

        self.declare_parameter("kp_yaw", 2.0) 
        self.declare_parameter("ki_yaw", 0.0)
        self.declare_parameter("kd_yaw", 0.3) #0.2
        self.declare_parameter("imax_yaw", 0.5)

        # logging
        self.declare_parameter("out_csv", "/opt/project/robot_base_ctl/pid_traj.csv")
        self.declare_parameter("log_every_n", 1)  # log every N control ticks

        self.odom_topic = self.get_parameter("odom_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.path_topic = self.get_parameter("path_topic").value

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.goal_yaw = float(self.get_parameter("goal_yaw").value)

        self.control_hz = float(self.get_parameter("control_hz").value)
        self.dt = 1.0 / max(1e-6, self.control_hz)

        self.pos_tol = float(self.get_parameter("pos_tol").value)
        self.yaw_tol = float(self.get_parameter("yaw_tol").value)
        self.slow_dist = float(self.get_parameter("slow_dist").value)

        self.vx_max = float(self.get_parameter("vx_max").value)
        self.vy_max = float(self.get_parameter("vy_max").value)
        self.wz_max = float(self.get_parameter("wz_max").value)

        gxy = PIDGains(
            kp=float(self.get_parameter("kp_xy").value),
            ki=float(self.get_parameter("ki_xy").value),
            kd=float(self.get_parameter("kd_xy").value),
            i_max=float(self.get_parameter("imax_xy").value),
        )
        gyaw = PIDGains(
            kp=float(self.get_parameter("kp_yaw").value),
            ki=float(self.get_parameter("ki_yaw").value),
            kd=float(self.get_parameter("kd_yaw").value),
            i_max=float(self.get_parameter("imax_yaw").value),
        )

        self.pid_x = PID(gxy)
        self.pid_y = PID(gxy)
        self.pid_yaw = PID(gyaw)

        # State from odom
        self.have_odom = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # ROS I/O
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.cb_odom, 50)
        self.pub_cmd = self.create_publisher(Twist, self.cmd_topic, 10)
        self.pub_path = self.create_publisher(Path, self.path_topic, 10)
        self.fb_vx = 0.0
        self.fb_vy = 0.0
        self.sub_fb = self.create_subscription(
            TwistStamped, "/base_vel_fb", self.cb_fb, 10
        )
        # Path msg
        self.path = Path()
        self.path.header.frame_id = "odom"  # will update to match odom.header.frame_id if possible

        # CSV
        self.out_csv = self.get_parameter("out_csv").value
        self.log_every_n = int(self.get_parameter("log_every_n").value)
        self._tick = 0
        self._csv_file = open(self.out_csv, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(["t_sec","x","y","yaw","err_x","err_y","err_yaw","cmd_vx","cmd_vy","cmd_wz","fb_vx","fb_vy"])


        # Timer control loop
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f"PID go-to-goal running. goal=({self.goal_x:.3f},{self.goal_y:.3f}, yaw={self.goal_yaw:.3f}rad) "
            f"odom={self.odom_topic} cmd={self.cmd_topic} csv={self.out_csv}"
        )

    def destroy_node(self):
        try:
            self.stop_robot()
        except Exception:
            pass
        try:
            self._csv_file.flush()
            self._csv_file.close()
        except Exception:
            pass
        super().destroy_node()
    
    def cb_fb(self, msg: TwistStamped):
        self.fb_vx = float(msg.twist.linear.x)
        self.fb_vy = float(msg.twist.linear.y)

    def cb_odom(self, msg: Odometry):
        self.have_odom = True
        # Coordinate transformation: odom_x = robot_(-y), odom_y = robot_x
        # So: robot_x = odom_y, robot_y = -odom_x
        odom_x = float(msg.pose.pose.position.x)
        odom_y = float(msg.pose.pose.position.y)
        # self.x = odom_y   # robot x = odom y
        # self.y = -odom_x  # robot y = -odom x
        
        self.x = odom_x
        self.y = odom_y
        # TODO enable later 
        q = msg.pose.pose.orientation
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        # self.yaw = 0

        # Keep path frame consistent with odom
        if msg.header.frame_id:
            self.path.header.frame_id = msg.header.frame_id

        # Append pose to path for RViz
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(ps)

        # Avoid unlimited growth (keep last N points)
        if len(self.path.poses) > 5000:
            self.path.poses = self.path.poses[-5000:]

        self.pub_path.publish(self.path)

    def stop_robot(self):
        tw = Twist()
        self.pub_cmd.publish(tw)

    def control_loop(self):
        if not self.have_odom:
            return

        # world-frame position error
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = math.hypot(dx, dy)

        # yaw error
        eyaw = wrap_to_pi(self.goal_yaw - self.yaw)

        # rotate world error into body frame (base_link-like)
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        ex =  cy * dx + sy * dy
        ey = -sy * dx + cy * dy

        # near-goal check
        if dist < self.pos_tol and abs(eyaw) < self.yaw_tol:
            self.stop_robot()
            if self._tick % int(self.control_hz) == 0:  # about 1Hz print
                self.get_logger().info(f"Reached goal. dist={dist:.3f} yaw_err={eyaw:.3f}. stopped.")
            return

        # PID compute
        # If we saturate heavily, you may freeze integrator; here we keep it simple and rely on i_max + small Ki.
        vx = self.pid_x.step(ex, self.dt)
        vy = self.pid_y.step(ey, self.dt)
        wz = self.pid_yaw.step(eyaw, self.dt)

        # slow down near target (position only)
        if self.slow_dist > 1e-6:
            scale = min(1.0, dist / self.slow_dist)
            scale = max(scale, 0.12) # 最低保留 20% 力度（你可以 0.15~0.3 试）
            vx *= scale
            vy *= scale

        # clamp outputs
        vx = clamp(vx, -self.vx_max, self.vx_max)
        vy = clamp(vy, -self.vy_max, self.vy_max)
        wz = clamp(wz, -self.wz_max, self.wz_max)

        # publish
        tw = Twist()
        tw.linear.x = float(vx)
        tw.linear.y = float(vy)
        tw.angular.z = float(wz)
        self.get_logger().info(f"pub cmd: {tw}")
        self.pub_cmd.publish(tw)

        # log
        self._tick += 1
        if self.log_every_n > 0 and (self._tick % self.log_every_n == 0):
            t = self.get_clock().now().nanoseconds * 1e-9
            self._csv.writerow([f"{t:.6f}", f"{self.x:.6f}", f"{self.y:.6f}", f"{self.yaw:.6f}",
                                f"{ex:.6f}", f"{ey:.6f}", f"{eyaw:.6f}",
                                f"{vx:.6f}", f"{vy:.6f}", f"{wz:.6f}",
                                f"{self.fb_vx:.6f}", f"{self.fb_vy:.6f}"])
            # flush sometimes
            if self._tick % int(self.control_hz) == 0:
                self._csv_file.flush()

        # optional debug print (light)
        if self._tick % int(self.control_hz) == 0:
            self.get_logger().info(
                f"pos=({self.x:.2f},{self.y:.2f}) yaw={self.yaw:.2f} | "
                f"dist={dist:.2f} ex={ex:.2f} ey={ey:.2f} eyaw={eyaw:.2f} | "
                f"cmd(vx,vy,wz)=({vx:.2f},{vy:.2f},{wz:.2f})"
            )




def main():
    rclpy.init()
    node = PIDGoToGoal()
    t0 = time.perf_counter()          # ✅ 开始计时（高精度）
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        dt = time.perf_counter() - t0 # ✅ 结束计时
        print(f"\nRUN TIME = {dt:.3f} s\n")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()


