#!/usr/bin/env python3
"""
Phi Robot Torso Control Node

ROS2 node for controlling the torso/waist lift mechanism of the Phi robot.
Provides action-based height control, status publishing, and parameter management.

Namespace: /phi/motion/torso
Node name: phi_motion_torso_node

Control Modes:
- TELEOP (0): Teleoperation mode - topic-based control
- MBC (1): Action-based control mode
- JOYSTICK (2): Joystick incremental control mode
"""

import time
import threading
from typing import Optional, Dict
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float64

from phi_motion_torso.action import MoveToHeight
from phi_motion_torso.msg import TorsoStatus
from phi_motion_torso.srv import Initialize, GetMotionParams, SetMotionParams
from phi_motion_torso.srv import GetControlMode, SetControlMode
from phi_motion_torso.controller import LiftColumnController, LiftConfig


class ControlMode(Enum):
    TELEOP = 0
    MBC = 1
    JOYSTICK = 2

    @classmethod
    def from_int(cls, value: int) -> 'ControlMode':
        if value == 0:
            return cls.TELEOP
        elif value == 1:
            return cls.MBC
        elif value == 2:
            return cls.JOYSTICK
        raise ValueError(f"Invalid control mode: {value}")

    def to_string(self) -> str:
        return {
            ControlMode.TELEOP: "TELEOP",
            ControlMode.MBC: "MBC",
            ControlMode.JOYSTICK: "JOYSTICK"
        }[self]


def control_mode_from_launch_string(value: object) -> ControlMode:
    """
    Launch / params string -> ControlMode.

    Supported aliases:
      action, mbc        -> MBC (MoveToHeight action)
      topic, teleop      -> TELEOP (target_height_cmd)
      joystick           -> JOYSTICK (joystick_increment)
    """
    s = str(value).strip().lower()
    if s in ('action', 'mbc'):
        return ControlMode.MBC
    if s in ('topic', 'teleop'):
        return ControlMode.TELEOP
    if s in ('joystick',):
        return ControlMode.JOYSTICK
    raise ValueError(
        f"Invalid control_mode {value!r}; "
        f"expected one of: action, mbc, topic, teleop, joystick"
    )


class PhiMotionTorsoNode(Node):
    """
    ROS2 node for Phi robot torso control.

    This node wraps the CANopen lift controller and provides ROS2 interfaces:
    - Action: /phi/motion/torso/move_to_height (MBC mode only)
    - Topic: /phi/motion/torso/status
    - Topic: /phi/motion/torso/target_height_cmd (TELEOP mode only)
    - Topic: /phi/motion/torso/joystick_increment (JOYSTICK mode only)
    - Services: initialize, get_motion_params, set_motion_params
    - Services: get_mode, set_mode
    """

    def __init__(self):
        super().__init__('phi_motion_torso_node')

        # Use reentrant callback group for concurrent service/action handling
        self.callback_group = ReentrantCallbackGroup()

        self.mode_lock = threading.Lock()

        # Declare parameters
        self._declare_parameters()
        try:
            cm_raw = self.get_parameter('control_mode').value
            self.current_mode = control_mode_from_launch_string(cm_raw)
        except ValueError as e:
            self.get_logger().error(f"{e}; falling back to TELEOP")
            self.current_mode = ControlMode.TELEOP

        # Initialize controller (will be connected during startup)
        self.controller: Optional[LiftColumnController] = None
        self.initialized = False

        # Action server state
        self.current_goal_handle = None
        self.goal_lock = threading.Lock()

        # Create ROS2 interfaces
        self._create_status_publisher()
        self._create_services()
        self._create_action_server()
        self._create_topic_subscriptions()

        self.get_logger().info("Phi Motion Torso Node started")
        self.get_logger().info("Namespace: /phi/motion/torso")
        self.get_logger().info(f"Initial control mode: {self.current_mode.to_string()}")
        
        # Auto-initialize controller on startup using current parameters
        self._auto_initialize_controller()

    def _auto_initialize_controller(self):
        """Automatically initialize controller at node startup"""
        if self.initialized:
            return

        try:
            config = LiftConfig(
                eds_file=self.get_parameter('eds_file').value,
                channel=self.get_parameter('can_channel').value,
                bitrate=self.get_parameter('can_bitrate').value,
                node_id=self.get_parameter('node_id').value,
                feedback_frequency_hz=self.get_parameter('feedback_frequency_hz').value,
            )

            self.controller = LiftColumnController(config)
            self.controller.initialize()
            self.initialized = True
            self.get_logger().info("Controller auto-initialized successfully on startup")
        except Exception as e:
            self.initialized = False
            self.controller = None
            self.get_logger().error(
                f"Auto-initialization failed on startup: {e}. "
                f"You can retry via /phi/motion/torso/initialize service."
            )

    def _declare_parameters(self):
        """Declare ROS2 parameters"""
        cfg = LiftConfig()
        self.declare_parameter('eds_file', cfg.eds_file)
        self.declare_parameter('can_channel', cfg.channel)
        self.declare_parameter('can_bitrate', cfg.bitrate)
        self.declare_parameter('node_id', cfg.node_id)
        self.declare_parameter('status_publish_rate', 50.0)  # Hz
        self.declare_parameter('feedback_frequency_hz', 100)  # TPDO feedback frequency
        # Same vocabulary as torso_control.launch.py `control_mode`:
        # action | topic | joystick (aliases: mbc, teleop)
        self.declare_parameter('control_mode', 'action')
        # Deprecated: use control_mode; kept for older YAML that never set control_mode
        self.declare_parameter('initial_control_mode', 'teleop')

    def _create_action_server(self):
        """Create action server for move_to_height"""
        self.action_server = ActionServer(
            self,
            MoveToHeight,
            '/phi/motion/torso/move_to_height',
            execute_callback=self.execute_move_to_height,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        self.get_logger().info("Action server created: /phi/motion/torso/move_to_height")

    def _create_status_publisher(self):
        """Create status topic publisher"""
        self.status_pub = self.create_publisher(
            TorsoStatus,
            '/phi/motion/torso/status',
            10
        )

        # Create timer for status publishing
        rate = self.get_parameter('status_publish_rate').value
        period = 1.0 / rate
        self.status_timer = self.create_timer(period, self.publish_status)

        self.get_logger().info(f"Status publisher created: /phi/motion/torso/status ({rate} Hz)")

    def _create_topic_subscriptions(self):
        """Create topic subscriptions for different control modes"""
        # TELEOP mode: direct height command
        self.height_cmd_sub = self.create_subscription(
            Float64,
            '/phi/motion/torso/target_height_cmd',
            self.handle_height_command,
            10,
        )

        # JOYSTICK mode: incremental command
        self.joystick_increment_sub = self.create_subscription(
            Float64,
            '/phi/motion/torso/joystick_increment',
            self.handle_joystick_increment,
            10,
        )

        self.get_logger().info("Topic subscriptions created")

    def _create_services(self):
        """Create service servers"""
        self.init_service = self.create_service(
            Initialize,
            '/phi/motion/torso/initialize',
            self.handle_initialize,
            callback_group=self.callback_group
        )

        self.get_params_service = self.create_service(
            GetMotionParams,
            '/phi/motion/torso/get_motion_params',
            self.handle_get_motion_params,
            callback_group=self.callback_group
        )

        self.set_params_service = self.create_service(
            SetMotionParams,
            '/phi/motion/torso/set_motion_params',
            self.handle_set_motion_params,
            callback_group=self.callback_group
        )

        # Control mode services
        self.get_mode_service = self.create_service(
            GetControlMode,
            '/phi/motion/torso/get_mode',
            self.handle_get_mode,
            callback_group=self.callback_group
        )

        self.set_mode_service = self.create_service(
            SetControlMode,
            '/phi/motion/torso/set_mode',
            self.handle_set_mode,
            callback_group=self.callback_group
        )

        self.get_logger().info("Services created: initialize, get_motion_params, set_motion_params, get_mode, set_mode")

    # -------------------------------------------------------------------------
    # Control Mode Service handlers
    # -------------------------------------------------------------------------
    def handle_get_mode(self, request, response):
        """Handle get control mode service request"""
        with self.mode_lock:
            mode = self.current_mode
        
        response.success = True
        response.message = "Control mode retrieved"
        response.mode = mode.value
        response.mode_name = mode.to_string()
        
        self.get_logger().info(f"Get mode: {mode.to_string()} ({mode.value})")
        return response

    def handle_set_mode(self, request, response):
        """Handle set control mode service request"""
        try:
            new_mode = ControlMode.from_int(request.mode)
        except ValueError as e:
            response.success = False
            response.message = str(e)
            response.current_mode = self.current_mode.value
            response.current_mode_name = self.current_mode.to_string()
            self.get_logger().error(f"Invalid mode requested: {request.mode}")
            return response

        with self.mode_lock:
            old_mode = self.current_mode
            self.current_mode = new_mode

        response.success = True
        response.message = f"Control mode changed from {old_mode.to_string()} to {new_mode.to_string()}"
        response.current_mode = new_mode.value
        response.current_mode_name = new_mode.to_string()

        self.get_logger().info(f"Set mode: {old_mode.to_string()} -> {new_mode.to_string()}")
        return response

    # -------------------------------------------------------------------------
    # Service handlers
    # -------------------------------------------------------------------------
    def handle_initialize(self, request, response):
        """Handle initialization service request (manual re-init or retry)"""
        if self.initialized:
            response.success = True
            response.message = "Already initialized"
            self.get_logger().info("Initialize called but already initialized")
            return response

        try:
            # Create configuration from parameters
            config = LiftConfig(
                eds_file=self.get_parameter('eds_file').value,
                channel=self.get_parameter('can_channel').value,
                bitrate=self.get_parameter('can_bitrate').value,
                node_id=self.get_parameter('node_id').value,
                feedback_frequency_hz=self.get_parameter('feedback_frequency_hz').value,
            )

            # Create and initialize controller
            self.controller = LiftColumnController(config)
            self.controller.initialize()

            self.initialized = True
            response.success = True
            response.message = "Initialization successful"
            self.get_logger().info("Controller initialized successfully")

        except Exception as e:
            response.success = False
            response.message = f"Initialization failed: {str(e)}"
            self.get_logger().error(f"Initialization failed: {e}")

        return response

    def handle_get_motion_params(self, request, response):
        """Handle get motion parameters service request"""
        if not self.initialized or self.controller is None:
            response.success = False
            response.message = "Not initialized. Call initialize service first."
            return response

        try:
            params = self.controller.get_motion_params()

            response.success = True
            response.message = "Motion parameters retrieved"
            response.profile_velocity = params.get('profile_velocity', 0)
            response.profile_acceleration = params.get('profile_acceleration', 0)
            response.profile_deceleration = params.get('profile_deceleration', 0)

            self.get_logger().info(f"Motion params: vel={response.profile_velocity}, "
                                 f"acc={response.profile_acceleration}, "
                                 f"dec={response.profile_deceleration}")

        except Exception as e:
            response.success = False
            response.message = f"Failed to get motion params: {str(e)}"
            self.get_logger().error(f"Get motion params failed: {e}")

        return response

    def handle_set_motion_params(self, request, response):
        """Handle set motion parameters service request"""
        if not self.initialized or self.controller is None:
            response.success = False
            response.message = "Not initialized. Call initialize service first."
            return response

        try:
            # Set parameters (0 or negative = keep current)
            velocity = request.profile_velocity if request.profile_velocity > 0 else None
            acceleration = request.profile_acceleration if request.profile_acceleration > 0 else None
            deceleration = request.profile_deceleration if request.profile_deceleration > 0 else None

            # Apply and read back
            params = self.controller.set_motion_params(velocity, acceleration, deceleration)

            response.success = True
            response.message = "Motion parameters set successfully"
            response.profile_velocity = params.get('profile_velocity', 0)
            response.profile_acceleration = params.get('profile_acceleration', 0)
            response.profile_deceleration = params.get('profile_deceleration', 0)

            self.get_logger().info(f"Motion params set: vel={response.profile_velocity}, "
                                 f"acc={response.profile_acceleration}, "
                                 f"dec={response.profile_deceleration}")

        except Exception as e:
            response.success = False
            response.message = f"Failed to set motion params: {str(e)}"
            self.get_logger().error(f"Set motion params failed: {e}")

        return response

    # -------------------------------------------------------------------------
    # Topic-driven control handlers
    # -------------------------------------------------------------------------
    def handle_height_command(self, msg: Float64) -> None:
        """Handle direct height command (TELEOP mode only)"""
        with self.mode_lock:
            mode = self.current_mode
        
        # Only respond in TELEOP mode
        if mode != ControlMode.TELEOP:
            self.get_logger().warn(
                f"Height command received but not in TELEOP mode (current: {mode.to_string()}), ignoring",
                throttle_duration_sec=5.0,
            )
            return

        if not self.initialized or self.controller is None:
            self.get_logger().warn(
                "Received height command before controller initialized, ignoring",
                throttle_duration_sec=5.0,
            )
            return

        with self.goal_lock:
            if self.current_goal_handle is not None and self.current_goal_handle.is_active:
                return

        target_height = float(msg.data)
        success, message, _ = self.controller.move_to_height_mm_async(target_height)
        if not success:
            self.get_logger().warn(f"TELEOP height command rejected: {message}")
        else:
            self.get_logger().info(f"TELEOP height command accepted: {target_height:.2f} mm")

    def handle_joystick_increment(self, msg: Float64) -> None:
        """Handle joystick incremental command (JOYSTICK mode only)"""
        with self.mode_lock:
            mode = self.current_mode
        
        # Only respond in JOYSTICK mode
        if mode != ControlMode.JOYSTICK:
            self.get_logger().warn(
                f"Joystick increment received but not in JOYSTICK mode (current: {mode.to_string()}), ignoring",
                throttle_duration_sec=5.0,
            )
            return

        if not self.initialized or self.controller is None:
            self.get_logger().warn(
                "Received joystick increment before controller initialized, ignoring",
                throttle_duration_sec=5.0,
            )
            return

        with self.goal_lock:
            if self.current_goal_handle is not None and self.current_goal_handle.is_active:
                return

        # Calculate target height based on current position + increment
        increment_mm = float(msg.data)
        current_height = self.controller.get_current_height_mm()
        target_height = current_height + increment_mm

        # Validate target height
        if not self.controller.is_valid_height(target_height):
            self.get_logger().warn(
                f"JOYSTICK target height out of range: {target_height:.2f} mm (increment: {increment_mm:.2f} mm)"
            )
            return

        success, message, _ = self.controller.move_to_height_mm_async(target_height)
        if not success:
            self.get_logger().warn(f"JOYSTICK command rejected: {message}")
        else:
            self.get_logger().info(f"JOYSTICK command accepted: current={current_height:.2f} mm, increment={increment_mm:.2f} mm, target={target_height:.2f} mm")

    # -------------------------------------------------------------------------
    # Action server callbacks
    # -------------------------------------------------------------------------
    def goal_callback(self, goal_request):
        """Handle new goal request (MBC mode only)"""
        with self.mode_lock:
            mode = self.current_mode
        
        # Only accept goals in MBC mode
        if mode != ControlMode.MBC:
            self.get_logger().warn(f"Goal rejected: control_mode is {mode.to_string()}, expected MBC")
            return GoalResponse.REJECT

        if not self.initialized or self.controller is None:
            self.get_logger().warn("Goal rejected: not initialized")
            return GoalResponse.REJECT

        # Check if height is valid
        if not self.controller.is_valid_height(goal_request.target_height_mm):
            self.get_logger().warn(
                f"Goal rejected: invalid height {goal_request.target_height_mm:.2f} mm "
                f"(valid range: {self.controller.cfg.min_height_mm:.2f} ~ "
                f"{self.controller.cfg.max_height_mm:.2f} mm)"
            )
            return GoalResponse.REJECT

        # Check if another goal is active
        with self.goal_lock:
            if self.current_goal_handle is not None and self.current_goal_handle.is_active:
                self.get_logger().warn("Goal rejected: another goal is already active")
                return GoalResponse.REJECT

        self.get_logger().info(f"Goal accepted: move to {goal_request.target_height_mm:.2f} mm")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle goal cancellation request"""
        self.get_logger().info("Goal cancellation requested")
        return CancelResponse.ACCEPT

    async def execute_move_to_height(self, goal_handle):
        """Execute move to height action"""
        self.get_logger().info(f"Executing move to height: {goal_handle.request.target_height_mm:.2f} mm")

        with self.goal_lock:
            self.current_goal_handle = goal_handle

        try:
            target_height_mm = goal_handle.request.target_height_mm

            # Start motion
            success, message, target_pos = self.controller.move_to_height_mm_async(target_height_mm)

            if not success:
                result = MoveToHeight.Result()
                result.success = False
                result.message = message
                result.final_position_pulse = self.controller.get_current_position()
                result.final_height_mm = self.controller.get_current_height_mm()
                goal_handle.abort()
                return result

            # Monitor progress and publish feedback
            feedback = MoveToHeight.Feedback()
            stable_start = None
            rate = self.create_rate(20)  # 20 Hz feedback

            while rclpy.ok():
                # Check for cancellation
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = MoveToHeight.Result()
                    result.success = False
                    result.message = "Goal canceled by client"
                    result.final_position_pulse = self.controller.get_current_position()
                    result.final_height_mm = self.controller.get_current_height_mm()
                    self.get_logger().info("Goal canceled")
                    return result

                # Check position
                in_window, pos_error, has_data = self.controller.check_position_reached(target_pos)

                if has_data:
                    current_pos = self.controller.get_current_position()
                    current_height = self.controller.pulse_to_height_mm(current_pos)

                    # Publish feedback（只保留当前高度）
                    feedback.current_height_mm = current_height
                    goal_handle.publish_feedback(feedback)

                    # Check completion
                    if in_window:
                        if stable_start is None:
                            stable_start = time.time()
                        elif time.time() - stable_start >= self.controller.cfg.stable_time_sec:
                            # Motion complete
                            result = MoveToHeight.Result()
                            result.success = True
                            result.message = "Target height reached"
                            result.final_position_pulse = current_pos
                            result.final_height_mm = current_height
                            goal_handle.succeed()
                            self.get_logger().info(f"Goal succeeded: reached {current_height:.2f} mm")
                            return result
                    else:
                        stable_start = None

                rate.sleep()

        except Exception as e:
            self.get_logger().error(f"Action execution failed: {e}")
            result = MoveToHeight.Result()
            result.success = False
            result.message = f"Execution error: {str(e)}"
            result.final_position_pulse = self.controller.get_current_position() if self.controller else 0
            result.final_height_mm = self.controller.get_current_height_mm() if self.controller else 0.0
            goal_handle.abort()
            return result

        finally:
            with self.goal_lock:
                self.current_goal_handle = None

    # -------------------------------------------------------------------------
    # Status publishing
    # -------------------------------------------------------------------------
    def publish_status(self):
        """Publish current torso status"""
        if not self.initialized or self.controller is None:
            return

        try:
            snap = self.controller.get_status_snapshot()

            msg = TorsoStatus()

            # 当前高度
            if snap["position"] is not None:
                msg.current_height_mm = snap["height_mm"]
            else:
                msg.current_height_mm = 0.0

            # 目标高度 + 状态
            with self.goal_lock:
                if self.current_goal_handle is not None and self.current_goal_handle.is_active:
                    msg.has_active_goal = True
                    msg.target_height_mm = self.current_goal_handle.request.target_height_mm

                    # 到位窗口判断
                    target_pulse = self.controller.height_mm_to_pulse(msg.target_height_mm)
                    in_window, _, _ = self.controller.check_position_reached(target_pulse)
                    msg.in_position_window = in_window
                else:
                    msg.has_active_goal = False
                    msg.target_height_mm = msg.current_height_mm
                    msg.in_position_window = True

            msg.status_word = int(snap.get("status_word", 0)) & 0xFFFF
            msg.status_word_valid = bool(snap.get("status_word_valid", False))

            self.status_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Status publish failed: {e}", throttle_duration_sec=5.0)

    # -------------------------------------------------------------------------
    # Shutdown
    # -------------------------------------------------------------------------
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down Phi Motion Torso Node")

        if self.controller is not None:
            self.controller.shutdown()

        self.get_logger().info("Shutdown complete")


def main(args=None):
    rclpy.init(args=args)

    node = PhiMotionTorsoNode()

    # Use multi-threaded executor for concurrent callbacks
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
