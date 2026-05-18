"""ROS2：执行 513 workflow 的动作序列（底盘/升降/IK），支持可配置步骤。"""

from __future__ import annotations

from typing import Any, Callable, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

try:
    from phi_motion_torso.action import MoveToHeight  # type: ignore
except ImportError:
    MoveToHeight = None  # type: ignore

try:
    from pymbc_msgs.srv import SetWholeBodyIkCartesianMode  # type: ignore
except ImportError:
    SetWholeBodyIkCartesianMode = None  # type: ignore


def _fill_pose_stamped(data: dict[str, Any]) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = str(data.get("frame_id", "map"))
    pos = data.get("position") or {}
    ori = data.get("orientation") or {}
    ps.pose.position.x = float(pos.get("x", 0.0))
    ps.pose.position.y = float(pos.get("y", 0.0))
    ps.pose.position.z = float(pos.get("z", 0.0))
    ps.pose.orientation.x = float(ori.get("x", 0.0))
    ps.pose.orientation.y = float(ori.get("y", 0.0))
    ps.pose.orientation.z = float(ori.get("z", 0.0))
    ps.pose.orientation.w = float(ori.get("w", 1.0))
    return ps


def _fill_srv_request(req: Any, fields: dict[str, Any]) -> None:
    for k, v in fields.items():
        if hasattr(req, k):
            setattr(req, k, type(getattr(req, k))(v))


class Workflow513RosBridge(Node):
    def __init__(
        self,
        ros_cfg: dict[str, Any],
        *,
        logger: Optional[Callable[[str], None]] = None,
        action_timeout_sec: float = 180.0,
    ):
        super().__init__("workflow_513_ros_bridge")
        self._log = logger or (lambda _s: None)
        self._action_timeout_sec = float(action_timeout_sec)

        nav_name = str(ros_cfg.get("navigate_action", "/phi/motion/control/navigate_to_pose"))
        torso_name = str(ros_cfg.get("torso_action", "/phi/motion/torso/move_to_height"))
        ik_name = str(ros_cfg.get("ik_service", "/phi/motion/teleop/set_whole_body_ik_cartesian_mode"))

        self._nav_client = ActionClient(self, NavigateToPose, nav_name)
        self._torso_client = (
            ActionClient(self, MoveToHeight, torso_name) if MoveToHeight else None
        )
        self._ik_client = (
            self.create_client(SetWholeBodyIkCartesianMode, ik_name)
            if SetWholeBodyIkCartesianMode
            else None
        )

        self._nav_name = nav_name
        self._torso_name = torso_name
        self._ik_name = ik_name

    def run_sequence(self, sequence: dict[str, Any], *, ik_first: bool = False) -> bool:
        """顺序执行 sequence。默认: base_goals[] → torso_height_mm → ik_preset。"""
        goals = sequence.get("base_goals") or []
        if isinstance(goals, dict):
            goals = [goals]
        ik = sequence.get("ik_preset")

        pre_ik_ok = True
        if ik_first and ik:
            self._log(f"[ros] IK mode service {self._ik_name} (pre-action) …")
            pre_ik_ok = self._run_ik(ik)
            if not pre_ik_ok:
                self._log(
                    "[ros] IK service (pre-action) FAILED — "
                    "continuing navigate/torso (motion sequence still runs)"
                )

        for i, g in enumerate(goals):
            self._log(
                f"[ros] navigate waypoint {i + 1}/{len(goals)} "
                f"(NavigateToPose {self._nav_name}) …"
            )
            if not self._run_navigate(g):
                self._log(f"[ros] navigate waypoint {i + 1} FAILED")
                return False

        height = sequence.get("torso_height_mm")
        if height is not None:
            self._log(
                f"[ros] torso height {float(height):.3f} mm "
                f"(MoveToHeight {self._torso_name}) …"
            )
            if not self._run_torso(float(height)):
                self._log("[ros] torso FAILED")
                return False

        if ik and not ik_first:
            self._log(f"[ros] IK mode service {self._ik_name} …")
            if not self._run_ik(ik):
                self._log("[ros] IK service FAILED")
                return False

        if ik_first and ik and not pre_ik_ok:
            self._log("[ros] sequence OK (motion done; pre-action IK had failed)")
        else:
            self._log("[ros] sequence OK")
        return True

    def run_steps(
        self,
        steps: list[dict[str, Any]],
        *,
        reverse: bool = False,
        name: str = "transition",
    ) -> bool:
        """执行 YAML 配置步骤（可选择逆序）。"""
        if not steps:
            return True
        ordered = list(reversed(steps)) if reverse else list(steps)
        self._log(
            f"[ros] {name}: execute {len(ordered)} step(s), reverse={str(reverse).lower()}"
        )
        for idx, step in enumerate(ordered, start=1):
            if not isinstance(step, dict):
                self._log(f"[ros] {name} step {idx}: invalid step type={type(step).__name__}")
                return False
            if not self._run_step(step, idx=idx, total=len(ordered), name=name):
                return False
        self._log(f"[ros] {name}: all steps OK")
        return True

    def _run_step(self, step: dict[str, Any], *, idx: int, total: int, name: str) -> bool:
        kind = str(step.get("kind", "")).strip().lower()
        if kind in ("navigate", "base_goal"):
            goal = step.get("goal", step)
            self._log(
                f"[ros] {name} step {idx}/{total}: navigate "
                f"(NavigateToPose {self._nav_name}) …"
            )
            if not isinstance(goal, dict):
                self._log(f"[ros] {name} step {idx}: navigate goal must be object")
                return False
            if not self._run_navigate(goal):
                self._log(f"[ros] {name} step {idx}: navigate FAILED")
                return False
            return True

        if kind in ("torso", "torso_height"):
            height = step.get("height_mm", step.get("torso_height_mm", step.get("value_mm")))
            if height is None:
                self._log(f"[ros] {name} step {idx}: torso height missing")
                return False
            h = float(height)
            self._log(
                f"[ros] {name} step {idx}/{total}: torso {h:.3f} mm "
                f"(MoveToHeight {self._torso_name}) …"
            )
            if not self._run_torso(h):
                self._log(f"[ros] {name} step {idx}: torso FAILED")
                return False
            return True

        if kind in ("ik", "ik_service"):
            req = step.get("preset", step.get("request", step))
            self._log(f"[ros] {name} step {idx}/{total}: IK service {self._ik_name} …")
            if not isinstance(req, dict):
                self._log(f"[ros] {name} step {idx}: IK request must be object")
                return False
            if not self._run_ik(req):
                self._log(f"[ros] {name} step {idx}: IK service FAILED")
                return False
            return True

        self._log(f"[ros] {name} step {idx}: unknown kind={kind!r}")
        return False

    def _wait_future(self, fut, desc: str):
        rclpy.spin_until_future_complete(self, fut, timeout_sec=self._action_timeout_sec)
        if not fut.done():
            self._log(f"[ros] timeout waiting: {desc}")
            return None
        return fut.result()

    def _run_navigate(self, pose_dict: dict[str, Any]) -> bool:
        self._nav_client.wait_for_server(timeout_sec=30.0)
        goal = NavigateToPose.Goal()
        goal.pose = _fill_pose_stamped(pose_dict)
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        send_fut = self._nav_client.send_goal_async(goal)
        gh = self._wait_future(send_fut, "navigate send_goal")
        if gh is None:
            return False
        if not gh.accepted:
            self._log("[ros] navigate goal rejected")
            return False
        result_fut = gh.get_result_async()
        res = self._wait_future(result_fut, "navigate result")
        if res is None:
            try:
                gh.cancel_goal_async()
            except Exception:
                pass
            return False
        st = res.status
        ok = st == GoalStatus.STATUS_SUCCEEDED
        if not ok:
            self._log(f"[ros] navigate ended status={st}")
        return ok

    def _run_torso(self, height_mm: float) -> bool:
        if self._torso_client is None or MoveToHeight is None:
            self._log("[ros] phi_motion_torso MoveToHeight not importable — skip torso")
            return False
        self._torso_client.wait_for_server(timeout_sec=30.0)
        goal = MoveToHeight.Goal()
        goal.target_height_mm = height_mm

        send_fut = self._torso_client.send_goal_async(goal)
        gh = self._wait_future(send_fut, "torso send_goal")
        if gh is None:
            return False
        if not gh.accepted:
            self._log("[ros] torso goal rejected")
            return False
        result_fut = gh.get_result_async()
        res = self._wait_future(result_fut, "torso result")
        if res is None:
            try:
                gh.cancel_goal_async()
            except Exception:
                pass
            return False
        if res.status != GoalStatus.STATUS_SUCCEEDED:
            self._log(f"[ros] torso ended status={res.status}")
            return False
        mr = res.result
        if hasattr(mr, "success") and not mr.success:
            msg = getattr(mr, "message", "")
            self._log(f"[ros] torso result.success=False {msg}")
            return False
        return True

    def _run_ik(self, preset: dict[str, Any]) -> bool:
        if self._ik_client is None or SetWholeBodyIkCartesianMode is None:
            self._log("[ros] pymbc_msgs SetWholeBodyIkCartesianMode missing — skip IK")
            return True
        if not self._ik_client.wait_for_service(timeout_sec=10.0):
            self._log("[ros] IK service not available")
            return False
        req = SetWholeBodyIkCartesianMode.Request()
        _fill_srv_request(req, preset)
        fut = self._ik_client.call_async(req)
        res = self._wait_future(fut, "ik service")
        if res is None:
            return False
        # 常见字段 success；若无则视为成功
        ok = getattr(res, "success", True)
        if hasattr(res, "message") and res.message:
            self._log(f"[ros] IK msg: {res.message}")
        return bool(ok)
