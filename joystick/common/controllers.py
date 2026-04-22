import math
import os
from typing import Literal, Optional, cast

from oni_ctrl import LeKiwiBaseConfig, LeKiwiBaseController


class BaseControllerAdapter:
    def __init__(
        self,
        root_dir: str,
        base_velocity_mode: str = "PV",
        *,
        base_active_node_ids: tuple[int, ...] = (1, 2, 3),
        apply_uniform_profile_dynamics: bool = True,
        base_profile_accel_pulses_s2: int = 3_600_000,
        base_profile_decel_pulses_s2: int = 3_600_000,
        motor_diagnostics_profile: str = "tongchuan_mdx",
        motor_diagnostics_config_path: Optional[str] = None,
        motor_diagnostics_debug: bool = False,
    ):
        eds_path = os.path.normpath(
            os.path.join(root_dir, "..", "hardware", "canopen", "eds", "CANOPEN-EDS-MBDV-Servo-DulAxes-V1.0.eds")
        )
        bvm = base_velocity_mode.upper()
        if bvm not in ("PV", "CSV"):
            bvm = "PV"
        bvm_lit = cast(Literal["PV", "CSV"], bvm)
        cfg = LeKiwiBaseConfig(
            can_channel="can0",
            bitrate=1_000_000,
            eds_path=eds_path,
            encoder_cpr=2 ** 16,
            gear_ratio=10.0,
            wheel_radius=0.1015,
            base_radius=0.203,
            max_wheel_rad_s=4.0 * math.pi,
            dir_left=+1,
            dir_back=+1,
            dir_right=+1,
            base_active_node_ids=base_active_node_ids,
            base_velocity_mode=bvm_lit,
            apply_uniform_profile_dynamics=apply_uniform_profile_dynamics,
            base_profile_accel_pulses_s2=base_profile_accel_pulses_s2,
            base_profile_decel_pulses_s2=base_profile_decel_pulses_s2,
            motor_diagnostics_profile=motor_diagnostics_profile,
            motor_diagnostics_config_path=motor_diagnostics_config_path,
            motor_diagnostics_debug=motor_diagnostics_debug,
        )
        self._ctl = LeKiwiBaseController(cfg)
        self._ctl.connect()

    def set_body_velocity(self, x: float, y: float, theta_deg_s: float) -> None:
        self._ctl.set_body_velocity(x, y, theta_deg_s)

    def poll_mdx_diagnostics_if_needed(self, now_mono: float, *, cooldown_s: float = 0.5) -> None:
        """状态字异常时 SDO 读 0x603F/0x1001/0x200F（见 oni_ctrl / MDX+ 手册）。"""
        self._ctl.poll_mdx_diagnostics_if_needed(now_mono, cooldown_s=cooldown_s)

    def shutdown(self) -> None:
        self._ctl.stop()
        self._ctl.disconnect()


class TorsoControllerAdapter:
    def __init__(self):
        # Lazy import so base-only mode doesn't require CANopen init path.
        from joystick_torso_teleop import LiftColumnPVController, LiftConfig

        self._ctl = LiftColumnPVController(LiftConfig())
        self._ctl.initialize()

    def update_by_buttons(self, btn_tr: bool, hat0y: int) -> None:
        self._ctl.state.update_button_tr(btn_tr)
        self._ctl.state.update_hat0y(hat0y)
        self._ctl.update_command_from_joystick_state()
        self._ctl.apply_command()

    def shutdown(self) -> None:
        self._ctl.shutdown()

