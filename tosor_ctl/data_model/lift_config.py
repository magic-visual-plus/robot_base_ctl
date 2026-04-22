from dataclasses import dataclass


@dataclass
class LiftConfig:
    eds_file: str = "/home/nvidia/robot_base_ctl/motor/moons/CANOPEN-EDS-MBDV-Servo-SingleAxis-V1.1.1.eds"
    channel: str = "can0"
    bitrate: int = 1000000
    node_id: int = 4

    # TPDO2 position + RPDO2 command
    position_tpdo_index: int = 2
    cmd_rpdo_index: int = 2

    control_word_name: str = "Control word"
    target_position_name: str = "Target position"

    status_print_period: float = 0.1
    pp_wait_timeout: float = 120.0
    # Communication period (ms) for PDO / control loops (informational)
    comm_period_ms: float = 10.0

    # Position-based completion detection
    position_tolerance_pulse: int = 2000
    stable_time_sec: float = 0.3

    # Height model
    zero_encoder_height_mm: float = 630.0  # encoder=0 -> 480 mm
    min_height_mm: float = 380.0
    max_height_mm: float = 880.0

    # pulse +1,000,000 -> height +34.25 mm
    mm_per_1m_pulse: float = 34.25

    # PP motion params
    profile_velocity: int = 600000
    profile_acceleration: int = 3000000
    profile_deceleration: int = 3000000

    @property
    def pulse_per_mm(self) -> float:
        return 1000000.0 / self.mm_per_1m_pulse
