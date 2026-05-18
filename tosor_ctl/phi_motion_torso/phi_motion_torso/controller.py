"""
CANopen DS402 Lift Column Controller for ROS2 Integration
Adapted from base_tosor_ctl.py for use in phi robot motion system
"""

import time
import threading
import logging
import sys
from pathlib import Path
from typing import Optional, Callable

import canopen


logger = logging.getLogger("phi_motion_torso.controller")

# Reuse shared lift configuration from tosor_ctl/data_model/
_TOSOR_CTL_ROOT = Path(__file__).resolve().parents[3]
if str(_TOSOR_CTL_ROOT) not in sys.path:
    sys.path.append(str(_TOSOR_CTL_ROOT))

from data_model.lift_config import LiftConfig  # noqa: E402


class SharedState:
    """Thread-safe state container for position + CiA402 statusword feedback"""

    def __init__(self):
        self._lock = threading.Lock()
        self.position: Optional[int] = None
        self.recv_position_count = 0
        self.last_position_ts: Optional[float] = None
        self.status_word: Optional[int] = None
        self.recv_status_word_count = 0
        self.last_status_word_ts: Optional[float] = None
        self.running = True

    def update_position(self, position: int) -> None:
        with self._lock:
            self.position = int(position)
            self.recv_position_count += 1
            self.last_position_ts = time.time()

    def update_status_word(self, word: int) -> None:
        with self._lock:
            self.status_word = int(word) & 0xFFFF
            self.recv_status_word_count += 1
            self.last_status_word_ts = time.time()

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "position": self.position,
                "recv_position_count": self.recv_position_count,
                "last_position_ts": self.last_position_ts,
                "status_word": self.status_word,
                "recv_status_word_count": self.recv_status_word_count,
                "last_status_word_ts": self.last_status_word_ts,
                "running": self.running,
            }

    def stop(self) -> None:
        with self._lock:
            self.running = False


class LiftColumnController:
    """
    CANopen DS402 controller for lift column with ROS2 integration support.

    This controller manages a single-axis lift column using CANopen protocol
    and DS402 state machine. It provides height-based control with automatic
    pulse/mm conversion and position-based completion detection.
    """

    def __init__(self, config: LiftConfig):
        self.cfg = config
        self.network: Optional[canopen.Network] = None
        self.node: Optional[canopen.BaseNode402] = None

        self.tpdo_position = None
        self.tpdo_status = None
        self.rpdo_cmd = None

        self.state = SharedState()

        # Callback for external position updates (e.g., ROS2 publisher)
        self.position_callback: Optional[Callable[[int], None]] = None

    # -------------------------------------------------------------------------
    # Conversion utilities
    # -------------------------------------------------------------------------
    def pulse_to_height_mm(self, position_pulse: int) -> float:
        """Convert encoder pulse to height in millimeters"""
        return self.cfg.zero_encoder_height_mm + (float(position_pulse) / self.cfg.pulse_per_mm)

    def height_mm_to_pulse(self, height_mm: float) -> int:
        """Convert height in millimeters to encoder pulse"""
        return int(round((float(height_mm) - self.cfg.zero_encoder_height_mm) * self.cfg.pulse_per_mm))

    def is_valid_height(self, height_mm: float) -> bool:
        """Check if height is within valid range"""
        return self.cfg.min_height_mm <= float(height_mm) <= self.cfg.max_height_mm

    # -------------------------------------------------------------------------
    # DS402 state machine helpers
    # -------------------------------------------------------------------------
    def wait_for_state(self, target_state: str, timeout: float = 10.0) -> None:
        """Wait for DS402 state machine to reach target state"""
        assert self.node is not None
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.node.state == target_state:
                return
            time.sleep(0.01)
        raise TimeoutError(f"Timeout waiting for state={target_state}, current={self.node.state}")

    # -------------------------------------------------------------------------
    # Initialization sequence
    # -------------------------------------------------------------------------
    def initialize(self) -> None:
        """Initialize CANopen network and configure drive"""
        logger.info("Initializing lift column controller")
        self.network = canopen.Network()

        self.node = canopen.BaseNode402(self.cfg.node_id, self.cfg.eds_file)
        self.network.add_node(self.node)
        self.node.sdo.RESPONSE_TIMEOUT = 2.0

        self.network.connect(
            interface="socketcan",
            channel=self.cfg.channel,
            bitrate=self.cfg.bitrate
        )
        self.network.check()

        logger.info("NMT before: %s", self.node.nmt.state)
        self.node.nmt.state = "PRE-OPERATIONAL"
        logger.info("NMT after: %s", self.node.nmt.state)

        # Step 4: Check and clear error log (0x1003)
        assert self.node is not None
        logger.info("[4] Checking and clearing error log (0x1003)")
        try:
            error_log = self.node.sdo[0x1003]
            error_count = len(error_log.values())
            if error_count > 0:
                logger.warning("Found %d errors in log", error_count)
                for i, error in enumerate(error_log.values(), 1):
                    logger.warning("  Error %d: 0x%08X", i, error.raw)
            # Clear error log
            self.node.sdo[0x1003][0].raw = 0
            logger.info("Error log cleared")
        except Exception as exc:
            logger.warning("Failed to read/clear error log (0x1003): %s", exc)

        # Step 5: Configure communication parameters (0x1006)
        try:
            if 0x1006 in self.node.sdo.keys():
                # 0x1006 is in microseconds per CiA 301; cfg.comm_period_ms is milliseconds
                comm_period_us = int(self.cfg.comm_period_ms * 1000.0)
                self.node.sdo[0x1006].raw = comm_period_us
                logger.info("0x1006 Communication cycle period = %d us (%.3f ms)",
                            comm_period_us, self.cfg.comm_period_ms)
            else:
                logger.warning("SDO 0x1006 not available on this node")
        except Exception as exc:
            logger.warning("Failed to configure communication period (0x1006): %s", exc)

        self._load_configuration()
        self._read_pdo_configuration()
        self._disable_unused_tpdos()
        self._configure_tpdo_frequency()
        self._setup_state_machine()
        self._enter_operational()
        self._register_pdo_callbacks()
        self._enable_drive()
        self._set_operation_mode_profiled_position()
        self._configure_pp_parameters()

        logger.info("Initialization complete")

    def _load_configuration(self) -> None:
        assert self.node is not None
        logger.info("[1] Load configuration")
        self.node.load_configuration()

    def _read_pdo_configuration(self) -> None:
        assert self.node is not None
        logger.info("[2] Read PDO configuration")
        self.node.tpdo.read()
        self.node.rpdo.read()

        self.tpdo_position = self.node.tpdo[self.cfg.position_tpdo_index]
        self.rpdo_cmd = self.node.rpdo[self.cfg.cmd_rpdo_index]

        st_idx = int(self.cfg.status_tpdo_index)
        if st_idx in self.node.tpdo:
            self.tpdo_status = self.node.tpdo[st_idx]
            logger.info(
                "TPDO status index=%d COB-ID=0x%08X (expect 0x6041)",
                st_idx,
                self.tpdo_status.cob_id,
            )
        else:
            logger.warning(
                "TPDO index %d not present — statusword via SDO fallback only",
                st_idx,
            )

        logger.info(
            "TPDO position index=%d COB-ID=0x%08X",
            self.cfg.position_tpdo_index, self.tpdo_position.cob_id
        )
        logger.info(
            "RPDO cmd index=%d COB-ID=0x%08X",
            self.cfg.cmd_rpdo_index, self.rpdo_cmd.cob_id
        )

    def _disable_unused_tpdos(self) -> None:
        """
        Disable TPDO4 feedback stream (0x480 + node_id) to reduce CAN load.
        Keep TPDO used for position feedback (cfg.position_tpdo_index, default=2).
        """
        assert self.node is not None
        try:
            if 4 in self.node.tpdo:
                if int(self.cfg.position_tpdo_index) != 4:
                    self.node.tpdo[4].enabled = False
                    self.node.tpdo.save()
                    logger.info("Disabled TPDO4 (velocity/extra feedback) to reduce CAN traffic")
        except Exception as exc:
            logger.warning("Failed to disable TPDO4: %s", exc)

    def _configure_tpdo_frequency(self) -> None:
        """
        Configure TPDO event timer to achieve the desired feedback frequency.
        The event timer controls how often TPDOs are transmitted.
        """
        assert self.node is not None
        try:
            # Calculate event timer in ms based on desired frequency
            # TPDO event timer resolution is 1ms
            event_timer_ms = int(round(1000.0 / self.cfg.feedback_frequency_hz))
            
            # Configure the position TPDO
            tpdo = self.node.tpdo[self.cfg.position_tpdo_index]
            tpdo.event_timer = event_timer_ms
            # Set inhibit time to prevent excessive TPDO transmission
            tpdo.inhibit_time = int(round(event_timer_ms * 10))  # 100us units
            
            # Set transmission type to 255 (event-driven with timer)
            tpdo.trans_type = 255
            tpdo.enabled = True
            tpdo.save()
            
            logger.info("TPDO%d configured for %d Hz feedback (event_timer=%d ms)",
                        self.cfg.position_tpdo_index, self.cfg.feedback_frequency_hz, event_timer_ms)

            # TPDO1（状态字）与位置 PDO 分离时，同步事件周期便于 topic 对齐
            if (
                self.tpdo_status is not None
                and self.tpdo_status is not self.tpdo_position
            ):
                stp = self.tpdo_status
                stp.event_timer = event_timer_ms
                stp.inhibit_time = int(round(event_timer_ms * 10))
                stp.trans_type = 255
                stp.enabled = True
                stp.save()
                logger.info(
                    "TPDO%d (status) event_timer=%d ms @ ~%d Hz",
                    int(self.cfg.status_tpdo_index),
                    event_timer_ms,
                    self.cfg.feedback_frequency_hz,
                )
        except Exception as exc:
            logger.warning("Failed to configure TPDO frequency: %s", exc)

    def _setup_state_machine(self) -> None:
        assert self.node is not None
        logger.info("[3] Setup 402 state machine")
        self.node.setup_402_state_machine()

    def _enter_operational(self) -> None:
        assert self.node is not None
        logger.info("[4] Enter OPERATIONAL")
        self.node.nmt.state = "OPERATIONAL"

    def _register_pdo_callbacks(self) -> None:
        logger.info("[5] Register PDO callbacks")

        def on_position_tpdo(_):
            try:
                position = int(self.tpdo_position[0x6064].raw)
                self.state.update_position(position)

                # Notify external callback (e.g., ROS2 publisher)
                if self.position_callback:
                    self.position_callback(position)

            except Exception as exc:
                logger.warning("Parse position TPDO2 failed: %s", exc)

        self.tpdo_position.add_callback(on_position_tpdo)

        if self.tpdo_status is not None:

            def on_status_tpdo(_):
                try:
                    sw = int(self.tpdo_status[0x6041].raw)
                    self.state.update_status_word(sw)
                except Exception as exc:
                    logger.warning("Parse status TPDO (0x6041) failed: %s", exc)

            self.tpdo_status.add_callback(on_status_tpdo)
            logger.info(
                "Registered statusword callback on TPDO%d",
                int(self.cfg.status_tpdo_index),
            )

    def _enable_drive(self) -> None:
        assert self.node is not None
        logger.info("[6] Enable drive")

        self.node.state = "READY TO SWITCH ON"
        self.wait_for_state("READY TO SWITCH ON")

        self.node.state = "SWITCHED ON"
        self.wait_for_state("SWITCHED ON")

        self.node.state = "OPERATION ENABLED"
        self.wait_for_state("OPERATION ENABLED")

        logger.info("DS402 state: %s", self.node.state)

    def _set_operation_mode_profiled_position(self) -> None:
        assert self.node is not None
        logger.info("[7] Set operation mode = PROFILED POSITION")
        self.node.op_mode = "PROFILED POSITION"
        logger.info("Operation mode: %s", self.node.op_mode)

    def _configure_pp_parameters(self) -> None:
        assert self.node is not None
        logger.info("[8] Configure PP parameters")

        try:
            if 0x6081 in self.node.sdo.keys():
                self.node.sdo[0x6081].raw = self.cfg.profile_velocity
                logger.info("0x6081 Profile velocity = %d", self.cfg.profile_velocity)

            if 0x6083 in self.node.sdo.keys():
                self.node.sdo[0x6083].raw = self.cfg.profile_acceleration
                logger.info("0x6083 Profile acceleration = %d", self.cfg.profile_acceleration)

            if 0x6084 in self.node.sdo.keys():
                self.node.sdo[0x6084].raw = self.cfg.profile_deceleration
                logger.info("0x6084 Profile deceleration = %d", self.cfg.profile_deceleration)

        except Exception as exc:
            logger.warning("Configure PP params warning: %s", exc)

    # -------------------------------------------------------------------------
    # Motion parameter management
    # -------------------------------------------------------------------------
    def get_motion_params(self) -> dict:
        """Read current motion parameters from drive"""
        assert self.node is not None

        params = {}
        try:
            if 0x6081 in self.node.sdo.keys():
                params['profile_velocity'] = int(self.node.sdo[0x6081].raw)
            if 0x6083 in self.node.sdo.keys():
                params['profile_acceleration'] = int(self.node.sdo[0x6083].raw)
            if 0x6084 in self.node.sdo.keys():
                params['profile_deceleration'] = int(self.node.sdo[0x6084].raw)
        except Exception as exc:
            logger.error("Failed to read motion params: %s", exc)

        return params

    def set_motion_params(self, velocity: Optional[int] = None,
                         acceleration: Optional[int] = None,
                         deceleration: Optional[int] = None) -> dict:
        """Set motion parameters (None = keep current value)"""
        assert self.node is not None

        try:
            if velocity is not None and velocity > 0 and 0x6081 in self.node.sdo.keys():
                self.node.sdo[0x6081].raw = velocity
                self.cfg.profile_velocity = velocity
                logger.info("Set profile velocity = %d", velocity)

            if acceleration is not None and acceleration > 0 and 0x6083 in self.node.sdo.keys():
                self.node.sdo[0x6083].raw = acceleration
                self.cfg.profile_acceleration = acceleration
                logger.info("Set profile acceleration = %d", acceleration)

            if deceleration is not None and deceleration > 0 and 0x6084 in self.node.sdo.keys():
                self.node.sdo[0x6084].raw = deceleration
                self.cfg.profile_deceleration = deceleration
                logger.info("Set profile deceleration = %d", deceleration)

        except Exception as exc:
            logger.error("Failed to set motion params: %s", exc)

        # Read back to confirm
        return self.get_motion_params()

    # -------------------------------------------------------------------------
    # State query API
    # -------------------------------------------------------------------------
    def get_current_position(self) -> int:
        """Get current position from PDO or SDO fallback"""
        assert self.node is not None
        snap = self.state.snapshot()

        if snap["position"] is not None:
            return int(snap["position"])

        try:
            return int(self.node.sdo[0x6064].raw)
        except Exception:
            return 0

    def get_current_height_mm(self) -> float:
        """Get current height in millimeters"""
        return self.pulse_to_height_mm(self.get_current_position())

    def get_status_snapshot(self) -> dict:
        """Get complete status snapshot (statusword: PDO cache or SDO fallback)."""
        snap = self.state.snapshot()
        pos = snap["position"]
        snap["height_mm"] = None if pos is None else self.pulse_to_height_mm(pos)

        sw = snap.get("status_word")
        snap["status_word_sdo"] = False
        if sw is None and self.node is not None:
            try:
                sw = int(self.node.sdo[0x6041].raw) & 0xFFFF
                snap["status_word"] = sw
                snap["status_word_valid"] = True
                snap["status_word_sdo"] = True
            except Exception:
                snap["status_word"] = 0
                snap["status_word_valid"] = False
        else:
            if sw is not None:
                snap["status_word"] = int(sw) & 0xFFFF
                snap["status_word_valid"] = True
            else:
                snap["status_word"] = 0
                snap["status_word_valid"] = False
        return snap

    # -------------------------------------------------------------------------
    # Motion control API
    # -------------------------------------------------------------------------
    def trigger_pp_setpoint(self, target_pos: int) -> None:
        """Trigger PP new set-point with control word sequence"""
        self.rpdo_cmd[self.cfg.control_word_name].raw = 0x002F
        self.rpdo_cmd[self.cfg.target_position_name].raw = int(target_pos)
        self.rpdo_cmd.transmit()

        time.sleep(0.02)

        self.rpdo_cmd[self.cfg.control_word_name].raw = 0x003F
        self.rpdo_cmd[self.cfg.target_position_name].raw = int(target_pos)
        self.rpdo_cmd.transmit()

        time.sleep(0.02)

        self.rpdo_cmd[self.cfg.control_word_name].raw = 0x002F
        self.rpdo_cmd[self.cfg.target_position_name].raw = int(target_pos)
        self.rpdo_cmd.transmit()

    def check_position_reached(self, target_pos: int) -> tuple[bool, int, bool]:
        """
        Check if target position is reached.

        Returns:
            (in_window, position_error, has_position_data)
        """
        snap = self.state.snapshot()
        pos = snap["position"]

        if pos is None:
            return False, 0, False

        pos_error = target_pos - pos
        in_window = abs(pos_error) <= self.cfg.position_tolerance_pulse

        return in_window, pos_error, True

    def move_to_height_mm_async(self, target_height_mm: float) -> tuple[bool, str, int]:
        """
        Start move to target height (non-blocking).

        Returns:
            (success, message, target_position_pulse)
        """
        current_pos = self.get_current_position()
        current_height_mm = self.pulse_to_height_mm(current_pos)

        if not self.is_valid_height(target_height_mm):
            msg = (
                f"Invalid target height: {target_height_mm:.2f} mm, "
                f"allowed range: {self.cfg.min_height_mm:.2f} ~ {self.cfg.max_height_mm:.2f} mm"
            )
            logger.error(msg)
            return False, msg, 0

        target_pos = self.height_mm_to_pulse(target_height_mm)

        if target_pos == current_pos:
            msg = "Already at target position"
            logger.info(msg)
            return True, msg, target_pos

        delta_pulse = target_pos - current_pos
        delta_height = target_height_mm - current_height_mm

        logger.info("Move command accepted:")
        logger.info("  Current height : %.2f mm", current_height_mm)
        logger.info("  Target height  : %.2f mm", target_height_mm)
        logger.info("  Height delta   : %.2f mm", delta_height)
        logger.info("  Current pos    : %d", current_pos)
        logger.info("  Target pos     : %d", target_pos)
        logger.info("  Pulse delta    : %d", delta_pulse)

        self.trigger_pp_setpoint(target_pos)
        logger.info("PP new set-point triggered")

        return True, "Motion started", target_pos

    # -------------------------------------------------------------------------
    # Shutdown
    # -------------------------------------------------------------------------
    def shutdown(self) -> None:
        """Clean shutdown of CANopen network"""
        logger.info("Shutting down controller")
        self.state.stop()

        if self.network is not None:
            try:
                for nid in self.network:
                    try:
                        self.network[nid].nmt.state = "PRE-OPERATIONAL"
                        logger.info("Node %d -> PRE-OPERATIONAL", nid)
                    except Exception:
                        pass

                self.network.disconnect()
                logger.info("Network disconnected")
            except Exception as exc:
                logger.error("Cleanup error: %s", exc)

        logger.info("Shutdown complete")
