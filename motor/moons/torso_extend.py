import os
import sys
import time
import threading
import traceback
import logging
from dataclasses import dataclass
from typing import Optional

import canopen

from _paths import EDS_SINGLE_AXIS


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logging.getLogger("can.interfaces.socketcan").setLevel(logging.INFO)
logger = logging.getLogger("lift_column_pp")


@dataclass
class LiftConfig:
    eds_file: str = EDS_SINGLE_AXIS
    channel: str = "can0"
    bitrate: int = 100000
    node_id: int = 4

    # 只保留 TPDO2：位置
    position_tpdo_index: int = 2
    cmd_rpdo_index: int = 2

    control_word_name: str = "Control word"
    target_position_name: str = "Target position"

    status_print_period: float = 0.1
    pp_wait_timeout: float = 120.0

    # 纯位置到位判定参数
    position_tolerance_pulse: int = 2000
    stable_time_sec: float = 0.3

    # Height model
    zero_encoder_height_mm: float = 480.0   # encoder=0 -> 480 mm
    min_height_mm: float = 230.0
    max_height_mm: float = 700.0

    # pulse +1,000,000 -> height +34.25 mm
    mm_per_1m_pulse: float = 34.25

    # PP motion params
    profile_velocity: int = 100000
    profile_acceleration: int = 500000
    profile_deceleration: int = 500000

    @property
    def pulse_per_mm(self) -> float:
        return 1000000.0 / self.mm_per_1m_pulse


class SharedState:
    def __init__(self):
        self._lock = threading.Lock()
        self.position: Optional[int] = None

        self.recv_position_count = 0
        self.last_position_ts: Optional[float] = None

        self.running = True

    def update_position(self, position: int) -> None:
        with self._lock:
            self.position = int(position)
            self.recv_position_count += 1
            self.last_position_ts = time.time()

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "position": self.position,
                "recv_position_count": self.recv_position_count,
                "last_position_ts": self.last_position_ts,
                "running": self.running,
            }

    def stop(self) -> None:
        with self._lock:
            self.running = False


class LiftColumnController:
    def __init__(self, config: LiftConfig):
        self.cfg = config
        self.network: Optional[canopen.Network] = None
        self.node: Optional[canopen.BaseNode402] = None

        self.tpdo_position = None
        self.rpdo_cmd = None

        self.state = SharedState()
        self.status_thread: Optional[threading.Thread] = None

    # -------------------------------------------------------------------------
    # Conversion
    # -------------------------------------------------------------------------
    def pulse_to_height_mm(self, position_pulse: int) -> float:
        return self.cfg.zero_encoder_height_mm + (float(position_pulse) / self.cfg.pulse_per_mm)

    def height_mm_to_pulse(self, height_mm: float) -> int:
        return int(round((float(height_mm) - self.cfg.zero_encoder_height_mm) * self.cfg.pulse_per_mm))

    def is_valid_height(self, height_mm: float) -> bool:
        return self.cfg.min_height_mm <= float(height_mm) <= self.cfg.max_height_mm

    # -------------------------------------------------------------------------
    # DS402 helpers
    # -------------------------------------------------------------------------
    def wait_for_state(self, target_state: str, timeout: float = 10.0) -> None:
        assert self.node is not None
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.node.state == target_state:
                return
            time.sleep(0.01)
        raise TimeoutError(f"Timeout waiting for state={target_state}, current={self.node.state}")

    # -------------------------------------------------------------------------
    # CANopen / PDO initialization
    # -------------------------------------------------------------------------
    def initialize(self) -> None:
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

        self._load_configuration()
        self._read_pdo_configuration()
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

        logger.info(
            "TPDO position index=%d COB-ID=0x%08X",
            self.cfg.position_tpdo_index, self.tpdo_position.cob_id
        )
        logger.info(
            "RPDO cmd index=%d COB-ID=0x%08X",
            self.cfg.cmd_rpdo_index, self.rpdo_cmd.cob_id
        )

        logger.info("TPDO2 mapped objects:")
        for obj in self.tpdo_position.map:
            logger.info("  - 0x%04X:%02X %s", obj.index, obj.subindex, obj.name)

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
            except Exception as exc:
                logger.warning("Parse position TPDO2 failed: %s", exc)

        self.tpdo_position.add_callback(on_position_tpdo)

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
    # State / status API
    # -------------------------------------------------------------------------
    def get_current_position(self) -> int:
        assert self.node is not None
        snap = self.state.snapshot()

        if snap["position"] is not None:
            return int(snap["position"])

        try:
            return int(self.node.sdo[0x6064].raw)
        except Exception:
            return 0

    def get_current_height_mm(self) -> float:
        return self.pulse_to_height_mm(self.get_current_position())

    def get_status_snapshot(self) -> dict:
        snap = self.state.snapshot()
        pos = snap["position"]
        snap["height_mm"] = None if pos is None else self.pulse_to_height_mm(pos)
        return snap

    # -------------------------------------------------------------------------
    # Motion control API
    # -------------------------------------------------------------------------
    def trigger_pp_setpoint(self, target_pos: int) -> None:
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

    def wait_pp_done_by_position(self, target_pos: int, timeout: Optional[float] = None) -> bool:
        timeout = timeout if timeout is not None else self.cfg.pp_wait_timeout

        t0 = time.time()
        stable_start = None
        last_log_t = 0.0

        while time.time() - t0 < timeout:
            snap = self.state.snapshot()
            pos = snap["position"]

            if pos is not None:
                pos_error = target_pos - pos
                in_window = abs(pos_error) <= self.cfg.position_tolerance_pulse

                now = time.time()
                if now - last_log_t > 0.5:
                    height_str = f"{self.pulse_to_height_mm(pos):.2f}"
                    print(
                        f"[WAIT] target_pos={target_pos}, "
                        f"position={pos}, "
                        f"pos_error={pos_error}, "
                        f"in_window={in_window}, "
                        f"height_mm={height_str}"
                    )
                    last_log_t = now

                if in_window:
                    if stable_start is None:
                        stable_start = now
                    elif now - stable_start >= self.cfg.stable_time_sec:
                        return True
                else:
                    stable_start = None

            time.sleep(0.02)

        return False

    def move_to_height_mm(self, target_height_mm: float, wait: bool = True) -> bool:
        current_pos = self.get_current_position()
        current_height_mm = self.pulse_to_height_mm(current_pos)

        if not self.is_valid_height(target_height_mm):
            print(
                f"[MOVE] Invalid target height: {target_height_mm:.2f} mm, "
                f"allowed range: {self.cfg.min_height_mm:.2f} ~ {self.cfg.max_height_mm:.2f} mm"
            )
            return False

        target_pos = self.height_mm_to_pulse(target_height_mm)
        delta_pulse = target_pos - current_pos
        delta_height = target_height_mm - current_height_mm

        print("[MOVE] Command accepted")
        print(f"       Current height : {current_height_mm:.2f} mm")
        print(f"       Target height  : {target_height_mm:.2f} mm")
        print(f"       Height delta   : {delta_height:.2f} mm")
        print(f"       Current pos    : {current_pos}")
        print(f"       Target pos     : {target_pos}")
        print(f"       Pulse delta    : {delta_pulse}")

        if target_pos == current_pos:
            print("[MOVE] Already at target, no motion needed")
            return True

        self.trigger_pp_setpoint(target_pos)
        print("[MOVE] PP new set-point triggered")

        if not wait:
            return True

        done = self.wait_pp_done_by_position(target_pos=target_pos)
        snap = self.get_status_snapshot()

        if done:
            print(
                f"[MOVE] Done: "
                f"position={snap['position']}, "
                f"height_mm={snap['height_mm']:.2f}"
            )
        else:
            height_str = "None" if snap["height_mm"] is None else f"{snap['height_mm']:.2f}"
            print(
                f"[MOVE] Timeout: "
                f"position={snap['position']}, "
                f"height_mm={height_str}"
            )

        return done

    # -------------------------------------------------------------------------
    # Status publishing
    # -------------------------------------------------------------------------
    def start_status_publisher(self) -> None:
        if self.status_thread is not None and self.status_thread.is_alive():
            return

        self.status_thread = threading.Thread(
            target=self._status_publisher_loop,
            daemon=True
        )
        self.status_thread.start()

    def _status_publisher_loop(self) -> None:
        while True:
            snap = self.get_status_snapshot()
            if not snap["running"]:
                break

            position = snap["position"]
            height_mm = snap["height_mm"]

            position_str = "None" if position is None else str(position)
            height_str = "None" if height_mm is None else f"{height_mm:.2f}"

            print(
                f"[STATUS] "
                f"position={position_str}, "
                f"height_mm={height_str}, "
                f"rx_pos={snap['recv_position_count']}"
            )

            time.sleep(self.cfg.status_print_period)

    # -------------------------------------------------------------------------
    # Shutdown
    # -------------------------------------------------------------------------
    def shutdown(self) -> None:
        print("\n[CLEANUP] Shutting down...")
        self.state.stop()

        if self.status_thread is not None:
            try:
                self.status_thread.join(timeout=1.0)
            except Exception:
                pass

        if self.network is not None:
            try:
                for nid in self.network:
                    try:
                        self.network[nid].nmt.state = "PRE-OPERATIONAL"
                        print(f"  ✓ Node {nid} -> PRE-OPERATIONAL")
                    except Exception:
                        pass

                self.network.disconnect()
                print("  ✓ Network disconnected")
            except Exception as exc:
                print(f"  ! Cleanup error: {exc}")

        print("Goodbye!")


def main():
    controller = LiftColumnController(LiftConfig())

    try:
        print("=" * 60)
        print("CANopen DS402 PP Lift Column Height Control Demo")
        print("Use TPDO2 position only")
        print("=" * 60)

        controller.initialize()
        controller.start_status_publisher()

        current_pos = controller.get_current_position()
        current_height = controller.get_current_height_mm()

        print("\n[INFO] Current state")
        print(f"    Current position   : {current_pos}")
        print(f"    Current height(mm): {current_height:.2f}")
        print(
            f"    Valid range(mm)   : "
            f"{controller.cfg.min_height_mm:.2f} ~ {controller.cfg.max_height_mm:.2f}"
        )

        user_text = input("\nEnter target height (mm): ").strip()

        try:
            target_height_mm = float(user_text)
        except ValueError:
            print(f"Invalid input: '{user_text}' is not a number")
            return

        controller.move_to_height_mm(target_height_mm, wait=True)

        print("\nKeep publishing status, press Ctrl+C to exit")
        while True:
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    except Exception as exc:
        exc_type, _, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        print(f"\n! ERROR: {exc_type.__name__} in {fname}:{exc_tb.tb_lineno}")
        print(f"  Message: {exc}")
        traceback.print_exc()

    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()
