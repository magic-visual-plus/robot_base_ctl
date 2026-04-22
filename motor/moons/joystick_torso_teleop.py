#!/usr/bin/env python3
"""
CANopen lift column joystick PV control
- BTN_TR + HAT_UP   -> lift up
- BTN_TR + HAT_DOWN -> lift down
- release -> stop

PV control mode (DS402):
  0x6060 = 3 (PV / Profile Velocity)

Communication:
  - Use PDO for realtime feedback + command:
    - TPDO: statusword + position/velocity feedback
    - RPDO: controlword + target velocity setpoint

Halt semantics (bit8 in 0x6040 Control word):
  - halt=0: run/continue
  - halt=1: decel stop
"""

import os
import sys
import errno
import time
import threading
import traceback
import logging
from dataclasses import dataclass
from typing import Optional

import canopen
from evdev import InputDevice, ecodes, list_devices

from _paths import EDS_SINGLE_AXIS


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logging.getLogger("can.interfaces.socketcan").setLevel(logging.INFO)
logger = logging.getLogger("lift_column_pv_joystick")


@dataclass
class LiftConfig:
    eds_file: str = EDS_SINGLE_AXIS
    channel: str = "can0"
    bitrate: int = 1000000
    node_id: int = 4

    status_print_period: float = 0.2
    control_period_sec: float = 0.05   # 50 ms
    # If TPDO is event-driven, reduce this to get more realtime updates.
    tpdo_event_timer_ms: int = 10
    # Reduce bus load: only keep the TPDO which contains position+velocity enabled.
    # (For your "COB 480" case this is typically TPDO4.)
    enable_only_feedback_tpdo: bool = True
    # Prefer position-only feedback from TPDO2 (COB 280 + node_id).
    # If TPDO2 doesn't contain 0x6064, we'll fall back to the first TPDO that does.
    preferred_position_tpdo_no: int = 2

    # Height model
    zero_encoder_height_mm: float = 630.0
    min_height_mm: float = 380.0
    max_height_mm: float = 880.0

    # pulse +1,000,000 -> height +34.25 mm
    mm_per_1m_pulse: float = 34.25

    # PV target: 1 mm/s
    lift_speed_mm_s: float = 5

    # PV accel / decel
    profile_acceleration: int = 800000   # Pulses/s^2
    profile_deceleration: int = 800000   # Pulses/s^2

    # safety margin
    height_limit_margin_mm: float = 2.0

    @property
    def pulse_per_mm(self) -> float:
        return 1000000.0 / self.mm_per_1m_pulse

    @property
    def target_velocity_pulses_s(self) -> int:
        return int(round(self.lift_speed_mm_s * self.pulse_per_mm))


class SharedState:
    def __init__(self):
        self._lock = threading.Lock()

        # CAN feedback
        self.position: Optional[int] = None
        self.actual_velocity: Optional[int] = None
        self.statusword: Optional[int] = None
        self.op_mode_display: Optional[int] = None

        # joystick state
        self.btn_tr_pressed = False
        self.hat0y = 0

        # control state
        self.command = "STOP"         # STOP / UP / DOWN
        self.last_sent_command = None
        self.running = True

        self.recv_position_count = 0
        self.last_position_ts: Optional[float] = None

    def update_feedback(
        self,
        position: Optional[int] = None,
        actual_velocity: Optional[int] = None,
        statusword: Optional[int] = None,
        op_mode_display: Optional[int] = None,
    ) -> None:
        with self._lock:
            if position is not None:
                self.position = int(position)
                self.recv_position_count += 1
                self.last_position_ts = time.time()
            if actual_velocity is not None:
                self.actual_velocity = int(actual_velocity)
            if statusword is not None:
                self.statusword = int(statusword)
            if op_mode_display is not None:
                self.op_mode_display = int(op_mode_display)

    def update_button_tr(self, pressed: bool) -> None:
        with self._lock:
            self.btn_tr_pressed = pressed

    def update_hat0y(self, value: int) -> None:
        with self._lock:
            self.hat0y = int(value)

    def set_command(self, command: str) -> None:
        with self._lock:
            self.command = command

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "position": self.position,
                "actual_velocity": self.actual_velocity,
                "statusword": self.statusword,
                "op_mode_display": self.op_mode_display,
                "btn_tr_pressed": self.btn_tr_pressed,
                "hat0y": self.hat0y,
                "command": self.command,
                "last_sent_command": self.last_sent_command,
                "recv_position_count": self.recv_position_count,
                "last_position_ts": self.last_position_ts,
                "running": self.running,
            }

    def set_last_sent_command(self, command: str) -> None:
        with self._lock:
            self.last_sent_command = command

    def stop(self) -> None:
        with self._lock:
            self.running = False


def find_joystick():
    devices = [InputDevice(path) for path in list_devices()]
    for dev in devices:
        caps = dev.capabilities()
        if ecodes.EV_ABS in caps and "mouse" not in dev.name.lower() and "keyboard" not in dev.name.lower():
            print(f"Found joystick-like device: {dev.name} at {dev.path}")
            return dev
    raise RuntimeError("Joystick not found! Check /dev/input/event* permission.")


class LiftColumnPVController:
    def __init__(self, config: LiftConfig):
        self.cfg = config
        self.network: Optional[canopen.Network] = None
        self.node: Optional[canopen.BaseNode402] = None

        self.state = SharedState()

        self.joystick = find_joystick()

        self.status_thread: Optional[threading.Thread] = None
        self.control_thread: Optional[threading.Thread] = None
        self.joy_thread: Optional[threading.Thread] = None

        # PDO objects (detected after tpdo/rpdo.read()).
        self._tpdo_status_no: Optional[int] = None
        self._tpdo_pos_no: Optional[int] = None
        self._tpdo_vel_no: Optional[int] = None
        self._rpdo_cmd_no: Optional[int] = None

        self._tpdo_status = None
        self._tpdo_pos = None
        self._tpdo_vel = None
        self._rpdo_cmd = None

        # OD parameter names from the EDS object dictionary.
        self._OD_CONTROLWORD = "Control word"
        self._OD_TARGET_VEL = "Target velocity"
        self._OD_STATUSWORD = "Status word"
        self._OD_OP_MODE_DISPLAY = "Modes of operation display"
        self._OD_POSITION = "Position value calculated"
        self._OD_VELOCITY = "Velocity value calculated"

        # Whether current feedback TPDO also contains 0x606C (velocity).
        self._have_velocity_pdo: bool = False

    def _safe_transmit(self, pdo, retries: int = 60, backoff_s: float = 0.003) -> None:
        last_exc: Optional[BaseException] = None
        for _ in range(retries):
            try:
                pdo.transmit()
                return
            except Exception as exc:
                last_exc = exc
                # socketcan TX buffer might run out under load (Errno 105).
                if "No buffer space available" in str(exc):
                    time.sleep(backoff_s)
                    continue
                if isinstance(exc, OSError) and exc.errno == errno.ENOBUFS:
                    time.sleep(backoff_s)
                    continue
                raise
        if last_exc is not None:
            raise last_exc

    # -------------------------------------------------------------------------
    # Conversion
    # -------------------------------------------------------------------------
    def pulse_to_height_mm(self, position_pulse: int) -> float:
        return self.cfg.zero_encoder_height_mm + (float(position_pulse) / self.cfg.pulse_per_mm)

    def get_current_position(self) -> int:
        snap = self.state.snapshot()
        if snap["position"] is not None:
            return int(snap["position"])

        assert self.node is not None
        try:
            return int(self.node.sdo[0x6064].raw)
        except Exception:
            return 0

    def get_current_height_mm(self) -> float:
        return self.pulse_to_height_mm(self.get_current_position())

    # -------------------------------------------------------------------------
    # PDO configuration (realtime comms)
    # -------------------------------------------------------------------------
    def _setup_pdo_communication(self) -> None:
        assert self.node is not None

        # Load configuration so the PDO mapping objects are available.
        try:
            self.node.load_configuration()
        except Exception:
            # Some canopen setups may not require it; keep best-effort.
            pass

        self.node.tpdo.read()
        self.node.rpdo.read()

        # 1) RPDO command mapping: find where Control word + Target velocity are mapped.
        rpdo_cmd_no: Optional[int] = None
        for rpdo_no in range(1, 5):
            try:
                rpdo = self.node.rpdo[rpdo_no]
                obj_indices = {obj.index for obj in rpdo.map}
            except Exception:
                continue
            if 0x6040 in obj_indices and 0x60FF in obj_indices:
                rpdo_cmd_no = rpdo_no
                break
        if rpdo_cmd_no is None:
            raise RuntimeError("Failed to detect RPDO(cmd) mapping (0x6040+0x60FF).")

        # 2) Feedback TPDO mapping: only keep TPDO carrying position (0x6064).
        # Prefer TPDO2 (COB 280 + node_id) if it contains 0x6064.
        tpdo_pos_no: Optional[int] = None
        preferred = int(self.cfg.preferred_position_tpdo_no)

        # Prefer preferred TPDO first.
        try:
            tpdo_pref = self.node.tpdo[preferred]
            obj_indices_pref = {obj.index for obj in tpdo_pref.map}
            if 0x6064 in obj_indices_pref:
                tpdo_pos_no = preferred
                self._have_velocity_pdo = (0x606C in obj_indices_pref)
        except Exception:
            pass

        # Fallback: first TPDO with 0x6064.
        if tpdo_pos_no is None:
            self._have_velocity_pdo = False
            for tpdo_no in range(1, 5):
                try:
                    tpdo = self.node.tpdo[tpdo_no]
                    obj_indices = {obj.index for obj in tpdo.map}
                except Exception:
                    continue
                if 0x6064 in obj_indices:
                    tpdo_pos_no = tpdo_no
                    self._have_velocity_pdo = (0x606C in obj_indices)
                    break

        if tpdo_pos_no is None:
            raise RuntimeError("Failed to detect any feedback TPDO containing 0x6064 (position).")

        self._tpdo_status = None
        self._tpdo_status_no = None
        self._tpdo_pos_no = tpdo_pos_no
        self._tpdo_vel_no = tpdo_pos_no if self._have_velocity_pdo else None
        self._rpdo_cmd_no = rpdo_cmd_no

        # 3) Stop PDO traffic first: disable all TPDOs, then enable only position TPDO.
        for tpdo_no in range(1, 5):
            try:
                p = self.node.tpdo[tpdo_no]
                if self.cfg.enable_only_feedback_tpdo:
                    p.enabled = (tpdo_no == tpdo_pos_no)
                    # If disabling doesn't immediately stop, force event timer to 0.
                    p.event_timer = int(self.cfg.tpdo_event_timer_ms) if tpdo_no == tpdo_pos_no else 0
                    p.trans_type = 255  # event-driven is typically "255" in this codebase
                    p.inhibit_time = 0
                    p.save()
                else:
                    # Keep original enable state.
                    pass
            except Exception:
                pass

        self._tpdo_pos = self.node.tpdo[tpdo_pos_no]
        self._tpdo_vel = self.node.tpdo[tpdo_pos_no] if self._have_velocity_pdo else None
        self._rpdo_cmd = self.node.rpdo[rpdo_cmd_no]

        try:
            self._rpdo_cmd.enabled = True
        except Exception:
            pass

        cob_pos_base = 0x200 + (tpdo_pos_no - 1) * 0x80
        logger.info(
            "PDO configured: only TPDO%d(pos=0x6064) enabled (COB %d+node_id), RPDO(cmd)=%d enabled, velocity_pdo=%s",
            tpdo_pos_no,
            cob_pos_base,
            rpdo_cmd_no,
            self._have_velocity_pdo,
        )

        self._register_pdo_callbacks()

    def _detect_pdo_indices(self) -> None:
        assert self.node is not None

        tpdo_status_no: Optional[int] = None
        tpdo_pos_no: Optional[int] = None
        tpdo_vel_no: Optional[int] = None
        rpdo_cmd_no: Optional[int] = None

        # TPDO: scan for where key objects are mapped.
        for tpdo_no in range(1, 5):
            try:
                tpdo = self.node.tpdo[tpdo_no]
                obj_indices = {obj.index for obj in tpdo.map}
            except Exception:
                continue

            if tpdo_status_no is None and 0x6041 in obj_indices:
                tpdo_status_no = tpdo_no
            if tpdo_pos_no is None and 0x6064 in obj_indices:
                tpdo_pos_no = tpdo_no
            if tpdo_vel_no is None and 0x606C in obj_indices:
                tpdo_vel_no = tpdo_no

        # RPDO: scan for where controlword + target velocity are mapped.
        for rpdo_no in range(1, 5):
            try:
                rpdo = self.node.rpdo[rpdo_no]
                obj_indices = {obj.index for obj in rpdo.map}
            except Exception:
                continue

            if 0x6040 in obj_indices and 0x60FF in obj_indices:
                rpdo_cmd_no = rpdo_no
                break

        if tpdo_status_no is None or tpdo_pos_no is None or tpdo_vel_no is None or rpdo_cmd_no is None:
            raise RuntimeError(
                "Failed to detect required PDO mapping "
                f"(status TPDO={tpdo_status_no}, pos TPDO={tpdo_pos_no}, vel TPDO={tpdo_vel_no}, rpdo={rpdo_cmd_no})"
            )

        self._tpdo_status_no = tpdo_status_no
        self._tpdo_pos_no = tpdo_pos_no
        self._tpdo_vel_no = tpdo_vel_no
        self._rpdo_cmd_no = rpdo_cmd_no

    def _register_pdo_callbacks(self) -> None:
        assert self._tpdo_pos is not None

        def try_read_op_mode(tpdo) -> Optional[int]:
            try:
                return int(tpdo[self._OD_OP_MODE_DISPLAY].raw)
            except Exception:
                return None

        # Statusword callback (optional; we often disable status TPDO to save bus).
        if self._tpdo_status is not None:
            try:
                def on_status(_map):
                    try:
                        sw = int(self._tpdo_status[self._OD_STATUSWORD].raw)
                        op_mode = try_read_op_mode(self._tpdo_status)
                        self.state.update_feedback(statusword=sw, op_mode_display=op_mode)
                    except Exception:
                        pass

                self._tpdo_status.add_callback(on_status)
            except Exception:
                pass

        # Position callback (TPDO with 0x6064).
        try:
            def on_pos(_map):
                try:
                    pos = int(self._tpdo_pos[self._OD_POSITION].raw)
                except Exception:
                    pos = None
                op_mode = try_read_op_mode(self._tpdo_pos)
                self.state.update_feedback(position=pos, op_mode_display=op_mode)

            self._tpdo_pos.add_callback(on_pos)
        except Exception:
            pass

        # Velocity callback (only if TPDO also contains 0x606C).
        if self._have_velocity_pdo and self._tpdo_vel is not None:
            try:
                def on_vel(_map):
                    try:
                        vel = int(self._tpdo_vel[self._OD_VELOCITY].raw)
                    except Exception:
                        vel = None
                    self.state.update_feedback(actual_velocity=vel)

                self._tpdo_vel.add_callback(on_vel)
            except Exception:
                pass

    def _seed_state_via_sdo_once(self) -> None:
        assert self.node is not None
        try:
            pos = int(self.node.sdo[0x6064].raw)
        except Exception:
            pos = None
        try:
            actual_velocity = int(self.node.sdo[0x606C].raw)
        except Exception:
            actual_velocity = None
        try:
            statusword = int(self.node.sdo[0x6041].raw)
        except Exception:
            statusword = None
        try:
            op_mode_display = int(self.node.sdo[0x6061].raw)
        except Exception:
            op_mode_display = None

        self.state.update_feedback(
            position=pos,
            actual_velocity=actual_velocity,
            statusword=statusword,
            op_mode_display=op_mode_display,
        )

    # -------------------------------------------------------------------------
    # CANopen init
    # -------------------------------------------------------------------------
    def initialize(self) -> None:
        logger.info("Initializing lift column PV joystick controller")
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
        logger.info("Connected to CAN network")

        # Stop PDO traffic first (PRE-OPERATIONAL), then configure PDOs, then start OPERATIONAL.
        # NMT commands (CiA 301):
        #   0x80 = enter pre-operational
        #   0x01 = start remote node
        self.node.nmt.send_command(0x80)
        timeout = time.time() + 3.0
        while self.node.nmt.state != "PRE-OPERATIONAL" and time.time() < timeout:
            time.sleep(0.1)
        logger.info("NMT current state (pre-op): %s", self.node.nmt.state)

        # PDO realtime feedback + command (still PV control mode).
        self._setup_pdo_communication()

        self._enter_pv_mode()
        self._enable_operation()
        self._configure_pv_parameters()

        # Now allow PDO traffic.
        self.node.nmt.send_command(0x01)
        timeout = time.time() + 3.0
        while self.node.nmt.state != "OPERATIONAL" and time.time() < timeout:
            time.sleep(0.1)
        logger.info("NMT current state (op): %s", self.node.nmt.state)

        # Seed initial state once; afterwards TPDO callbacks keep it realtime.
        self._seed_state_via_sdo_once()

        logger.info(
            "PV target speed = %d pulses/s (%.3f mm/s)",
            self.cfg.target_velocity_pulses_s,
            self.cfg.lift_speed_mm_s
        )

    def _enter_pv_mode(self) -> None:
        assert self.node is not None

        self.node.sdo[0x6060].raw = 3
        time.sleep(0.1)
        mode_display = int(self.node.sdo[0x6061].raw)
        if mode_display != 3:
            raise RuntimeError(f"Failed to enter PV mode, 0x6061={mode_display}")
        logger.info("PV mode enabled (0x6060=3)")

    def _enable_operation(self) -> None:
        assert self.node is not None

        # DS402 enable sequence
        self.node.sdo[0x6040].raw = 0x0006
        time.sleep(0.05)
        self.node.sdo[0x6040].raw = 0x0007
        time.sleep(0.05)
        # PV example in manual uses 0x010F
        self.node.sdo[0x6040].raw = 0x010F
        time.sleep(0.05)

        sw = int(self.node.sdo[0x6041].raw)
        logger.info("Drive enabled, statusword=0x%04X", sw)

    def _configure_pv_parameters(self) -> None:
        assert self.node is not None

        # max speed >= target speed
        max_speed = max(self.cfg.target_velocity_pulses_s * 2, 100000)

        self.node.sdo[0x607F].raw = int(max_speed)
        self.node.sdo[0x6083].raw = int(self.cfg.profile_acceleration)
        self.node.sdo[0x6084].raw = int(self.cfg.profile_deceleration)

        # startup as stop
        self.node.sdo[0x60FF].raw = 0
        self.node.sdo[0x6040].raw = 0x010F  # halt=1, stopped
        logger.info(
            "PV params set: 0x607F=%d, 0x6083=%d, 0x6084=%d",
            max_speed,
            self.cfg.profile_acceleration,
            self.cfg.profile_deceleration,
        )

    # -------------------------------------------------------------------------
    # Low-level PV actions
    # -------------------------------------------------------------------------
    def pv_run(self, target_velocity: int) -> None:
        """
        PV run/continue:
          write 0x60FF
          clear halt bit8 => 0x000F
        """
        assert self._rpdo_cmd is not None

        # Atomic RPDO: controlword + target velocity.
        self._rpdo_cmd[self._OD_CONTROLWORD].raw = 0x000F  # halt=0, OP enabled
        self._rpdo_cmd[self._OD_TARGET_VEL].raw = int(target_velocity)
        self._safe_transmit(self._rpdo_cmd)

    def pv_stop(self) -> None:
        """
        Controlled stop:
          set target speed to 0
          set halt bit8 => 0x010F
        """
        assert self._rpdo_cmd is not None

        self._rpdo_cmd[self._OD_CONTROLWORD].raw = 0x010F  # halt=1 decel stop
        self._rpdo_cmd[self._OD_TARGET_VEL].raw = 0
        self._safe_transmit(self._rpdo_cmd)

    # -------------------------------------------------------------------------
    # Control decision
    # -------------------------------------------------------------------------
    def update_command_from_joystick_state(self) -> None:
        snap = self.state.snapshot()

        if snap["btn_tr_pressed"] and snap["hat0y"] == -1:
            cmd = "UP"
        elif snap["btn_tr_pressed"] and snap["hat0y"] == 1:
            cmd = "DOWN"
        else:
            cmd = "STOP"

        self.state.set_command(cmd)

    def apply_command(self) -> None:
        snap = self.state.snapshot()
        cmd = snap["command"]
        last_cmd = snap["last_sent_command"]

        current_height = self.get_current_height_mm()

        # soft limit protection
        if cmd == "UP" and current_height >= (self.cfg.max_height_mm - self.cfg.height_limit_margin_mm):
            cmd = "STOP"
        elif cmd == "DOWN" and current_height <= (self.cfg.min_height_mm + self.cfg.height_limit_margin_mm):
            cmd = "STOP"

        if cmd == last_cmd:
            return

        v = self.cfg.target_velocity_pulses_s

        if cmd == "UP":
            self.pv_run(+v)
            print(f">>> LIFT UP    target_velocity=+{v} pulses/s ({self.cfg.lift_speed_mm_s:.3f} mm/s)")
        elif cmd == "DOWN":
            self.pv_run(-v)
            print(f">>> LIFT DOWN  target_velocity=-{v} pulses/s ({self.cfg.lift_speed_mm_s:.3f} mm/s)")
        else:
            self.pv_stop()
            print(">>> LIFT STOP")

        self.state.set_last_sent_command(cmd)

    # -------------------------------------------------------------------------
    # Threads
    # -------------------------------------------------------------------------
    def start(self) -> None:
        self.status_thread = threading.Thread(target=self._status_loop, daemon=True)
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.joy_thread = threading.Thread(target=self._joystick_loop, daemon=True)

        self.status_thread.start()
        self.control_thread.start()
        self.joy_thread.start()

    def _status_loop(self) -> None:
        while True:
            snap = self.state.snapshot()
            if not snap["running"]:
                break

            pos = snap["position"]
            if pos is None:
                pos = self.get_current_position()
            height = self.pulse_to_height_mm(int(pos))

            # PDO callbacks keep feedback realtime; just print the latest snapshot.
            actual_velocity = snap["actual_velocity"] if self._have_velocity_pdo else None
            statusword = snap["statusword"]
            op_mode_display = snap["op_mode_display"]

            print(
                f"[STATUS] "
                f"height_mm={height:.2f}, "
                f"position={pos}, "
                f"actual_velocity={actual_velocity if actual_velocity is not None else '--'}, "
                f"cmd={snap['command']}, "
                f"mode={op_mode_display}, "
                f"statusword={None if statusword is None else f'0x{statusword:04X}'}"
            )
            time.sleep(self.cfg.status_print_period)

    def _control_loop(self) -> None:
        while True:
            snap = self.state.snapshot()
            if not snap["running"]:
                break

            try:
                self.apply_command()
            except Exception as exc:
                logger.warning("apply_command failed: %s", exc)

            time.sleep(self.cfg.control_period_sec)

    def _joystick_loop(self) -> None:
        print("Listening joystick...")
        print("Combo:")
        print("  BTN_TR + HAT_UP   -> lift up")
        print("  BTN_TR + HAT_DOWN -> lift down")
        print("  release           -> stop")

        for event in self.joystick.read_loop():
            snap = self.state.snapshot()
            if not snap["running"]:
                break

            try:
                if event.type == ecodes.EV_KEY:
                    code = event.code
                    val = event.value
                    code_name = ecodes.bytype[ecodes.EV_KEY].get(code, f"UNKNOWN_KEY_{code}")

                    if val == 1:
                        state = "PRESSED"
                    elif val == 0:
                        state = "RELEASED"
                    elif val == 2:
                        state = "HOLD"
                    else:
                        state = f"STATE_{val}"

                    print(f"[BUTTON] {code_name:<15} code={code:<4} state={state}")

                    if code == ecodes.BTN_TR:
                        if val == 1:
                            self.state.update_button_tr(True)
                        elif val == 0:
                            self.state.update_button_tr(False)

                        self.update_command_from_joystick_state()

                elif event.type == ecodes.EV_ABS:
                    code = event.code
                    val = event.value
                    code_name = ecodes.bytype[ecodes.EV_ABS].get(code, f"UNKNOWN_ABS_{code}")

                    # only print HAT0Y for cleaner output
                    if code == ecodes.ABS_HAT0Y:
                        print(f"[AXIS] {code_name:<10} raw={val}")
                        self.state.update_hat0y(val)
                        self.update_command_from_joystick_state()

            except Exception as exc:
                logger.warning("Joystick event handling failed: %s", exc)

    # -------------------------------------------------------------------------
    # Shutdown
    # -------------------------------------------------------------------------
    def shutdown(self) -> None:
        print("\n[CLEANUP] Shutting down...")
        self.state.stop()

        try:
            self.pv_stop()
        except Exception:
            pass

        # Make sure the drive stops emitting TPDOs after this process exits.
        self._disable_pdo_traffic()

        for th in [self.joy_thread, self.control_thread, self.status_thread]:
            if th is not None:
                try:
                    th.join(timeout=1.0)
                except Exception:
                    pass

        if self.network is not None:
            try:
                self.network.disconnect()
                print("  ✓ Network disconnected")
            except Exception as exc:
                print(f"  ! Cleanup error: {exc}")

        print("Goodbye!")

    def _disable_pdo_traffic(self) -> None:
        """
        Stop PDO traffic aggressively:
        - Set NMT to PRE-OPERATIONAL (CiA301), so drives should stop sending TPDO
        - Disable TPDO1..TPDO4 and set event_timer=0 (best-effort)
        - Disable the RPDO we use
        """
        if self.node is None:
            return

        # 1) NMT -> PRE-OPERATIONAL
        try:
            self.node.nmt.send_command(0x80)  # 0x80 = PRE-OPERATIONAL
        except Exception:
            try:
                self.node.nmt.state = "PRE-OPERATIONAL"
            except Exception:
                pass

        # 2) Disable TPDO
        for tpdo_no in range(1, 5):
            try:
                p = self.node.tpdo[tpdo_no]
                p.enabled = False
                try:
                    p.event_timer = 0
                except Exception:
                    pass
                try:
                    p.inhibit_time = 0
                except Exception:
                    pass
                try:
                    p.save()
                except Exception:
                    pass
            except Exception:
                pass

        # 3) Disable RPDO cmd we use (if any)
        for rpdo_no in range(1, 5):
            try:
                rp = self.node.rpdo[rpdo_no]
                rp.enabled = False
            except Exception:
                pass

        # Give the drive a brief moment to stop PDO emission.
        time.sleep(0.1)


def main():
    controller = LiftColumnPVController(LiftConfig())

    try:
        print("=" * 60)
        print("CANopen Lift Column PV Joystick Control")
        print("BTN_TR + HAT_UP   -> UP")
        print("BTN_TR + HAT_DOWN -> DOWN")
        print("Release           -> STOP")
        print("=" * 60)

        print(
            f"[CONFIG] 1 mm/s = {controller.cfg.target_velocity_pulses_s} pulses/s "
            f"(pulse_per_mm={controller.cfg.pulse_per_mm:.3f})"
        )

        controller.initialize()
        controller.start()

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
