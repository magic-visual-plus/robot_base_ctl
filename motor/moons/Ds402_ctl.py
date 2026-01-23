#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Multi-axis CANopen DS402 (PP / PV / CSV) clean abstraction + EmergencyStopQuiet mode
+ External API in radians (pos) and rad/s (vel), internal in encoder counts.

Key features:
- External interface:
    * PV/CSV: set_target_velocity_rad_s(rad/s)  -> internal counts/s
    * PP:     pp_move_to_rad(rad)               -> internal counts
    * Feedback provides both raw (counts) and converted (rad / rad/s)
- Position "zero" at init:
    * DO NOT write 0x6064 (RO)
    * Use vendor object 0x200C = 1 to clear actual position (0x6064 -> 0)
- safe_transmit(): handles socketcan ENOBUFS (Errno 105) with retry/backoff
- MotorManager.estop_quiet_all(): 3-stage stop + quiet PDO traffic (best effort)
"""

import time
import errno
import math
import logging
from dataclasses import dataclass
from typing import Dict, Optional, Protocol, List, Literal

import canopen

# ------------------------------
# Logging
# ------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
LOG = logging.getLogger("ds402")
logging.getLogger("can.interfaces.socketcan").setLevel(logging.INFO)

# ---- KEEP ORIGINAL OD STRINGS EXACTLY (DO NOT CHANGE) ----
VEL_VARIABLE = "Velocity value calculated"
STATUS_VARIABLE = "Status word"
CTL_VARIABLE = "Control word"
TARGET_VEL_VARIABLE = "Target velocity"
# ---------------------------------------------------------


# ------------------------------
# ENOBUFS retry helpers
# ------------------------------
def is_no_buffer_space(exc: Exception) -> bool:
    msg = str(exc)
    if "No buffer space available" in msg:
        return True
    if isinstance(exc, OSError) and exc.errno == errno.ENOBUFS:
        return True
    return False


def safe_transmit(pdo, retries: int = 80, backoff_s: float = 0.003):
    """Retry transmit when socketcan TX buffer is full."""
    last_exc = None
    for _ in range(retries):
        try:
            pdo.transmit()
            return
        except Exception as e:
            last_exc = e
            if is_no_buffer_space(e):
                time.sleep(backoff_s)
                continue
            raise
    raise last_exc


# ------------------- Helpers: SDO read -------------------
def safe_sdo_read(node: canopen.BaseNode402, idx: int, sub=None, default=None):
    try:
        if sub is None:
            return node.sdo[idx].raw
        return node.sdo[idx][sub].raw
    except Exception:
        return default


# ------------------------------
# Kinematics: counts <-> rad
# ------------------------------
INT32_MIN = -(2 ** 31)
INT32_MAX = (2 ** 31) - 1


def clamp_int32(x: int) -> int:
    return max(INT32_MIN, min(INT32_MAX, int(x)))


@dataclass(frozen=True)
class MotorKinematics:
    """
    counts_per_output_rev = encoder_cpr * gear_ratio
    Example: encoder_cpr=2^16, gear_ratio=10 -> 655360 counts / output-shaft-rev
    """
    encoder_cpr: int = 2 ** 16
    gear_ratio: float = 10.0

    @property
    def counts_per_output_rev(self) -> float:
        return float(self.encoder_cpr) * float(self.gear_ratio)


def rad_s_to_counts_s(rad_s: float, kin: MotorKinematics) -> int:
    counts_s = (rad_s / (2.0 * math.pi)) * kin.counts_per_output_rev
    return clamp_int32(int(round(counts_s)))


def counts_s_to_rad_s(counts_s: int, kin: MotorKinematics) -> float:
    return (float(counts_s) / kin.counts_per_output_rev) * (2.0 * math.pi)


def rad_to_counts_pos(rad: float, kin: MotorKinematics) -> int:
    counts = (rad / (2.0 * math.pi)) * kin.counts_per_output_rev
    return clamp_int32(int(round(counts)))


def counts_to_rad_pos(counts: int, kin: MotorKinematics) -> float:
    return (float(counts) / kin.counts_per_output_rev) * (2.0 * math.pi)


# ------------------------------
# Control modes (CiA402 0x6060)
# ------------------------------
ControlModeName = Literal["PP", "PV", "CSV"]


class ControlMode:
    PP = 1    # Profile Position Mode
    PV = 3    # Profile Velocity Mode
    CSV = 9   # Cyclic Synchronous Velocity Mode


MODE_MAP: Dict[str, int] = {
    "PP": ControlMode.PP,
    "PV": ControlMode.PV,
    "CSV": ControlMode.CSV,
}


# ------------------------------
# User-configurable PDO mapping
# ------------------------------
@dataclass(frozen=True)
class PdoMapConfig:
    # TPDO: statusword
    tpdo_status_num: int = 1
    statusword_name: str = STATUS_VARIABLE

    # TPDO: feedback (pos/vel)
    tpdo_fb_num: int = 4
    position_name: str = "Position value calculated"
    velocity_name: str = VEL_VARIABLE

    # RPDO for velocity command (PV/CSV): controlword + target velocity
    rpdo_vel_num: int = 3
    controlword_name: str = CTL_VARIABLE
    target_velocity_name: str = TARGET_VEL_VARIABLE

    # RPDO for position command (PP)
    rpdo_pos_num: int = 2
    target_position_name: str = "Target position"


PDO_MAP = PdoMapConfig()


# ------------------------------
# DS402 feedback + interfaces
# ------------------------------
@dataclass
class MotorFeedback:
    statusword: int
    position_counts: Optional[int]
    velocity_counts_s: Optional[int]
    ts: float
    position_rad: Optional[float] = None
    velocity_rad_s: Optional[float] = None


class Motor(Protocol):
    axis_id: int

    def init(self, default_mode: ControlModeName = "PV") -> None: ...
    def enable(self, timeout_s: float = 5.0) -> None: ...
    def disable(self) -> None: ...
    def fault_reset(self) -> None: ...
    def set_mode(self, mode: ControlModeName) -> None: ...

    # External API (rad / rad/s)
    def set_target_velocity_rad_s(self, rad_s: float) -> None: ...
    def pp_move_to_rad(self, pos_rad: float, relative: bool = False, immediate: bool = True,
                       wait_ack: bool = False, timeout_s: float = 2.0) -> None: ...
    def stop(self) -> None: ...
    def feedback(self) -> MotorFeedback: ...


class Ds402IO(Protocol):
    """Protocol-agnostic DS402 semantic IO."""
    def write_controlword(self, cw: int) -> None: ...
    def read_statusword(self) -> int: ...
    def write_mode_of_operation(self, mode: int) -> None: ...
    def write_target_velocity_counts_s(self, vel_counts_s: int, cw: int = 0x000F) -> None: ...
    def write_target_position_counts(self, pos_counts: int, cw: int = 0x000F) -> None: ...
    def read_position_counts(self) -> Optional[int]: ...
    def read_velocity_counts_s(self) -> Optional[int]: ...


# ------------------------------
# DS402 state machine helpers
# ------------------------------
class Ds402State:
    NOT_READY = "NOT_READY"
    SWITCH_ON_DISABLED = "SWITCH_ON_DISABLED"
    READY_TO_SWITCH_ON = "READY_TO_SWITCH_ON"
    SWITCHED_ON = "SWITCHED_ON"
    OPERATION_ENABLED = "OPERATION_ENABLED"
    QUICK_STOP_ACTIVE = "QUICK_STOP_ACTIVE"
    FAULT_REACTION_ACTIVE = "FAULT_REACTION_ACTIVE"
    FAULT = "FAULT"
    UNKNOWN = "UNKNOWN"


def decode_ds402_state(statusword: int) -> str:
    masked = statusword & 0x006F
    if masked == 0x0000:
        return Ds402State.NOT_READY
    if masked == 0x0040:
        return Ds402State.SWITCH_ON_DISABLED
    if masked == 0x0021:
        return Ds402State.READY_TO_SWITCH_ON
    if masked == 0x0023:
        return Ds402State.SWITCHED_ON
    if masked == 0x0027:
        return Ds402State.OPERATION_ENABLED
    if masked == 0x0007:
        return Ds402State.QUICK_STOP_ACTIVE
    if masked == 0x000F:
        return Ds402State.FAULT_REACTION_ACTIVE
    if masked == 0x0008:
        return Ds402State.FAULT
    return Ds402State.UNKNOWN


def sw_bit(statusword: int, bit: int) -> bool:
    return bool(statusword & (1 << bit))


class Ds402StateMachine:
    """Pure DS402 semantic state machine (depends only on Ds402IO)."""
    def __init__(self, io: Ds402IO):
        self.io = io

    def wait_state(self, target: str, timeout_s: float = 5.0, poll_s: float = 0.01) -> None:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            sw = self.io.read_statusword()
            st = decode_ds402_state(sw)
            if st == target:
                return
            time.sleep(poll_s)
        sw = self.io.read_statusword()
        raise TimeoutError(
            f"Timeout waiting state={target}, last SW=0x{sw:04X}, decoded={decode_ds402_state(sw)}"
        )

    def fault_reset(self) -> None:
        self.io.write_controlword(0x0080)
        time.sleep(0.05)
        self.io.write_controlword(0x0000)
        time.sleep(0.05)

    def enable_operation(self, timeout_s: float = 5.0) -> None:
        self.io.write_controlword(0x0006)
        self.wait_state(Ds402State.READY_TO_SWITCH_ON, timeout_s=timeout_s)

        self.io.write_controlword(0x0007)
        self.wait_state(Ds402State.SWITCHED_ON, timeout_s=timeout_s)

        self.io.write_controlword(0x000F)
        self.wait_state(Ds402State.OPERATION_ENABLED, timeout_s=timeout_s)


# ------------------------------
# CANopen shared bus + IO implementation
# ------------------------------
@dataclass
class _AxisCache:
    statusword: int = 0
    position_counts: Optional[int] = None
    velocity_counts_s: Optional[int] = None
    ts: float = 0.0


class CanopenBus:
    def __init__(self, channel: str = "can0", bitrate: int = 500000, interface: str = "socketcan"):
        self.channel = channel
        self.bitrate = bitrate
        self.interface = interface
        self.net = canopen.Network()
        self.nodes: Dict[int, canopen.BaseNode402] = {}
        self.cache: Dict[int, _AxisCache] = {}

    def connect(self, can_filters: Optional[List[Dict]] = None) -> None:
        try:
            if can_filters is not None:
                self.net.connect(
                    interface=self.interface, channel=self.channel, bitrate=self.bitrate,
                    can_filters=can_filters
                )
            else:
                self.net.connect(interface=self.interface, channel=self.channel, bitrate=self.bitrate)
        except TypeError:
            LOG.warning("connect() does not accept can_filters; connecting without filters.")
            self.net.connect(interface=self.interface, channel=self.channel, bitrate=self.bitrate)

        LOG.info("Connected: interface=%s channel=%s bitrate=%s", self.interface, self.channel, self.bitrate)

    def disconnect(self) -> None:
        try:
            try:
                self.net.sync.stop()
            except Exception:
                pass
            self.net.disconnect()
        except Exception:
            pass
        LOG.info("Disconnected CAN network")

    def add_axis(self, node_id: int, eds_path: str, sdo_timeout: float = 2.0) -> None:
        node = canopen.BaseNode402(node_id, eds_path)
        node.sdo.RESPONSE_TIMEOUT = sdo_timeout
        self.net.add_node(node)
        self.nodes[node_id] = node
        self.cache[node_id] = _AxisCache()
        LOG.info("Added axis node_id=%d", node_id)

    def quiet_pdo_traffic(self, node_ids: Optional[List[int]] = None, set_preop: bool = True) -> None:
        """
        Try to stop drives from emitting TPDOs (what you see in candump).
        - Stop SYNC from this process (if any)
        - Set NMT PRE-OPERATIONAL (PDO should stop)
        - Disable TPDO1..TPDO4 and set event_timer=0 (best effort)
        """
        if node_ids is None:
            node_ids = list(self.nodes.keys())

        try:
            self.net.sync.stop()
        except Exception:
            pass

        for nid in node_ids:
            node = self.nodes.get(nid)
            if not node:
                continue

            if set_preop:
                try:
                    node.nmt.state = "PRE-OPERATIONAL"
                    time.sleep(0.02)
                except Exception:
                    pass

            try:
                node.tpdo.read()
            except Exception:
                pass

            for tpdo_num in [1, 2, 3, 4]:
                try:
                    p = node.tpdo[tpdo_num]
                    try:
                        p.callbacks.clear()
                    except Exception:
                        pass

                    p.enabled = False
                    try:
                        p.event_timer = 0
                    except Exception:
                        pass
                    try:
                        p.inhibit_time = 0
                    except Exception:
                        pass
                except Exception:
                    pass

            try:
                node.tpdo.save()
            except Exception:
                pass


class CanopenDs402IO(Ds402IO):
    """
    CANopen implementation of Ds402IO.
    - Writes: RPDO where possible (atomic per-axis)
    - Reads: TPDO callbacks update cache; read_* returns cache with SDO fallback for statusword
    """

    def __init__(self, bus: CanopenBus, axis_id: int, pdo_cfg: PdoMapConfig = PDO_MAP):
        self.bus = bus
        self.axis_id = axis_id
        self.node = bus.nodes[axis_id]
        self.cfg = pdo_cfg

        self._tpdo_sw = None
        self._tpdo_fb = None

    def _pdo_has(self, pdo, name: str) -> bool:
        try:
            _ = pdo[name]
            return True
        except Exception:
            return False

    # ---------- TPDO receive callbacks ----------
    def bind_tpdo_callbacks(self) -> None:
        tpdo_sw = self.node.tpdo[self.cfg.tpdo_status_num]
        tpdo_fb = self.node.tpdo[self.cfg.tpdo_fb_num]
        self._tpdo_sw = tpdo_sw
        self._tpdo_fb = tpdo_fb

        def on_status(_map):
            try:
                sw = tpdo_sw[self.cfg.statusword_name].raw
                c = self.bus.cache[self.axis_id]
                c.statusword = int(sw)
                c.ts = time.time()
            except Exception as e:
                LOG.debug("TPDO status parse error axis=%d: %s", self.axis_id, e)

        def on_fb(_map):
            try:
                pos = tpdo_fb[self.cfg.position_name].raw
                vel = tpdo_fb[self.cfg.velocity_name].raw
                c = self.bus.cache[self.axis_id]
                c.position_counts = int(pos)
                c.velocity_counts_s = int(vel)
                c.ts = time.time()
            except Exception as e:
                LOG.debug("TPDO fb parse error axis=%d: %s", self.axis_id, e)

        tpdo_sw.add_callback(on_status)
        tpdo_fb.add_callback(on_fb)

    def unbind_tpdo_callbacks(self) -> None:
        for p in [self._tpdo_sw, self._tpdo_fb]:
            if p is None:
                continue
            try:
                p.callbacks.clear()
            except Exception:
                pass

    # ---------- Ds402IO ----------
    def write_controlword(self, cw: int) -> None:
        rpdo = self.node.rpdo[self.cfg.rpdo_vel_num]
        if self._pdo_has(rpdo, self.cfg.controlword_name):
            rpdo[self.cfg.controlword_name].raw = cw
            safe_transmit(rpdo)
            return
        self.node.sdo[0x6040].raw = cw

    def read_statusword(self) -> int:
        c = self.bus.cache[self.axis_id]
        if c.ts <= 0.0 or (time.time() - c.ts) > 0.5:
            try:
                sw = int(self.node.sdo[0x6041].raw)
                c.statusword = sw
                c.ts = time.time()
            except Exception:
                pass
        return c.statusword

    def write_mode_of_operation(self, mode: int) -> None:
        self.node.sdo[0x6060].raw = mode

    def write_target_velocity_counts_s(self, vel_counts_s: int, cw: int = 0x000F) -> None:
        rpdo = self.node.rpdo[self.cfg.rpdo_vel_num]
        if self._pdo_has(rpdo, self.cfg.controlword_name) and self._pdo_has(rpdo, self.cfg.target_velocity_name):
            rpdo[self.cfg.controlword_name].raw = cw
            rpdo[self.cfg.target_velocity_name].raw = int(vel_counts_s)
            safe_transmit(rpdo)
            return
        self.node.sdo[0x60FF].raw = int(vel_counts_s)
        self.write_controlword(cw)

    def write_target_position_counts(self, pos_counts: int, cw: int = 0x000F) -> None:
        rpdo = self.node.rpdo[self.cfg.rpdo_pos_num]
        if self._pdo_has(rpdo, self.cfg.target_position_name):
            if self._pdo_has(rpdo, self.cfg.controlword_name):
                rpdo[self.cfg.controlword_name].raw = cw
            rpdo[self.cfg.target_position_name].raw = int(pos_counts)
            safe_transmit(rpdo)
            return
        self.node.sdo[0x607A].raw = int(pos_counts)
        self.write_controlword(cw)

    def read_position_counts(self) -> Optional[int]:
        return self.bus.cache[self.axis_id].position_counts

    def read_velocity_counts_s(self) -> Optional[int]:
        return self.bus.cache[self.axis_id].velocity_counts_s

    # ---------- Optional bus-friendly PDO config helpers ----------
    def configure_tpdo_limit_rate(
        self,
        tpdo_num: int,
        enable: bool = True,
        trans_type: int = 255,
        event_timer_ms: int = 50,
        inhibit_time_100us: int = 0
    ) -> None:
        pdo = self.node.tpdo[tpdo_num]
        pdo.enabled = enable
        try:
            pdo.trans_type = trans_type
        except Exception:
            pass
        try:
            pdo.event_timer = event_timer_ms
        except Exception:
            pass
        try:
            pdo.inhibit_time = inhibit_time_100us
        except Exception:
            pass
        pdo.save()

    # ---------- Vendor: zero position ----------
    def zero_position_counter(self, servo_off_first: bool = True, retries: int = 3) -> bool:
        """
        Vendor command per manual:
          write 0x200C = 1  -> clears actual position (0x6064) to 0, 0x200C auto returns to 0.
        Returns True if 0x6064 becomes 0 (best-effort).
        """
        last_exc = None
        for _ in range(max(1, retries)):
            try:
                if servo_off_first:
                    try:
                        self.node.sdo[0x6040].raw = 0x0000  # disable voltage
                        time.sleep(0.02)
                    except Exception:
                        pass

                before = safe_sdo_read(self.node, 0x6064, default=None)

                self.node.sdo[0x200C].raw = 1
                time.sleep(0.05)

                after = safe_sdo_read(self.node, 0x6064, default=None)
                flag = safe_sdo_read(self.node, 0x200C, default=None)

                LOG.info("Axis %d zero via 0x200C: 0x6064 %s -> %s, 0x200C=%s",
                         self.axis_id, before, after, flag)
                return (after == 0)
            except Exception as e:
                last_exc = e
                time.sleep(0.05)

        LOG.warning("Axis %d zero via 0x200C failed: %s", self.axis_id, last_exc)
        return False


# ------------------------------
# High-level Motor implementation
# ------------------------------
class Ds402Motor(Motor):
    def __init__(self, axis_id: int, io: CanopenDs402IO, kin: MotorKinematics):
        self.axis_id = axis_id
        self.io = io
        self.kin = kin
        self.sm = Ds402StateMachine(io)
        self._mode: ControlModeName = "PV"

    def init(self, default_mode: ControlModeName = "PV", zero_on_init: bool = True) -> None:
        node = self.io.node

        # PRE-OP: safe for config / SDO
        node.nmt.state = "PRE-OPERATIONAL"
        time.sleep(0.05)

        # Load mappings
        node.load_configuration()
        node.tpdo.read()
        node.rpdo.read()

        # Zero position via vendor object (0x200C=1), best effort
        if zero_on_init:
            try:
                ok = self.io.zero_position_counter(servo_off_first=True, retries=3)
                if not ok:
                    LOG.warning("Axis %d: zero_on_init did not confirm 0x6064==0", self.axis_id)
            except Exception as e:
                LOG.warning("Axis %d: zero_on_init exception: %s", self.axis_id, e)

        self._assert_required_pdo_fields()
        self.io.bind_tpdo_callbacks()

        # Disable unused TPDOs by default: keep TPDO1 & TPDO4
        for tpdo_num in [2, 3]:
            try:
                node.tpdo[tpdo_num].enabled = False
            except Exception:
                pass
        try:
            node.tpdo.save()
        except Exception:
            pass

        # Limit TPDO rate (example 20Hz)
        try:
            self.io.configure_tpdo_limit_rate(self.io.cfg.tpdo_status_num, enable=True, trans_type=255, event_timer_ms=50)
            self.io.configure_tpdo_limit_rate(self.io.cfg.tpdo_fb_num, enable=True, trans_type=255, event_timer_ms=50)
        except Exception as e:
            LOG.warning("TPDO rate limit config failed axis=%d: %s", self.axis_id, e)

        node.nmt.state = "OPERATIONAL"
        time.sleep(0.05)

        self.set_mode(default_mode)
        LOG.info("Axis %d init done, NMT=%s, default_mode=%s, counts_per_out_rev=%.0f",
                 self.axis_id, node.nmt.state, default_mode, self.kin.counts_per_output_rev)

    def _assert_required_pdo_fields(self) -> None:
        cfg = self.io.cfg
        node = self.io.node

        # TPDO status
        _ = node.tpdo[cfg.tpdo_status_num][cfg.statusword_name]
        # TPDO feedback
        _ = node.tpdo[cfg.tpdo_fb_num][cfg.position_name]
        _ = node.tpdo[cfg.tpdo_fb_num][cfg.velocity_name]
        # RPDO velocity command
        _ = node.rpdo[cfg.rpdo_vel_num][cfg.controlword_name]
        _ = node.rpdo[cfg.rpdo_vel_num][cfg.target_velocity_name]
        # RPDO position command (optional but recommended for PP)
        try:
            _ = node.rpdo[cfg.rpdo_pos_num][cfg.target_position_name]
        except Exception:
            LOG.warning("Axis %d: RPDO(pos) mapping not found. PP will fall back to SDO for 0x607A.", self.axis_id)

    # ---------- State control ----------
    def enable(self, timeout_s: float = 5.0) -> None:
        sw = self.io.read_statusword()
        if decode_ds402_state(sw) == Ds402State.FAULT:
            LOG.warning("Axis %d in FAULT, fault_reset()", self.axis_id)
            self.fault_reset()
            time.sleep(0.1)

        self.sm.enable_operation(timeout_s=timeout_s)
        LOG.info("Axis %d enabled (OPERATION_ENABLED)", self.axis_id)

    def disable(self) -> None:
        self.io.write_controlword(0x0000)
        LOG.info("Axis %d disabled", self.axis_id)

    def fault_reset(self) -> None:
        self.sm.fault_reset()
        LOG.info("Axis %d fault reset pulse sent", self.axis_id)

    # ---------- Mode ----------
    def set_mode(self, mode: ControlModeName) -> None:
        if mode not in MODE_MAP:
            raise ValueError(f"Unsupported mode={mode}, supported={list(MODE_MAP.keys())}")
        self.io.write_mode_of_operation(MODE_MAP[mode])
        self._mode = mode
        LOG.info("Axis %d set mode: %s (%d)", self.axis_id, mode, MODE_MAP[mode])

    # ---------- Commands (external: rad / rad/s) ----------
    def set_target_velocity_rad_s(self, rad_s: float) -> None:
        vel_counts_s = rad_s_to_counts_s(rad_s, self.kin)
        self.io.write_target_velocity_counts_s(vel_counts_s, cw=0x000F)

    def _pp_pulse_new_setpoint(self, relative: bool, immediate: bool) -> None:
        """
        PP trigger: controlword bit4 new set-point needs rising edge.
        """
        base = 0x000F
        if immediate:
            base |= (1 << 5)
        if relative:
            base |= (1 << 6)

        self.io.write_controlword(base)              # bit4=0
        self.io.write_controlword(base | (1 << 4))   # bit4=1
        time.sleep(0.002)
        self.io.write_controlword(base)              # bit4=0 again

    def pp_move_to_rad(
        self,
        pos_rad: float,
        relative: bool = False,
        immediate: bool = True,
        wait_ack: bool = False,
        timeout_s: float = 2.0
    ) -> None:
        """
        External pos is output-shaft rad.
        Internal 0x607A is counts.
        For relative=True, the driver interprets target position as increment (counts).
        """
        if self._mode != "PP":
            raise RuntimeError(f"Axis {self.axis_id}: current mode={self._mode}, need PP before pp_move_to_rad()")

        pos_counts = rad_to_counts_pos(pos_rad, self.kin)
        self.io.write_target_position_counts(pos_counts, cw=0x000F)
        self._pp_pulse_new_setpoint(relative=relative, immediate=immediate)

        if wait_ack:
            deadline = time.time() + timeout_s
            while time.time() < deadline:
                sw = self.io.read_statusword()
                if sw_bit(sw, 12) or sw_bit(sw, 10):
                    return
                time.sleep(0.01)
            sw = self.io.read_statusword()
            raise TimeoutError(f"Axis {self.axis_id}: PP ack timeout, SW=0x{sw:04X}")

    def stop(self) -> None:
        # PV/CSV: vel=0 rad/s
        if self._mode in ("PV", "CSV"):
            self.set_target_velocity_rad_s(0.0)

    # ---------- Feedback ----------
    def feedback(self) -> MotorFeedback:
        c = self.io.bus.cache[self.axis_id]

        pos_rad = None
        if c.position_counts is not None:
            pos_rad = counts_to_rad_pos(int(c.position_counts), self.kin)

        vel_rad_s = None
        if c.velocity_counts_s is not None:
            vel_rad_s = counts_s_to_rad_s(int(c.velocity_counts_s), self.kin)

        return MotorFeedback(
            statusword=c.statusword,
            position_counts=c.position_counts,
            velocity_counts_s=c.velocity_counts_s,
            ts=c.ts,
            position_rad=pos_rad,
            velocity_rad_s=vel_rad_s,
        )


# ------------------------------
# Motor manager (multi-axis)
# ------------------------------
class MotorManager:
    def __init__(self, bus: CanopenBus):
        self.bus = bus
        self.motors: Dict[int, Ds402Motor] = {}

    def add_motor(
        self,
        node_id: int,
        eds_path: str,
        *,
        encoder_cpr: int = 2 ** 16,
        gear_ratio: float = 10.0,
    ) -> Ds402Motor:
        self.bus.add_axis(node_id, eds_path)
        io = CanopenDs402IO(self.bus, node_id, PDO_MAP)
        kin = MotorKinematics(encoder_cpr=encoder_cpr, gear_ratio=gear_ratio)
        m = Ds402Motor(node_id, io, kin)
        self.motors[node_id] = m
        return m

    def init_all(self, default_mode: ControlModeName = "PV", zero_on_init: bool = True) -> None:
        for m in self.motors.values():
            m.init(default_mode=default_mode, zero_on_init=zero_on_init)

    def enable_all(self, timeout_s: float = 5.0) -> None:
        for m in self.motors.values():
            m.enable(timeout_s=timeout_s)

    def set_mode_all(self, mode: ControlModeName) -> None:
        for m in self.motors.values():
            m.set_mode(mode)

    # ---- batch commands (external: rad / rad/s) ----
    def set_velocities_rad_s(self, vel_by_axis: Dict[int, float]) -> None:
        for axis_id, rad_s in vel_by_axis.items():
            self.motors[axis_id].set_target_velocity_rad_s(rad_s)

    def pp_move_many_rad(
        self,
        pos_by_axis_rad: Dict[int, float],
        relative: bool = False,
        immediate: bool = True,
        wait_ack: bool = False,
        timeout_s: float = 2.0
    ) -> None:
        for axis_id, pos_rad in pos_by_axis_rad.items():
            self.motors[axis_id].pp_move_to_rad(
                pos_rad, relative=relative, immediate=immediate,
                wait_ack=wait_ack, timeout_s=timeout_s
            )

    def stop_all(self) -> None:
        for m in self.motors.values():
            try:
                m.stop()
            except Exception:
                pass

    def disable_all(self) -> None:
        for m in self.motors.values():
            try:
                m.disable()
            except Exception:
                pass

    # -------- Emergency stop + quiet PDO traffic --------
    def estop_quiet_all(self, stage_sleep_s: float = 0.10, quiet_pdo: bool = True) -> None:
        """
        Stop state:
          Stage1: CW=0x000F, target vel=0
          Stage2: CW=0x000B (quick stop), target vel=0
          Stage3: CW=0x0000 (disable voltage), target vel=0
        Then (optional): PRE-OP + disable TPDO + event_timer=0 (best effort).
        """
        # ensure OPERATIONAL so PDO writes are allowed
        for m in self.motors.values():
            try:
                m.io.node.nmt.state = "OPERATIONAL"
            except Exception:
                pass

        # stage 1
        for m in self.motors.values():
            try:
                m.io.write_target_velocity_counts_s(0, cw=0x000F)
            except Exception:
                pass
        time.sleep(stage_sleep_s)

        # stage 2
        for m in self.motors.values():
            try:
                m.io.write_target_velocity_counts_s(0, cw=0x000B)
            except Exception:
                pass
        time.sleep(stage_sleep_s)

        # stage 3
        for m in self.motors.values():
            try:
                m.io.write_target_velocity_counts_s(0, cw=0x0000)
            except Exception:
                pass

        # stop local callbacks immediately (best effort)
        for m in self.motors.values():
            try:
                m.io.unbind_tpdo_callbacks()
            except Exception:
                pass

        if quiet_pdo:
            self.bus.quiet_pdo_traffic(node_ids=list(self.motors.keys()), set_preop=True)


# ------------------------------
# Example main
# ------------------------------
def main():
    EDS_PATH = "/home/nvidia/github/robot_base_ctl/motor/moons/CANOPEN-EDS-MBDV-Servo-SingleAxis-V1.1.1.eds"
    NODE_IDS = [1, 2, 3]
    CAN_CHANNEL = "can0"
    BITRATE = 1_000_000

    # Your motor: encoder=2^16, gear_ratio=10 -> 655360 counts/output_rev
    ENCODER_CPR = 2 ** 16
    GEAR_RATIO = 10.0

    bus = CanopenBus(channel=CAN_CHANNEL, bitrate=BITRATE, interface="socketcan")
    mgr = MotorManager(bus)

    try:
        bus.connect(can_filters=None)

        for nid in NODE_IDS:
            mgr.add_motor(nid, EDS_PATH, encoder_cpr=ENCODER_CPR, gear_ratio=GEAR_RATIO)

        mgr.init_all(default_mode="PV", zero_on_init=True)
        mgr.enable_all(timeout_s=8.0)

        # ---- Example: PV, command output-shaft velocity in rad/s ----
        mgr.set_mode_all("PV")
        LOG.info("PV: set all axis velocity (rad/s) ...")

        # Example values:
        #   -pi rad/s corresponds to -0.5 rev/s -> about -327680 counts/s (with 655360 counts/rev)
        mgr.set_velocities_rad_s({1: -math.pi, 2: -math.pi, 3: -math.pi})
        time.sleep(0.5)
        mgr.set_velocities_rad_s({1: -2.0 * math.pi, 2: -2.0 * math.pi, 3: -2.0 * math.pi})  # -1 rev/s
        time.sleep(0.5)

        t0 = time.time()
        while time.time() - t0 < 3.0:
            for nid in NODE_IDS:
                fb = mgr.motors[nid].feedback()
                st = decode_ds402_state(fb.statusword)
                print(
                    f"[PV] Axis{nid} SW=0x{fb.statusword:04X}({st}) "
                    f"Pos={fb.position_counts}cnt ({fb.position_rad:.3f}rad) "
                    f"Vel={fb.velocity_counts_s}cnt/s ({fb.velocity_rad_s:.3f}rad/s) "
                    f"ts={fb.ts:.3f}"
                )
            time.sleep(0.1)

        mgr.set_velocities_rad_s({1: 0.0, 2: 0.0, 3: 0.0})
        time.sleep(0.2)

        # ---- Example: PP, command output-shaft position in rad ----
        # mgr.set_mode_all("PP")
        # LOG.info("PP: move to target positions (rad) ...")
        # mgr.pp_move_many_rad({1: 2*math.pi, 2: 0.0, 3: -math.pi}, relative=False, immediate=True, wait_ack=False)
        # time.sleep(1.0)

        LOG.info("Demo done, emergency stop + quiet pdo ...")
        mgr.estop_quiet_all(stage_sleep_s=0.10, quiet_pdo=True)

    except KeyboardInterrupt:
        LOG.warning("Interrupted, emergency stop + quiet pdo ...")
        try:
            mgr.estop_quiet_all(stage_sleep_s=0.10, quiet_pdo=True)
        except Exception:
            pass
    except Exception as e:
        LOG.exception("ERROR: %s", e)
        try:
            mgr.estop_quiet_all(stage_sleep_s=0.10, quiet_pdo=True)
        except Exception:
            pass
    finally:
        bus.disconnect()


if __name__ == "__main__":
    main()
