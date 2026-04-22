#!/usr/bin/env python3
"""
CANopen lift column monitor (read-only)
Reads TPDO position and status via CANopen, no control commands.
"""

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
logger = logging.getLogger("lift_column_pp_readonly")


@dataclass
class LiftConfig:
    eds_file: str = EDS_SINGLE_AXIS
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


class LiftColumnMonitor:
    """
    只读监视器：
    - 不发送 RPDO
    - 不写 SDO 参数
    - 不做 DS402 使能
    - 仅接收 TPDO / 读取状态
    """
    def __init__(self, config: LiftConfig):
        self.cfg = config
        self.network: Optional[canopen.Network] = None
        self.node: Optional[canopen.BaseNode402] = None

        self.tpdo_position = None

        self.state = SharedState()
        self.status_thread: Optional[threading.Thread] = None

    # -------------------------------------------------------------------------
    # Conversion
    # -------------------------------------------------------------------------
    def pulse_to_height_mm(self, position_pulse: int) -> float:
        return self.cfg.zero_encoder_height_mm + (float(position_pulse) / self.cfg.pulse_per_mm)

    def get_status_snapshot(self) -> dict:
        snap = self.state.snapshot()
        pos = snap["position"]
        snap["height_mm"] = None if pos is None else self.pulse_to_height_mm(pos)
        return snap

    # -------------------------------------------------------------------------
    # CANopen / PDO initialization (READ ONLY)
    # -------------------------------------------------------------------------
    def initialize(self) -> None:
        logger.info("Initializing lift column monitor (READ ONLY)")
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
        logger.info("NMT current state: %s", self.node.nmt.state)

        # --- 新增：将节点切换至 OPERATIONAL 状态 ---
        # 发送 NMT 启动命令（0x01），使节点进入 OPERATIONAL
        self.node.nmt.send_command(0x01)  # 0x01 = Start
        # 等待节点状态变为 OPERATIONAL
        timeout = 3.0
        start_time = time.time()
        while self.node.nmt.state != 'OPERATIONAL' and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        if self.node.nmt.state != 'OPERATIONAL':
            logger.warning("Node did not reach OPERATIONAL state within %.1f seconds", timeout)
        else:
            logger.info("Node entered OPERATIONAL state")
        # --- 新增结束 ---

        self._read_pdo_configuration()
        self._register_pdo_callbacks()

        logger.info("Read-only initialization complete")

    def _read_pdo_configuration(self) -> None:
        assert self.node is not None
        logger.info("[1] Read PDO configuration")

        # 读取节点对象字典中的 PDO 配置
        self.node.tpdo.read()

        self.tpdo_position = self.node.tpdo[self.cfg.position_tpdo_index]

        logger.info(
            "TPDO position index=%d COB-ID=0x%08X",
            self.cfg.position_tpdo_index, self.tpdo_position.cob_id
        )

        logger.info("TPDO mapped objects:")
        for obj in self.tpdo_position.map:
            logger.info("  - 0x%04X:%02X %s", obj.index, obj.subindex, obj.name)

    def _register_pdo_callbacks(self) -> None:
        logger.info("[2] Register PDO callbacks")

        def on_position_tpdo(_):
            try:
                # 实际位置 0x6064
                position = int(self.tpdo_position[0x6064].raw)
                self.state.update_position(position)
            except Exception as exc:
                logger.warning("Parse position TPDO failed: %s", exc)

        self.tpdo_position.add_callback(on_position_tpdo)

    # -------------------------------------------------------------------------
    # State / status API (READ ONLY)
    # -------------------------------------------------------------------------
    def get_current_position(self) -> int:
        """
        优先取 TPDO 回调缓存。
        若暂时没收到 TPDO，再尝试 SDO 读取 0x6064 实际位置。
        """
        assert self.node is not None
        snap = self.state.snapshot()

        if snap["position"] is not None:
            return int(snap["position"])

        try:
            return int(self.node.sdo[0x6064].raw)
        except Exception as exc:
            logger.warning("Read actual position via SDO failed: %s", exc)
            return 0

    def get_current_height_mm(self) -> float:
        return self.pulse_to_height_mm(self.get_current_position())

    def get_nmt_state(self) -> str:
        assert self.node is not None
        try:
            return str(self.node.nmt.state)
        except Exception:
            return "UNKNOWN"

    def get_ds402_state(self) -> str:
        assert self.node is not None
        try:
            return str(self.node.state)
        except Exception as exc:
            logger.warning("Read DS402 state failed: %s", exc)
            return "UNKNOWN"

    def try_read_statusword(self) -> Optional[int]:
        assert self.node is not None
        try:
            return int(self.node.sdo[0x6041].raw)
        except Exception as exc:
            logger.warning("Read statusword failed: %s", exc)
            return None

    def try_read_op_mode_display(self):
        assert self.node is not None
        try:
            return self.node.sdo[0x6061].raw
        except Exception as exc:
            logger.warning("Read mode display failed: %s", exc)
            return None

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
                # 只断开，不改节点状态，避免额外写操作
                self.network.disconnect()
                print("  ✓ Network disconnected")
            except Exception as exc:
                print(f"  ! Cleanup error: {exc}")

        print("Goodbye!")


def main():
    monitor = LiftColumnMonitor(LiftConfig())

    try:
        print("=" * 60)
        print("CANopen Lift Column Read-Only Communication Test")
        print("Read status only, no control command will be sent")
        print("=" * 60)

        monitor.initialize()
        monitor.start_status_publisher()

        current_pos = monitor.get_current_position()
        current_height = monitor.get_current_height_mm()
        nmt_state = monitor.get_nmt_state()
        ds402_state = monitor.get_ds402_state()
        statusword = monitor.try_read_statusword()
        op_mode_display = monitor.try_read_op_mode_display()

        print("\n[INFO] Current state")
        print(f"    NMT state         : {nmt_state}")
        print(f"    DS402 state       : {ds402_state}")
        print(f"    Current position  : {current_pos}")
        print(f"    Current height(mm): {current_height:.2f}")
        print(
            f"    Valid range(mm)   : "
            f"{monitor.cfg.min_height_mm:.2f} ~ {monitor.cfg.max_height_mm:.2f}"
        )

        if statusword is not None:
            print(f"    Statusword        : 0x{statusword:04X}")
        else:
            print("    Statusword        : None")

        if op_mode_display is not None:
            print(f"    Op mode display   : {op_mode_display}")
        else:
            print("    Op mode display   : None")

        print("\nRead-only monitor is running, press Ctrl+C to exit")
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
        monitor.shutdown()


if __name__ == "__main__":
    main()