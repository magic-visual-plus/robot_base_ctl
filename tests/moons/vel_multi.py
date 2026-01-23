import os
import sys
import time
import traceback
import logging
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

import canopen

# -----------------------------------------------------------------------------
# Logging
# -----------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logging.getLogger("can.interfaces.socketcan").setLevel(logging.INFO)

# -----------------------------------------------------------------------------
# Config dataclasses
# -----------------------------------------------------------------------------
@dataclass
class DS402PdoMap:
    # 你原来用的变量名（来自 EDS 的对象名）
    status_word: str = "Status word"
    control_word: str = "Control word"
    actual_velocity: str = "Velocity value calculated"
    target_velocity: str = "Target velocity"

    # 你原来用的 PDO 编号（取决于 EDS 映射）
    tpdo_status_idx: int = 1   # tpdo[1]里有 Status word
    tpdo_vel_idx: int = 3      # tpdo[3]里有 actual_velocity
    rpdo_cmd_idx: int = 3      # rpdo[3]里有 Control word + Target velocity


@dataclass
class VelocityParams:
    profile_acc: int = 6553600
    profile_dec: int = 6553600
    quick_stop_dec: int = 2000000


@dataclass
class NetworkConfig:
    interface: str = "socketcan"
    channel: str = "can0"
    bitrate: int = 50000
    sync_period_s: float = 0.1
    sdo_timeout_s: float = 2.0
    node_guard_period_s: float = 0.01


# -----------------------------------------------------------------------------
# DS402 Multi-motor Velocity Controller
# -----------------------------------------------------------------------------
class DS402VelocityController:
    def __init__(
        self,
        eds_file_path: str,
        node_ids: List[int],
        net_cfg: NetworkConfig = NetworkConfig(),
        pdo_map: DS402PdoMap = DS402PdoMap(),
        vel_params: VelocityParams = VelocityParams(),
        op_mode: str = "PROFILED VELOCITY",
    ):
        self.eds_file_path = eds_file_path
        self.node_ids = node_ids
        self.net_cfg = net_cfg
        self.pdo_map = pdo_map
        self.vel_params = vel_params
        self.op_mode = op_mode

        self.network: Optional[canopen.Network] = None
        self.nodes: Dict[int, canopen.BaseNode402] = {}

    # -----------------------------
    # Public lifecycle
    # -----------------------------
    def connect(self):
        self.network = canopen.Network()
        for nid in self.node_ids:
            node = canopen.BaseNode402(nid, self.eds_file_path)
            node.sdo.RESPONSE_TIMEOUT = self.net_cfg.sdo_timeout_s
            self.network.add_node(node)
            self.nodes[nid] = node

        try:
            self.network.connect(
                interface=self.net_cfg.interface,
                channel=self.net_cfg.channel,
                bitrate=self.net_cfg.bitrate,
            )
            print(f"Connected to CAN interface: {self.net_cfg.channel}")
        except Exception as e:
            raise RuntimeError(f"Failed to connect to CAN bus: {e}") from e

        self.network.check()

        # Start SYNC
        self.network.sync.start(self.net_cfg.sync_period_s)
        print(f"✓ SYNC started ({self.net_cfg.sync_period_s*1000:.0f} ms)")

    def setup_all_nodes(self):
        for nid, node in self.nodes.items():
            print("\n" + "=" * 60)
            print(f"Setup node {nid}")
            print("=" * 60)
            self._setup_single_node(node)

    def disconnect(self):
        if not self.network:
            return
        try:
            # stop motors + set pre-op + stop guarding
            for nid, node in self.nodes.items():
                try:
                    # 尝试用你映射好的 rpdo_cmd_idx 来清零速度
                    self._safe_set_velocity(node, 0)
                except Exception:
                    pass

                try:
                    node.nmt.state = "PRE-OPERATIONAL"
                except Exception:
                    pass

                try:
                    node.nmt.stop_node_guarding()
                except Exception:
                    pass

                print(f"  ✓ Node {nid} stopped")

            try:
                self.network.sync.stop()
            except Exception:
                pass

            self.network.disconnect()
            print("  ✓ Network disconnected")
        finally:
            self.network = None

    # -----------------------------
    # Public control APIs
    # -----------------------------
    def set_velocity(self, node_id: int, target_vel: int, controlword: int = 0x000F):
        node = self.nodes[node_id]
        self._send_velocity_command(node, target_vel, controlword)

    def set_velocities(self, targets: Dict[int, int], controlword: int = 0x000F):
        """
        多电机一起下发（每个节点发一帧 RPDO）
        """
        for nid, vel in targets.items():
            node = self.nodes[nid]
            self._send_velocity_command(node, vel, controlword)

    def read_feedback(self, node_id: int, timeout: float = 0.5) -> Dict[str, int]:
        """
        返回: actual_velocity, statusword
        """
        node = self.nodes[node_id]
        pm = self.pdo_map

        # 等 status tpdo
        node.tpdo[pm.tpdo_status_idx].wait_for_reception(timeout=timeout)
        # 等 vel tpdo（如果你的 vel 也是独立 tpdo）
        node.tpdo[pm.tpdo_vel_idx].wait_for_reception(timeout=timeout)

        statusword = node.tpdo[pm.tpdo_status_idx][pm.status_word].raw
        actual_vel = node.tpdo[pm.tpdo_vel_idx][pm.actual_velocity].raw
        return {"actual_velocity": actual_vel, "statusword": statusword}

    def run_sequence(
        self,
        sequence: List[Tuple[Dict[int, int], float, str]],
        print_period_s: float = 0.1,
    ):
        """
        sequence: [
          ({1: vel1, 2: vel2, 3: vel3}, duration_s, description),
          ...
        ]
        """
        if not self.network:
            raise RuntimeError("Network not connected")

        pm = self.pdo_map

        print("\n" + "=" * 60)
        print("VELOCITY CONTROL ACTIVE (3 motors)")
        print("Press Ctrl+C to stop")
        print("=" * 60)

        for targets, duration, desc in sequence:
            print(f"\n{desc}")
            print(f"  Targets: {targets} | Duration: {duration:.2f}s")

            # 下发目标速度
            self.set_velocities(targets)

            start = time.time()
            next_print = start

            while (time.time() - start) < duration:
                self.network.check()

                now = time.time()
                if now >= next_print:
                    # 打印每个节点反馈
                    for nid in sorted(self.nodes.keys()):
                        try:
                            fb = self.read_feedback(nid, timeout=0.5)
                            print(
                                f"Node {nid} | "
                                f"Target {targets.get(nid, 0):+7d} | "
                                f"Actual {fb['actual_velocity']:+7d} | "
                                f"SW 0x{fb['statusword']:04X} | "
                                f"State {self.nodes[nid].state}"
                            )
                        except Exception as e:
                            print(f"Node {nid} | ! feedback error: {e}")

                    next_print += print_period_s

                time.sleep(0.001)

    def stop_all(self):
        self.set_velocities({nid: 0 for nid in self.nodes.keys()})

    # -----------------------------------------------------------------------------
    # Internal: per-node setup
    # -----------------------------------------------------------------------------
    def _setup_single_node(self, node: canopen.BaseNode402):
        pm = self.pdo_map

        # PRE-OP then OPERATIONAL
        node.nmt.state = "PRE-OPERATIONAL"
        print(f"✓ Node {node.id} NMT -> {node.nmt.state}")

        # Clear error log (0x1003)
        self._clear_error_log(node)

        # Communication cycle period (0x1006)
        try:
            node.sdo[0x1006].raw = 10
            print("✓ Comm cycle period (0x1006) set")
        except Exception as e:
            print(f"! Warning: set 0x1006 failed: {e}")

        # Load node configuration (PDO mapping etc.)
        try:
            node.load_configuration()
            node.tpdo.read()
            node.rpdo.read()
            print("✓ Configuration loaded, TPDO/RPDO read")
        except Exception as e:
            print(f"! Warning: load_configuration failed: {e}")

        # Setup DS402 state machine
        try:
            node.setup_402_state_machine()
            print("✓ DS402 state machine configured")
        except Exception as e:
            print(f"! Warning: setup_402_state_machine failed: {e}")

        # Switch to OPERATIONAL
        node.nmt.state = "OPERATIONAL"
        print(f"✓ Node {node.id} NMT -> {node.nmt.state}")

        # Wait a TPDO to ensure communication OK
        try:
            node.tpdo[pm.tpdo_status_idx].wait_for_reception(timeout=1.0)
        except Exception as e:
            print(f"! Warning: TPDO wait failed: {e}")

        # Fault reset if needed
        if node.is_faulted():
            print("! Node is FAULT, sending fault reset...")
            self._fault_reset(node, pm)
        else:
            print("✓ No fault detected")

        # Bring to OPERATION ENABLED (robust loop)
        self._transition_to_op_enabled(node)

        # Set operation mode
        try:
            node.op_mode = self.op_mode
            print(f"✓ op_mode set -> {node.op_mode}")
        except Exception as e:
            print(f"! Warning: set op_mode failed: {e}")

        # Set velocity parameters if objects exist
        self._set_velocity_params(node)

        # Start node guarding
        try:
            node.nmt.start_node_guarding(self.net_cfg.node_guard_period_s)
            print(f"✓ Node guarding started ({self.net_cfg.node_guard_period_s*1000:.0f} ms)")
        except Exception as e:
            print(f"! Warning: start_node_guarding failed: {e}")

        print(f"✓ Node {node.id} ready, state={node.state}")

    def _clear_error_log(self, node: canopen.BaseNode402):
        try:
            error_log = node.sdo[0x1003]
            error_count = len(error_log.values())
            if error_count > 0:
                print(f"! Found {error_count} errors:")
                for i, err in enumerate(error_log.values(), 1):
                    print(f"  Error {i}: 0x{err.raw:08X}")
            node.sdo[0x1003][0].raw = 0
            print("✓ Error log cleared (0x1003)")
        except Exception as e:
            print(f"! Warning: clear error log failed: {e}")

    def _fault_reset(self, node: canopen.BaseNode402, pm: DS402PdoMap):
        try:
            node.rpdo[pm.rpdo_cmd_idx][pm.control_word].raw = 0x80
            node.rpdo[pm.rpdo_cmd_idx].transmit()
            time.sleep(0.1)
            node.rpdo[pm.rpdo_cmd_idx][pm.control_word].raw = 0x00
            node.rpdo[pm.rpdo_cmd_idx].transmit()
            time.sleep(0.2)
            print("✓ Fault reset sent")
        except Exception as e:
            print(f"! Fault reset failed: {e}")

    def _transition_to_op_enabled(self, node: canopen.BaseNode402, timeout_s: float = 15.0):
        # 这里用 node.state 的状态机封装（canopen BaseNode402 提供）
        for st in ["READY TO SWITCH ON", "SWITCHED ON", "OPERATION ENABLED"]:
            t_end = time.time() + timeout_s
            node.state = st
            while node.state != st:
                if time.time() > t_end:
                    raise TimeoutError(f"Timeout entering state {st} on node {node.id}")
                time.sleep(0.001)

    def _set_velocity_params(self, node: canopen.BaseNode402):
        vp = self.vel_params
        try:
            if 0x6083 in node.sdo.keys():
                node.sdo[0x6083].raw = vp.profile_acc
                print("✓ Profile acc (0x6083) set")
            if 0x6084 in node.sdo.keys():
                node.sdo[0x6084].raw = vp.profile_dec
                print("✓ Profile dec (0x6084) set")
            if 0x6085 in node.sdo.keys():
                node.sdo[0x6085].raw = vp.quick_stop_dec
                print("✓ Quick stop dec (0x6085) set")
        except Exception as e:
            print(f"! Warning: set velocity params failed: {e}")

    def _send_velocity_command(self, node: canopen.BaseNode402, target_vel: int, controlword: int):
        pm = self.pdo_map
        # 关键：同一 RPDO 同时写 controlword + target velocity，再 transmit
        node.rpdo[pm.rpdo_cmd_idx][pm.control_word].raw = controlword
        node.rpdo[pm.rpdo_cmd_idx][pm.target_velocity].raw = int(target_vel)
        node.rpdo[pm.rpdo_cmd_idx].transmit()

    def _safe_set_velocity(self, node: canopen.BaseNode402, vel: int):
        self._send_velocity_command(node, vel, controlword=0x000F)


# -----------------------------------------------------------------------------
# Demo main
# -----------------------------------------------------------------------------
def main():
    eds_file_path = "/home/nvidia/github/robot_base_ctl/motor/moons/CANOPEN-EDS-MBDV-Servo-SingleAxis-V1.1.1.eds"
    node_ids = [1, 2, 3]  # 支持 1/2/3

    ctrl = DS402VelocityController(
        eds_file_path=eds_file_path,
        node_ids=node_ids,
        net_cfg=NetworkConfig(channel="can0", bitrate=50000),
        op_mode="PROFILED VELOCITY",
    )

    try:
        print("=" * 60)
        print("CANopen DS402 Velocity Control Demo (3 motors)")
        print("=" * 60)

        ctrl.connect()
        ctrl.setup_all_nodes()

        # 速度序列：每一步是 (targets_dict, duration, description)
        sequence = [
            ({1: -327680, 2: -327680, 3: -327680}, 1.0, "Step 1: all motors to -327680"),
            ({1: 0, 2: 0, 3: 0}, 1.0, "Step 2: stop"),
            ({1: 327680, 2: -327680, 3: 327680}, 2.0, "Step 3: mixed directions"),
            ({1: 0, 2: 0, 3: 0}, 1.0, "Final stop"),
        ]

        ctrl.run_sequence(sequence, print_period_s=0.1)

    except KeyboardInterrupt:
        print("\n! User interrupted - stopping...")
    except Exception as e:
        exc_type, _, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1] if exc_tb else "unknown"
        print(f"\n! ERROR: {exc_type.__name__} in {fname}:{exc_tb.tb_lineno if exc_tb else '??'}")
        print(f"  Message: {e}")
        traceback.print_exc()
    finally:
        print("\n[CLEANUP] stopping motors and disconnecting...")
        try:
            ctrl.stop_all()
            time.sleep(0.2)
        except Exception:
            pass
        ctrl.disconnect()
        print("Goodbye!")


if __name__ == "__main__":
    main()
