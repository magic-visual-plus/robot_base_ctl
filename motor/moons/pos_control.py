#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
CANopen DS402 Profiled Position Demo (RELATIVE / INCREMENTAL) - Structured

目标：
1) 把“状态机初始化”和“运动控制”分开，代码更清晰
2) 初始化时把位置计数清零（不让电机动、不是跑到0）
3) 使用 PP(Profiled Position) 相对模式：0x607A + 控制字bit4上升沿触发

PDO mapping assumed:
  RPDO2 (COB-ID 0x301): 0x6040 Control word + 0x607A Target position
  TPDO1 (COB-ID 0x181): 0x6041 Status word
  TPDO2 (COB-ID 0x281): 0x6064 Position value calculated
"""

import os
import sys
import time
import traceback
import logging
from dataclasses import dataclass
import canopen


# ------------------- Logging -------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logging.getLogger("can.interfaces.socketcan").setLevel(logging.WARNING)


# ------------------- Config -------------------
@dataclass(frozen=True)
class Config:
    eds_file: str = "/home/nvidia/github/robot_base_ctl/motor/moons/CANOPEN-EDS-MBDV-Servo-SingleAxis-V1.1.1.eds"
    node_id: int = 2

    can_interface: str = "socketcan"
    can_channel: str = "can0"
    can_bitrate: int = 50000

    sync_period_s: float = 0.1
    node_guard_s: float = 0.01

    # EDS object names (must match EDS)
    ctl: str = "Control word"                   # 0x6040
    status: str = "Status word"                 # 0x6041
    tgt_pos: str = "Target position"            # 0x607A
    act_pos: str = "Position value calculated"  # 0x6064


CFG = Config()


# ------------------- Helpers: Read / Wait -------------------
def safe_sdo_read(node: canopen.BaseNode402, idx: int, sub=None, default=None):
    try:
        if sub is None:
            return node.sdo[idx].raw
        return node.sdo[idx][sub].raw
    except Exception:
        return default


def wait_tpdo(node: canopen.BaseNode402, tpdo_id: int, timeout: float = 0.5) -> bool:
    try:
        node.tpdo[tpdo_id].wait_for_reception(timeout=timeout)
        return True
    except Exception:
        return False


def dump_feedback(node: canopen.BaseNode402) -> None:
    pos = safe_sdo_read(node, 0x6064, default=0)
    vel = safe_sdo_read(node, 0x606C, default=0)
    tq  = safe_sdo_read(node, 0x6077, default=0)
    cur = safe_sdo_read(node, 0x6078, default=0)
    print(f"    SDO  POS={pos:+12d}  VEL={vel:+8d}  TQ={tq:+8d}  CUR={cur:+8d}")


# ------------------- Part A: State / Init -------------------
class DS402Controller:
    """
    专门负责：网络连接 / NMT / DS402 状态机 / 模式设置 / 参数设置 / 清零当前位置
    """
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.network = canopen.Network()
        self.node = canopen.BaseNode402(cfg.node_id, cfg.eds_file)
        self.network.add_node(self.node)
        self.node.sdo.RESPONSE_TIMEOUT = 2.0

    def connect(self):
        self.network.connect(
            interface=self.cfg.can_interface,
            channel=self.cfg.can_channel,
            bitrate=self.cfg.can_bitrate
        )
        logging.info("Connected to CAN")
        self.network.check()

    def disconnect(self):
        try:
            self.network.sync.stop()
        except Exception:
            pass
        try:
            self.network.disconnect()
        except Exception:
            pass

    def set_nmt(self, state: str):
        self.node.nmt.state = state

    def start_sync(self):
        try:
            self.network.sync.start(self.cfg.sync_period_s)
        except Exception as e:
            print(f"    ! SYNC start failed (can still work): {e}")

    def load_pdos(self):
        self.node.load_configuration()
        self.node.tpdo.read()
        self.node.rpdo.read()

    def setup_state_machine(self):
        self.node.setup_402_state_machine()

    def clear_error_log(self):
        try:
            self.node.sdo[0x1003][0].raw = 0
        except Exception as e:
            print(f"    ! Could not clear error log: {e}")

    def set_comm_cycle(self, cycle_ms: int = 10):
        # 0x1006 comm cycle period
        if 0x1006 in self.node.sdo.keys():
            try:
                self.node.sdo[0x1006].raw = cycle_ms
            except Exception as e:
                print(f"    ! Could not set 0x1006: {e}")

    def ensure_operational(self):
        self.set_nmt("OPERATIONAL")

    def fault_reset_if_needed(self):
        wait_tpdo(self.node, 1, timeout=0.5)
        if self.node.is_faulted():
            print("    ! FAULT detected, try reset...")
            # fault reset bit7
            self.node.rpdo[1][self.cfg.ctl].raw = 0x0080
            self.node.rpdo[1].transmit()
            time.sleep(0.05)
            self.node.rpdo[1][self.cfg.ctl].raw = 0x0000
            self.node.rpdo[1].transmit()
            time.sleep(0.2)

    def enable_operation(self):
        for target in ["READY TO SWITCH ON", "SWITCHED ON", "OPERATION ENABLED"]:
            timeout = time.time() + 10
            self.node.state = target
            while self.node.state != target:
                if time.time() > timeout:
                    raise RuntimeError(f"Timeout DS402 -> {target}, current={self.node.state}")
                time.sleep(0.005)

    def set_profiled_position_mode(self):
        self.node.op_mode = "PROFILED POSITION"
        # 某些驱动 0x6061 可能是 write-only / 或没映射TPDO，会读失败，忽略即可
        mode_6060 = safe_sdo_read(self.node, 0x6060, default=None)
        mode_6061 = safe_sdo_read(self.node, 0x6061, default=None)
        print(f"    Mode 0x6060={mode_6060}  0x6061={mode_6061}")

    def set_profile_params(self, vel: int, acc: int, dec: int):
        # 0x6081/83/84 单位取决于驱动配置（你现在用 counts/s 等）
        if 0x6081 in self.node.sdo.keys():
            self.node.sdo[0x6081].raw = int(vel)
        if 0x6083 in self.node.sdo.keys():
            self.node.sdo[0x6083].raw = int(acc)
        if 0x6084 in self.node.sdo.keys():
            self.node.sdo[0x6084].raw = int(dec)

    def start_node_guarding(self):
        try:
            self.node.nmt.start_node_guarding(self.cfg.node_guard_s)
        except Exception as e:
            print(f"    ! Node guarding start failed (can still work): {e}")

    def clear_position_counter_without_motion(self) -> bool:
        """
        重点：清零“当前位置计数器”，不让电机动、不发目标位置。
        常见做法：写 0x6064 (Position actual value) = 0
        - 如果驱动允许写，就会直接把内部位置计数清零（类似你在Luna里点C）
        - 如果不允许写，会抛异常，我们再考虑 fallback
        """
        try:
            before = safe_sdo_read(self.node, 0x6064, default=None)
            self.node.sdo[0x6064].raw = 0
            time.sleep(0.02)
            after = safe_sdo_read(self.node, 0x6064, default=None)
            print(f"    ✓ Cleared position counter: {before} -> {after} (no motion)")
            return True
        except Exception as e:
            print(f"    ! Could not write 0x6064 to clear counter: {e}")
            return False


# ------------------- Part B: Motion / Control -------------------
class ProfiledPositionIncremental:
    """
    专门负责：PP 相对增量运动指令
    """
    def __init__(self, node: canopen.BaseNode402, cfg: Config):
        self.node = node
        self.cfg = cfg

    def _rpdo2(self):
        return self.node.rpdo[2]  # RPDO2: Controlword + Target position

    def send_relative_move(self, delta_counts: int):
        """
        相对增量移动：
          - 0x607A = delta
          - 控制字 bit6=1(relative)
          - bit4 新设定点上升沿：0->1->0
          - bit5 change immediately=1（建议）
        """
        rpdo = self._rpdo2()

        # 写增量目标
        rpdo[self.cfg.tgt_pos].raw = int(delta_counts)

        base_cw = 0x000F | (1 << 5) | (1 << 6)  # OP enabled + immediate + relative

        # 保证 bit4=0
        rpdo[self.cfg.ctl].raw = base_cw
        rpdo.transmit()
        time.sleep(0.005)

        # bit4=1 触发
        rpdo[self.cfg.ctl].raw = (base_cw | (1 << 4))
        rpdo.transmit()
        time.sleep(0.01)

        # bit4=0 清掉
        rpdo[self.cfg.ctl].raw = base_cw
        rpdo.transmit()


# ------------------- Main -------------------
def main():
    ctrl = DS402Controller(CFG)

    try:
        print("=" * 60)
        print("CANopen DS402 PP Demo (RELATIVE) - Structured")
        print("=" * 60)

        # -------- 状态初始化区（State / Init）--------
        ctrl.connect()

        print(f"before set nmt state {ctrl.node.nmt.state}")
        ctrl.set_nmt("PRE-OPERATIONAL")
        print(f"✓ after set Node nmt state: {ctrl.node.nmt.state}")

        print("\n[INIT] Clear error log")
        ctrl.clear_error_log()

        print("\n[INIT] Set comm cycle (0x1006)")
        ctrl.set_comm_cycle(10)

        print("\n[INIT] Start SYNC (optional)")
        ctrl.start_sync()

        print("\n[INIT] Load configuration & PDOs")
        ctrl.load_pdos()

        print("\n[INIT] Setup DS402 state machine")
        ctrl.setup_state_machine()

        print("\n[INIT] Switch NMT to OPERATIONAL")
        ctrl.ensure_operational()

        print("\n[INIT] Fault reset (if needed)")
        ctrl.fault_reset_if_needed()

        print("\n[INIT] Enable DS402 -> OPERATION ENABLED")
        ctrl.enable_operation()
        print(f"    ✓ DS402 state: {ctrl.node.state}")

        print("\n[INIT] Set mode: PROFILED POSITION")
        ctrl.set_profiled_position_mode()

        print("\n[INIT] Set profile params (tune for speed)")
        # 这里给一个“明显更快”的默认值（你也可以按你系统比例再调）
        ctrl.set_profile_params(
            vel=655360,     # 0x6081
            acc=6553600,    # 0x6083
            dec=6553600     # 0x6084
        )

        print("\n[INIT] Start node guarding (optional)")
        ctrl.start_node_guarding()

        print("\n[INIT] Clear position counter (NO MOTION)")
        # 关键：清零编码器累计值，但不让电机去0
        ctrl.clear_position_counter_without_motion()

        # -------- 控制区（Motion / Control）--------
        mover = ProfiledPositionIncremental(ctrl.node, CFG)

        print("\n[RUN] POSITION CONTROL ACTIVE (RELATIVE)")
        print("=" * 60)

        sequence = [
            ( +655360, 3.0, "Move +delta"),
            ( -655360, 3.0, "Move -delta"),
        ]

        for delta, dur, desc in sequence:
            print(f"\n{desc}")
            print(f"  Command delta = {delta:+d} counts")
            print("  Before:")
            dump_feedback(ctrl.node)

            mover.send_relative_move(delta)

            print("  After:")
            dump_feedback(ctrl.node)

            t0 = time.time()
            while time.time() - t0 < dur:
                got1 = wait_tpdo(ctrl.node, 1, timeout=0.3)
                got2 = wait_tpdo(ctrl.node, 2, timeout=0.3)

                sw = (ctrl.node.tpdo[1][CFG.status].raw if got1 else safe_sdo_read(ctrl.node, 0x6041, default=0))
                ap = (ctrl.node.tpdo[2][CFG.act_pos].raw if got2 else safe_sdo_read(ctrl.node, 0x6064, default=0))
                print(f"  ActualPos={ap:+12d}  Status=0x{sw:04X}  DS402={ctrl.node.state}")
                time.sleep(0.1)

        print("\nDemo done.")

    except KeyboardInterrupt:
        print("\n\n! User interrupted - stopping...")

    except Exception as e:
        exc_type, _, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        print(f"\n! ERROR: {exc_type.__name__} in {fname}:{exc_tb.tb_lineno}")
        print(f"  Message: {e}")
        traceback.print_exc()

    finally:
        print("\n[CLEANUP] Shutting down...")
        try:
            # 退出前把 controlword 维持在相对基态，不再触发 setpoint
            if 2 in ctrl.node.rpdo:
                base_cw = 0x000F | (1 << 5) | (1 << 6)
                ctrl.node.rpdo[2][CFG.ctl].raw = base_cw
                ctrl.node.rpdo[2].transmit()
        except Exception:
            pass

        try:
            ctrl.node.nmt.stop_node_guarding()
        except Exception:
            pass

        try:
            ctrl.set_nmt("PRE-OPERATIONAL")
        except Exception:
            pass

        ctrl.disconnect()
        print("  ✓ Network disconnected\nGoodbye!")


if __name__ == "__main__":
    main()
