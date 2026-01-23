"""
CANopen DS402 Velocity Control Demo (Class-based, more robust)
- Multi motors (Node 1/2/3...)
- Minimal-gap velocity commands (back-to-back RPDO transmit)
- Monitor all motors in the SAME time window
- KEEP ORIGINAL OD STRINGS EXACTLY (including spaces)

Fix for: socketcan "No buffer space available [Errno 105]"
- Stop guarding + stop SYNC before sending final stop RPDO
- Safe transmit with retry/backoff when TX buffer is full
"""

import os
import sys
import time
import errno
import traceback
import logging
import canopen

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logging.getLogger('can.interfaces.socketcan').setLevel(logging.INFO)


class DS402MultiVelocityController:
    # ---- KEEP ORIGINAL OD STRINGS EXACTLY (DO NOT CHANGE) ----
    vel_variable = 'Velocity value calculated'
    status_variable = 'Status word'
    ctl_variable = 'Control word'
    velocity_variable = 'Target velocity'
    # ---------------------------------------------------------

    def __init__(self,
                 eds_file_path: str,
                 node_ids,
                 interface: str = 'socketcan',
                 channel: str = 'can0',
                 bitrate: int = 50000):
        self.eds_file_path = eds_file_path
        self.node_ids = list(node_ids)
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate

        self.network: canopen.Network | None = None
        self.nodes: list[canopen.BaseNode402] = []

        self._sync_started = False
        self._guarding_started = False

    # ---------------- lifecycle ----------------

    def connect(self):
        self.network = canopen.Network()
        self.network.connect(interface=self.interface, channel=self.channel, bitrate=self.bitrate)
        print(f"Connected to CAN interface: {self.channel}")

        self.nodes = []
        for nid in self.node_ids:
            n = canopen.BaseNode402(nid, self.eds_file_path)
            self.network.add_node(n)
            n.sdo.RESPONSE_TIMEOUT = 2.0
            self.nodes.append(n)

        self.network.check()

    def disconnect(self):
        if not self.network:
            return
        try:
            try:
                self.stop_sync()
            except:
                pass
            self.network.disconnect()
            print("  ✓ Network disconnected")
        finally:
            self.network = None

    def close(self):
        print("\n[CLEANUP] Shutting down...")
        try:
            # Best-effort stop (robust)
            try:
                self.stop_all()
            except:
                pass

            # Set pre-op + stop guarding
            if self.nodes:
                for n in self.nodes:
                    try:
                        n.nmt.state = 'PRE-OPERATIONAL'
                    except:
                        pass
                    try:
                        n.nmt.stop_node_guarding()
                    except:
                        pass
                    print(f"  ✓ Node {n.id} stopped")
        except Exception as e:
            print(f"  ! Cleanup error: {e}")
        finally:
            self.disconnect()
            print("\nGoodbye!")

    # ---------------- helpers ----------------

    def _require_net(self):
        if not self.network:
            raise RuntimeError("Network not connected. Call connect() first.")

    def set_nmt_all(self, state: str):
        for n in self.nodes:
            n.nmt.state = state

    def clear_error_log_all(self):
        for n in self.nodes:
            error_log = n.sdo[0x1003]
            error_count = len(error_log.values())
            if error_count > 0:
                print(f"    ! Node {n.id}: Found {error_count} errors in log")
                for i, err in enumerate(error_log.values(), 1):
                    print(f"      Error {i}: 0x{err.raw:08X}")
            n.sdo[0x1003][0].raw = 0
            print(f"    ✓ Node {n.id}: Error log cleared")

    def set_comm_cycle_all(self, cycle_period_ms: int = 10):
        for n in self.nodes:
            n.sdo[0x1006].raw = cycle_period_ms
            print(f"    ✓ Node {n.id}: Communication cycle period set to {cycle_period_ms} (0x1006)")

    def start_sync(self, period_s: float = 0.1):
        self._require_net()
        self.network.sync.start(period_s)
        self._sync_started = True
        print(f"    ✓ SYNC started ({period_s * 1000:.0f}ms)")

    def stop_sync(self):
        if self.network and self._sync_started:
            try:
                self.network.sync.stop()
            finally:
                self._sync_started = False

    def load_config_all(self):
        for n in self.nodes:
            n.load_configuration()
            n.tpdo.read()
            n.rpdo.read()
            print(f"    ✓ Node {n.id}: Configuration loaded + PDOs read")

    def setup_402_all(self):
        for n in self.nodes:
            n.setup_402_state_machine()
            print(f"    ✓ Node {n.id}: DS402 state machine configured")

    def read_device_info_all(self):
        for n in self.nodes:
            device_name = n.sdo[0x1008].raw
            vendor_id = n.sdo[0x1018][1].raw
            print(f"    Node {n.id} Device Name: {device_name}")
            print(f"    Node {n.id} Vendor ID: 0x{vendor_id:08X}")

    def start_guarding_all(self, period_s: float = 0.01):
        for n in self.nodes:
            n.nmt.start_node_guarding(period_s)
            print(f"    ✓ Node {n.id}: Node guarding started ({int(period_s * 1000)}ms)")
        self._guarding_started = True

    def stop_guarding_all(self):
        if not self._guarding_started:
            return
        for n in self.nodes:
            try:
                n.nmt.stop_node_guarding()
            except:
                pass
        self._guarding_started = False

    # ---------------- DS402 flow ----------------

    def fault_reset_if_needed(self):
        if any(n.is_faulted() for n in self.nodes):
            print("    ! Fault detected, attempting reset...")
            for n in self.nodes:
                n.rpdo[1][self.ctl_variable].raw = 0x80
                self._safe_transmit_pdo(n.rpdo[1])
                time.sleep(0.1)
                n.rpdo[1][self.ctl_variable].raw = 0x00
                self._safe_transmit_pdo(n.rpdo[1])
                time.sleep(0.2)
                print(f"    ✓ Node {n.id}: Fault reset command sent")
        else:
            print("    ✓ No fault detected")

    def _wait_state_all(self, desired_state: str, timeout_s: float = 15.0):
        deadline = time.time() + timeout_s
        while True:
            if all(n.state == desired_state for n in self.nodes):
                return
            if time.time() > deadline:
                current = {n.id: n.state for n in self.nodes}
                raise TimeoutError(f"Timeout waiting for {desired_state}, current={current}")
            time.sleep(0.001)

    def enable_operation_all(self):
        for target_state in ['READY TO SWITCH ON', 'SWITCHED ON', 'OPERATION ENABLED']:
            for n in self.nodes:
                n.state = target_state
            self._wait_state_all(target_state, timeout_s=15.0)

        for n in self.nodes:
            print(f"Node Status {n.id}: {n.state}")

    def set_op_mode_all(self, mode: str = 'PROFILED VELOCITY'):
        for n in self.nodes:
            n.op_mode = mode
            print(f"    ✓ Node {n.id} Operation mode: {n.op_mode}")

    def set_velocity_profile_all(self,
                                 accel: int = 6553600,
                                 decel: int = 6553600,
                                 quick_stop: int = 2000000):
        for n in self.nodes:
            try:
                if 0x6083 in n.sdo.keys():
                    n.sdo[0x6083].raw = accel
                    print(f"    ✓ Node {n.id}: Profile acceleration set")
                if 0x6084 in n.sdo.keys():
                    n.sdo[0x6084].raw = decel
                    print(f"    ✓ Node {n.id}: Profile deceleration set")
                if 0x6085 in n.sdo.keys():
                    n.sdo[0x6085].raw = quick_stop
                    print(f"    ✓ Node {n.id}: Quick stop deceleration set")
            except Exception as e:
                print(f"    ! Node {n.id}: Warning - Could not set all parameters - {e}")

    # ---------------- TX robust helpers ----------------

    def _is_no_buffer_space(self, exc: Exception) -> bool:
        # handle python-can CanOperationError wrapping OSError 105
        msg = str(exc)
        if "No buffer space available" in msg:
            return True
        # sometimes OSError bubbles up
        if isinstance(exc, OSError) and exc.errno == errno.ENOBUFS:
            return True
        return False

    def _safe_transmit_pdo(self, pdo, retries: int = 30, backoff_s: float = 0.002):
        """
        Retry transmit when socketcan TX buffer is full.
        backoff is small; total wait ~ retries*backoff (e.g. 30*2ms=60ms)
        """
        last_exc = None
        for _ in range(retries):
            try:
                pdo.transmit()
                return
            except Exception as e:
                last_exc = e
                if self._is_no_buffer_space(e):
                    time.sleep(backoff_s)
                    continue
                raise
        # still failed
        raise last_exc

    # ---------------- velocity control ----------------

    def send_velocity_all(self,
                          target_vel: int,
                          ctrlword: int = 0x000F,
                          rpdo_no: int = 3,
                          retries: int = 30,
                          backoff_s: float = 0.002):
        """
        Minimal gap sending:
        - Fill RPDO values for all nodes
        - Transmit back-to-back
        - Each transmit is "safe" (retries on ENOBUFS)
        """
        for n in self.nodes:
            n.rpdo[rpdo_no][self.ctl_variable].raw = ctrlword
            n.rpdo[rpdo_no][self.velocity_variable].raw = target_vel

        for n in self.nodes:
            self._safe_transmit_pdo(n.rpdo[rpdo_no], retries=retries, backoff_s=backoff_s)


    def stop_all(self,
             rpdo_no: int = 3,
             tpdo_status_no: int = 1,
             tpdo_vel_no: int = 3,
             vel_threshold: int = 50,
             wait_s: float = 2.0):
        """
        Robust stop for Profiled Velocity:
        1) Send Halt + target_vel=0 (do NOT stop SYNC before this)
        2) Wait until actual velocity close to 0 (best-effort)
        3) Optionally disable op (hard stop output)
        4) Then stop SYNC/guarding to reduce traffic
        """
        # 1) PV halt stop: bit8=1 while staying enabled
        CW_HALT_ENABLE_OP = 0x010F
        self.send_velocity_all(target_vel=0, ctrlword=CW_HALT_ENABLE_OP, rpdo_no=rpdo_no, retries=80, backoff_s=0.003)

        # 2) wait actual velocity -> 0
        t0 = time.time()
        while time.time() - t0 < wait_s:
            all_ok = True
            for n in self.nodes:
                try:
                    # read newest status, then velocity
                    n.tpdo[tpdo_status_no].wait_for_reception(timeout=0.2)
                    actual_vel = n.tpdo[tpdo_vel_no][self.vel_variable].raw
                    if abs(int(actual_vel)) > vel_threshold:
                        all_ok = False
                except Exception:
                    # if feedback missing, don't block forever
                    all_ok = False
            if all_ok:
                break
            time.sleep(0.05)

        # 3) optional: disable operation (drive output off)
        # 如果你希望“停住后还保持使能”，注释掉这段
        CW_DISABLE_OP = 0x0007
        for n in self.nodes:
            try:
                n.rpdo[rpdo_no][self.ctl_variable].raw = CW_DISABLE_OP
                n.rpdo[rpdo_no][self.velocity_variable].raw = 0
                self._safe_transmit_pdo(n.rpdo[rpdo_no], retries=80, backoff_s=0.003)
            except Exception:
                pass

        # 4) now reduce traffic
        try:
            self.stop_guarding_all()
        except:
            pass
        try:
            self.stop_sync()
        except:
            pass

    def run_sequence(self,
                     velocity_sequence,
                     rpdo_no: int = 3,
                     tpdo_status_no: int = 1,
                     tpdo_vel_no: int = 3,
                     monitor_sleep_s: float = 0.02,
                     print_every_s: float = 0.10):
        """
        For each step:
        - Send command to ALL nodes first (min gap)
        - Monitor ALL nodes in the same duration window
        """
        self._require_net()
        last_print_t = 0.0

        for target_vel, duration, description in velocity_sequence:
            print(f"\n{description}")
            print(f"  Target: {target_vel:+5d} rpm | Duration: {duration:.1f}s")

            self.send_velocity_all(target_vel=target_vel, ctrlword=0x000F, rpdo_no=rpdo_no)

            start_t = time.time()
            while time.time() - start_t < duration:
                self.network.check()

                now = time.time()
                do_print = (now - last_print_t) >= print_every_s
                if do_print:
                    last_print_t = now

                for n in self.nodes:
                    try:
                        n.tpdo[tpdo_status_no].wait_for_reception(timeout=0.05)
                        if do_print:
                            actual_vel = n.tpdo[tpdo_vel_no][self.vel_variable].raw
                            statusword = n.tpdo[tpdo_status_no][self.status_variable].raw
                            target_val = n.rpdo[rpdo_no][self.velocity_variable].raw

                            print(
                                f"  Node {n.id} Target {target_val:+5d}, Actual {actual_vel:+5d} rpm | "
                                f"Status 0x{statusword:04X} | State {n.state}"
                            )
                    except Exception as e:
                        if do_print:
                            print(f"  ! Node {n.id} feedback error: {e}")

                time.sleep(monitor_sleep_s)

    # ---------------- bringup ----------------

    def bringup(self):
        self._require_net()

        print("\n[1] Setting NMT PRE-OPERATIONAL...")
        self.set_nmt_all('PRE-OPERATIONAL')
        for n in self.nodes:
            print(f"    ✓ Node {n.id} NMT: {n.nmt.state}")

        print("\n[4] Checking and clearing error log...")
        self.clear_error_log_all()

        print("\n[5] Configuring communication parameters...")
        self.set_comm_cycle_all(cycle_period_ms=10)

        print("\n[6] Starting SYNC transmission (100ms cycle)...")
        self.start_sync(period_s=0.1)

        print("\n[7] Loading node configuration...")
        self.load_config_all()

        print("\n[8] Setting up DS402 state machine...")
        self.setup_402_all()

        print("\n[9] Reading device information...")
        self.read_device_info_all()

        print("\n[12] Switching NMT OPERATIONAL & checking fault...")
        self.set_nmt_all('OPERATIONAL')
        for n in self.nodes:
            n.tpdo[1].wait_for_reception(timeout=1.0)

        self.fault_reset_if_needed()

        print("\n[10] Enabling operation...")
        self.enable_operation_all()

        print("\n[14] Setting operation mode to PROFILED VELOCITY...")
        self.set_op_mode_all('PROFILED VELOCITY')

        print("\n[15] Configuring velocity parameters...")
        self.set_velocity_profile_all()

        print("\n[16] Starting node guarding...")
        # 如果你加到3轴后总线更拥挤，可以把 0.01 调大到 0.05 降低流量
        self.start_guarding_all(period_s=0.01)

    def quick_stop_all(self, rpdo_no: int = 3):
        # 0x000B = enable voltage + switched on + quick stop active (bit2=0) + operation enable(bit3=1)
        CW_QUICK_STOP = 0x000B
        for n in self.nodes:
            n.rpdo[rpdo_no][self.ctl_variable].raw = CW_QUICK_STOP
            n.rpdo[rpdo_no][self.velocity_variable].raw = 0
            time.sleep(1)
         
def main():
    controller = None
    try:
        print("=" * 60)
        print("CANopen DS402 Velocity Control Demo (Class-based, robust stop)")
        print("=" * 60)

        eds_file_path = '/home/nvidia/github/robot_base_ctl/motor/moons/CANOPEN-EDS-MBDV-Servo-SingleAxis-V1.1.1.eds'

        controller = DS402MultiVelocityController(
            eds_file_path=eds_file_path,
            node_ids=[3,2,1],   # add more nodes here
            interface='socketcan',
            channel='can0',
            bitrate=50000
        )

        controller.connect()
        controller.bringup()

        print("\n[17] Starting velocity control demonstration...")
        print("=" * 60)
        print("VELOCITY CONTROL ACTIVE")
        print("Press Ctrl+C to stop")
        print("=" * 60)
        for n in controller.nodes:
            print(f"before run node {n.id} state: {n.state}")

        velocity_sequence = [
            (-327680, 1.0, "Increase to 65536 rpm"),
        ]

        controller.run_sequence(
            velocity_sequence=velocity_sequence,
            monitor_sleep_s=0.02,
            print_every_s=0.10
        )

        print("\n[18] Stopping motors...")
        controller.quick_stop_all()
        time.sleep(0.2)
        print("    ✓ Motors stopped")

        print("\n" + "=" * 60)
        print("Velocity control demo completed successfully!")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n\n! User interrupted - stopping...")

    except Exception as e:
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1] if exc_tb else "unknown"
        lineno = exc_tb.tb_lineno if exc_tb else -1
        print(f"\n! ERROR: {exc_type.__name__} in {fname}:{lineno}")
        print(f"  Message: {e}")
        traceback.print_exc()

    finally:
        if controller:
            controller.close()


if __name__ == "__main__":
    main()
