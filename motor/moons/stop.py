#!/usr/bin/env python3
"""
DS402 Emergency Stop Script (robust)
- Keeps OD strings EXACTLY (including spaces)
- Handles socketcan ENOBUFS (Errno 105) with retry/backoff
- Sends multi-stage stop: vel->0, quick stop, disable voltage
"""

import time
import errno
import logging
import canopen

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logging.getLogger('can.interfaces.socketcan').setLevel(logging.INFO)

# ---- KEEP ORIGINAL OD STRINGS EXACTLY (DO NOT CHANGE) ----
VEL_VARIABLE = 'Velocity value calculated'
STATUS_VARIABLE = 'Status word'
CTL_VARIABLE = 'Control word'
TARGET_VEL_VARIABLE = 'Target velocity'
# ---------------------------------------------------------


def is_no_buffer_space(exc: Exception) -> bool:
    msg = str(exc)
    if "No buffer space available" in msg:
        return True
    if isinstance(exc, OSError) and exc.errno == errno.ENOBUFS:
        return True
    return False


def safe_transmit(pdo, retries: int = 80, backoff_s: float = 0.003):
    """
    Retry transmit when socketcan TX buffer is full.
    """
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


def set_rpdo3_and_send(node, controlword: int, target_vel: int, rpdo_no: int = 3):
    """
    Uses RPDO3 which (in your map) contains:
    - 'Control word'
    - 'Target velocity'
    """
    node.rpdo[rpdo_no][CTL_VARIABLE].raw = controlword
    node.rpdo[rpdo_no][TARGET_VEL_VARIABLE].raw = target_vel
    safe_transmit(node.rpdo[rpdo_no])


def stop_one_node(node, rpdo_no: int = 3):
    """
    Three-stage stop:
    1) keep enabled, target velocity -> 0 (smooth decel if profiled velocity)
    2) quick stop (common CW=0x000B) with target 0
    3) disable voltage (CW=0x0000) with target 0
    """
    print(f"\n[STOP] Node {node.id}")

    # Stage 1: Target velocity = 0, keep enabled
    try:
        set_rpdo3_and_send(node, controlword=0x000F, target_vel=0, rpdo_no=rpdo_no)
        print("  ✓ Stage1: Target velocity=0 (0x000F)")
    except Exception as e:
        print(f"  ! Stage1 failed: {e}")

    time.sleep(0.10)

    # Stage 2: Quick Stop (common)
    try:
        set_rpdo3_and_send(node, controlword=0x000B, target_vel=0, rpdo_no=rpdo_no)
        print("  ✓ Stage2: Quick Stop (0x000B)")
    except Exception as e:
        print(f"  ! Stage2 failed: {e}")

    time.sleep(0.10)

    # Stage 3: Disable Voltage (harder stop)
    try:
        set_rpdo3_and_send(node, controlword=0x0000, target_vel=0, rpdo_no=rpdo_no)
        print("  ✓ Stage3: Disable Voltage (0x0000)")
    except Exception as e:
        print(f"  ! Stage3 failed: {e}")

    # Optional: read back some status/velocity once (best effort)
    try:
        node.tpdo[1].wait_for_reception(timeout=0.2)
        v = node.tpdo[3][VEL_VARIABLE].raw
        sw = node.tpdo[1][STATUS_VARIABLE].raw
        print(f"  Feedback: Actual vel={v:+d}, Statusword=0x{sw:04X}, State={node.state}")
    except Exception:
        pass


def main():
    # === CONFIG YOU MAY EDIT ===
    eds_file_path = '/home/nvidia/github/robot_base_ctl/motor/moons/CANOPEN-EDS-MBDV-Servo-SingleAxis-V1.1.1.eds'
    interface = 'socketcan'
    channel = 'can0'
    bitrate = 1000000

    # Put the nodes you want to stop:
    node_ids = [1, 2, 3]
    # ===========================

    net = None
    nodes = []

    try:
        print("=" * 60)
        print("DS402 Emergency Stop Script")
        print("=" * 60)

        net = canopen.Network()
        net.connect(interface=interface, channel=channel, bitrate=bitrate)
        print(f"Connected to CAN interface: {channel}")

        for nid in node_ids:
            n = canopen.BaseNode402(nid, eds_file_path)
            net.add_node(n)
            n.sdo.RESPONSE_TIMEOUT = 3.0
            nodes.append(n)

        # reduce periodic traffic from THIS process (if any)
        try:
            net.sync.stop()
        except:
            pass

        # Ensure OPERATIONAL so PDO comms are allowed
        for n in nodes:
            try:
                n.nmt.state = 'OPERATIONAL'
            except:
                pass

        # Read PDO mapping (needed so 'Control word' etc are valid)
        for n in nodes:
            try:
                n.load_configuration()
            except:
                pass
            try:
                n.tpdo.read()
                n.rpdo.read()
            except:
                pass

        # Stop one by one, but each stop is robust and fast
        for n in nodes:
            stop_one_node(n, rpdo_no=3)

        # After stop, put to PRE-OP (optional)
        for n in nodes:
            try:
                n.nmt.state = 'PRE-OPERATIONAL'
            except:
                pass

        print("\n✓ Stop sequence done.")

    finally:
        if net:
            try:
                try:
                    net.sync.stop()
                except:
                    pass
                net.disconnect()
                print("✓ Network disconnected.")
            except Exception as e:
                print(f"! Disconnect error: {e}")


if __name__ == "__main__":
    main()
