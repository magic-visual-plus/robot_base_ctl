#!/usr/bin/env python3
"""
CAN connection test script.
Checks if CAN interface is up, and attempts to communicate with a CANopen node.
"""

import sys
import time
import subprocess
import can
import canopen
from canopen import Network, Node


def check_interface(interface="can0"):
    """Check if CAN interface is up and has bitrate set."""
    try:
        output = subprocess.check_output(["ip", "link", "show", interface], stderr=subprocess.STDOUT)
        output = output.decode()
        if "UP" in output:
            print(f"[OK] Interface {interface} is up.")
            # Try to get bitrate
            try:
                bitrate_output = subprocess.check_output(["ip", "-d", "link", "show", interface])
                if "bitrate" in bitrate_output.decode():
                    print(f"      {bitrate_output.decode().split('bitrate')[1].split()[0]}")
            except:
                pass
            return True
        else:
            print(f"[FAIL] Interface {interface} is down.")
            return False
    except subprocess.CalledProcessError:
        print(f"[FAIL] Interface {interface} does not exist.")
        return False
    except Exception as e:
        print(f"[FAIL] Error checking interface: {e}")
        return False


def test_can_bus(interface="can0", node_id=1, timeout=2.0):
    """
    Try to send a CANopen NMT start command and listen for any message from the node.
    Returns True if at least one message from node_id is received.
    """
    print(f"\nTesting CAN bus communication with node {node_id} on {interface}...")
    try:
        bus = can.interface.Bus(channel=interface, bustype="socketcan")
        # Send NMT start command (0x01) to node_id
        # NMT command format: 0x00 + node_id (if broadcast, node_id=0)
        # For specific node: cob_id = 0x700 + node_id? Actually NMT is on 0x000.
        # We'll use canopen library to send the command.
        network = Network()
        node = Node(node_id, None)  # No EDS needed for this test
        network.add_node(node)
        network.connect(interface="socketcan", channel=interface, bitrate=1000000)
        print("  Sending NMT start command...")
        node.nmt.send_command(0x01)  # Start node
        # Wait for some messages
        start = time.time()
        messages_received = 0
        print(f"  Listening for messages from node {node_id} for {timeout} seconds...")
        while time.time() - start < timeout:
            msg = bus.recv(timeout=0.5)
            if msg:
                # Check if message COB-ID is for node_id (TPDO/SDO usually have base 0x180+node_id, etc.)
                # For a quick test, just print all messages.
                print(f"    Received: {msg}")
                # Consider any message as a sign of activity.
                messages_received += 1
                # If we get a message, break early.
                break
        network.disconnect()
        bus.shutdown()
        if messages_received > 0:
            print(f"[OK] Received {messages_received} message(s) from the bus.")
            return True
        else:
            print("[FAIL] No messages received from the bus.")
            return False
    except Exception as e:
        print(f"[FAIL] Exception during CAN test: {e}")
        return False


def main():
    interface = "can0"
    node_id = 1

    print("=" * 60)
    print("CANopen Connection Test")
    print("=" * 60)

    # Step 1: Check interface
    if not check_interface(interface):
        print("\nPlease ensure the CAN interface is configured correctly:")
        print("  sudo ip link set can0 type can bitrate 1000000")
        print("  sudo ip link set can0 up")
        sys.exit(1)

    # Step 2: Test bus communication
    if test_can_bus(interface, node_id):
        print("\n[SUCCESS] CAN communication seems to be working.")
        print("  If you still have problems, check the node's EDS file and node ID.")
    else:
        print("\n[WARNING] No CAN messages detected.")
        print("  Possible issues:")
        print("   - Node is not powered on or not connected.")
        print("   - Node ID mismatch (current ID = 1).")
        print("   - Bitrate mismatch (current = 1 Mbps).")
        print("   - Physical wiring problem.")
        print("\nYou can also run 'candump can0' in another terminal to monitor raw traffic.")


if __name__ == "__main__":
    main()