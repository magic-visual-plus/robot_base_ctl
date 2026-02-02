"""
CANopen DS402 Velocity Control Demo

This example demonstrates velocity mode control for a DS402 motor drive.
It shows how to:
- Configure the node for velocity control
- Set target velocities
- Monitor actual velocity feedback
- Execute velocity ramps and direction changes
"""

import os
import sys
import time
import traceback
import math

import canopen

import logging

# 启用 socketcan 模块的详细日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

# 设置can日志针对性启用收发日志
logging.getLogger('can.interfaces.socketcan').setLevel(logging.WARN)
# logging.getLogger('can.interfaces.socketcan.tx').setLevel(logging.INFO)
# logging.getLogger('can.interfaces.socketcan.rx').setLevel(logging.INFO)



def main():
    network = None

    try:
        print("="*60)
        print("CANopen DS402 Velocity Control Demo")
        print("="*60)

        network = canopen.Network()

        # Connect to the CAN bus
        # eds_file_path = '/opt/project/ros2_canopen_ws/test/CANOPEN-EDS-MBDV-Servo-SingleAxis-V1.1.1.eds'
        eds_file_path = '/opt/project/robot_base_ctl/motor/moons/CANOPEN-EDS-MBDV-Servo-DulAxes-V1.0.eds'

        # Add some nodes with corresponding Object Dictionaries
        node = canopen.BaseNode402(2, eds_file_path)
        # node_2 = canopen.BaseNode402(2, eds_file_path)
        network.add_node(node)
        # network.add_node(node_2)

        node.sdo.RESPONSE_TIMEOUT = 2.0

        try:
            network.connect(interface='socketcan', channel='can0', bitrate=1000000)
            print(f"Connected to CAN interface: can1")
        except Exception as e:
            print(f"Failed to connect to CAN bus: {e}")
            sys.exit(1)

        network.check()

        print(f'before set nmt state {node.nmt.state}')
        start_op = time.time()
        # Reset network
        # node.nmt.state = 'RESET COMMUNICATION'
        # node.nmt.state = 'OPERATIONAL'
        node.nmt.state = 'PRE-OPERATIONAL'
        print(f"✓after set Node nmt state: {node.nmt.state}")
        end_op = time.time()
        print(f"Node booted up in {end_op - start_op:.2f} seconds")

        # Step 4: Clear Error Log
        print("\n[4] Checking and clearing error log...")
        error_log = node.sdo[0x1003]
        error_count = len(error_log.values())
        if error_count > 0:
            print(f"    ! Found {error_count} errors in log")
            for i, error in enumerate(error_log.values(), 1):
                print(f"      Error {i}: 0x{error.raw:08X}")
        node.sdo[0x1003][0].raw = 0  # Clear error log
        print("    ✓ Error log cleared")

        # Step 5: Configure Communication Parameters
        print("\n[5] Configuring communication parameters...")
        node.sdo[0x1006].raw = 100        # Communication cycle period
        # node.sdo[0x100c].raw = 100       # Guard time
        # node.sdo[0x100d].raw = 3         # Life time factor
        # node.sdo[0x1014].raw = 163       # COB-ID EMCY
        # node.sdo[0x1003][0].raw = 0   # Clear error log
        print("    ✓ Communication parameters set")

        # Step 6: Start SYNC (降低频率减少总线负载)
        print("\n[6] Starting SYNC transmission (200ms cycle)...")
        network.sync.start(0.2)  # 从100ms改为200ms，降低频率
        print("    ✓ SYNC started")

        # Step 7: Load Configuration
        print("\n[7] Loading node configuration...")
        node.load_configuration()
        print("    ✓ Configuration loaded")
        node.tpdo.read()
        node.rpdo.read()
        # Step 8: Setup DS402 State Machine
        print("\n[8] Setting up DS402 state machine...")
        node.setup_402_state_machine()
        print("    ✓ State machine configured")

        # Step 9: Read Device Information
        print("\n[9] Reading device information...")
        device_name = node.sdo[0x1008].raw
        vendor_id = node.sdo[0x1018][1].raw
        print(f"    Device Name: {device_name}")
        print(f"    Vendor ID: 0x{vendor_id:08X}")
        
        # Read encoder resolution to understand velocity units
        try:
            encoder_resolution = node.sdo[0x2A0B].raw
            print(f"    Encoder Resolution: {encoder_resolution} inc/rev")
        except Exception as e:
            encoder_resolution = None
            print(f"    ! Could not read encoder resolution: {e}")
        vel_variable = 'Velocity value calculated'
        status_variable = 'Status word'
        ctl_variable = 'Control word'
        velocity_variable = 'Target velocity'
         

        # 禁用不需要的RPDO (保留RPDO3用于速度控制)
        node.rpdo[2].enabled = False
        # node.rpdo[3].enabled = False  # 不禁用，用于发送速度命令
        node.rpdo[4].enabled = False
        
        
        node.rpdo.save()
        print("    ✓ RPDO1 configured: Controlword + Target Velocity")

        # Step 12: Reset from Fault (if needed)
        print("\n[12] Checking fault status...")
        node.nmt.state = 'OPERATIONAL'
        node.tpdo[1].wait_for_reception(timeout=1.0)
        if node.is_faulted():
            print("    ! Node is in FAULT state, attempting reset...")
            # Send fault reset command
            node.rpdo[1]['Control word'].raw = 0x80
            node.rpdo[1].transmit()
            time.sleep(0.1)
            node.rpdo[1]['Control word'].raw = 0x00
            node.rpdo[1].transmit()
            time.sleep(0.2)
            print("    ✓ Fault reset command sent")
        else:
            print("    ✓ No fault detected")

        print(f"✓ Current state before: {node.state}")
        print(f"✓after set Node nmt state: {node.nmt.state}")

        node.state = 'READY TO SWITCH ON'
        node.state = 'SWITCHED ON'

        # node.rpdo.export('database.dbc')

        # -----------------------------------------------------------------------------------------

        print('Node booted up')

        timeout = time.time() + 15
        node.state = 'READY TO SWITCH ON'
        while node.state != 'READY TO SWITCH ON':
            if time.time() > timeout:
                raise Exception('Timeout when trying to change state')
            time.sleep(0.001)

        timeout = time.time() + 15
        node.state = 'SWITCHED ON'
        while node.state != 'SWITCHED ON':
            if time.time() > timeout:
                raise Exception('Timeout when trying to change state')
            time.sleep(0.001)

        timeout = time.time() + 15
        node.state = 'OPERATION ENABLED'
        while node.state != 'OPERATION ENABLED':
            if time.time() > timeout:
                raise Exception('Timeout when trying to change state')
            time.sleep(0.001)

        print(f'Node Status {node.state}')

        # exit(1)
        # Step 14: Set Velocity Mode
        print("\n[14] Setting operation mode to PROFILED VELOCITY...")
        # Options: 'VELOCITY' or 'PROFILED VELOCITY'
        # PROFILED VELOCITY uses acceleration/deceleration ramps
        node.op_mode = 'PROFILED VELOCITY'
        print(f"    ✓ Operation mode: {node.op_mode}")

        # Step 15: Configure Velocity Parameters (optional)
        print("\n[15] Configuring velocity parameters...")
        # These are typical DS402 velocity mode objects
        try:
            # Read current Max profile speed (0x607F)
            if 0x607F in node.sdo.keys():
                current_max_speed = node.sdo[0x607F].raw
                print(f"    Current Max profile speed: {current_max_speed}")
                # Set a higher max speed if needed
                # node.sdo[0x607F].raw = 5000000  # Set high max speed
                # print(f"    ✓ Max profile speed set to: 5000000")

            # Profile acceleration (0x6083) - units depend on drive configuration
            if 0x6083 in node.sdo.keys():
                node.sdo[0x6083].raw = 1000000  # Acceleration
                print("    ✓ Profile acceleration set")

            # Profile deceleration (0x6084)
            if 0x6084 in node.sdo.keys():
                node.sdo[0x6084].raw = 1000000  # Deceleration
                print("    ✓ Profile deceleration set")

            # Quick stop deceleration (0x6085)
            if 0x6085 in node.sdo.keys():
                node.sdo[0x6085].raw = 2000000  # Quick stop deceleration
                print("    ✓ Quick stop deceleration set")

        except Exception as e:
            print(f"    ! Warning: Could not set all parameters - {e}")

        # Step 16: Start Node Guarding (调整到合理周期)
        print("\n[16] Starting node guarding...")
        node.nmt.start_node_guarding(1.0)  # 从10ms改为1000ms，大幅降低心跳频率
        print("    ✓ Node guarding started (1000ms)")

        # Step 17: Velocity Control Loop
        print("\n[17] Starting velocity control demonstration...")
        print("="*60)
        print("VELOCITY CONTROL ACTIVE")
        print("Press Ctrl+C to stop")
        print("="*60)
        print(f"before run node state: {node.state}")
        # Define velocity profile sequence
        velocity_sequence = [
            # (0, 2.0, "Initial position - zero velocity"),
            (12000, 30.0, "Accelerate to 1000 rpm"),
            # (6000, 40.0, "Increase to 6000 rpm"),
            # (1500, 2.0, "Decelerate to 1500 rpm"),
            # (5000, 3.0, "Slow down to 500 rpm"),
            # (0, 2.0, "Stop motor"),
            # (-1000, 3.0, "Reverse to -1000 rpm"),
            # (0, 2.0, "Return to zero"),
            # (3000, 2.0, "Fast forward 3000 rpm"),
            (0, 10.0, "Final stop"),
        ]

        for target_vel, duration, description in velocity_sequence:
            print(f"\n{description}")
            print(f"  Target: {target_vel:+5d} rpm | Duration: {duration:.1f}s")

            # Set target velocity via RPDO (using object dictionary index 0x60FF)
            # CRITICAL: Must send BOTH Controlword and Target Velocity in SAME RPDO
            node.rpdo[3][ctl_variable].raw = 0x000F  # OPERATION ENABLED command
            node.rpdo[3][velocity_variable].raw = target_vel
            node.rpdo[3].transmit()  # Atomic transmission

            # Monitor actual velocity for the duration
            start_time = time.time()
            sample_count = 0

            while time.time() - start_time < duration:
                try:
                    network.check()

                    # Wait for TPDO with velocity feedback
                    node.tpdo[1].wait_for_reception(timeout=0.5)

                    # Read actual velocity from TPDO
                    actual_vel = node.tpdo[3][vel_variable].raw
                    statusword = node.tpdo[1][status_variable].raw

                    # Also read via SDO for comparison
                    actual_vel_sdo = node.sdo[0x606C].raw
                    target_vel_sdo = node.sdo[0x60FF].raw

                    target_val = node.rpdo[3][velocity_variable].raw

                    # Print status every 10 samples (~100ms)
                    # sample_count += 1
                    # if sample_count % 10 == 0:
                    print(f" Target vel {target_val}, TPDO Actual: {actual_vel:+6d}, SDO Actual: {actual_vel_sdo:+6d}, SDO Target: {target_vel_sdo:+6d} | "
                              f"Status: 0x{statusword:04X} | "
                              f"State: {node.state}")

                    time.sleep(0.1)

                except Exception as e:
                    print(f"  ! Error reading feedback: {e}")
                    break

        # Step 18: Final Stop
        print("\n[18] Stopping motor...")
        node.rpdo[3][ctl_variable].raw = 0x000F  # Maintain OPERATION ENABLED
        node.rpdo[3][velocity_variable].raw = 0
        node.rpdo[3].transmit()  # Atomic transmission
        time.sleep(1.0)
        print("    ✓ Motor stopped")

        print("\n" + "="*60)
        print("Velocity control demo completed successfully!")
        print("="*60)

    except KeyboardInterrupt:
        print("\n\n! User interrupted - stopping...")

    except Exception as e:
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        print(f"\n! ERROR: {exc_type.__name__} in {fname}:{exc_tb.tb_lineno}")
        print(f"  Message: {e}")
        traceback.print_exc()

    finally:
        # Cleanup
        print("\n[CLEANUP] Shutting down...")
        if network:
            try:
                for node_id in network:
                    node = network[node_id]
                    # Stop motor
                    try:
                        node.rpdo[1][0x60FF].raw = 0
                        node.rpdo[1].transmit()
                    except:
                        pass
                    # Set to PRE-OPERATIONAL
                    node.nmt.state = 'PRE-OPERATIONAL'
                    node.nmt.stop_node_guarding()
                    print(f"  ✓ Node {node_id} stopped")

                network.sync.stop()
                network.disconnect()
                print("  ✓ Network disconnected")

            except Exception as e:
                print(f"  ! Cleanup error: {e}")

        print("\nGoodbye!")


if __name__ == "__main__":
    main()