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
logging.getLogger('can.interfaces.socketcan').setLevel(logging.INFO)
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
        eds_file_path = '/home/nvidia/github/robot_base_ctl/motor/moons/CANOPEN-EDS-MBDV-Servo-SingleAxis-V1.1.1.eds'

        # Add some nodes with corresponding Object Dictionaries
        node = canopen.BaseNode402(2, eds_file_path)
        node_2 = canopen.BaseNode402(3, eds_file_path)
        node_3 = canopen.BaseNode402(1, eds_file_path)
        network.add_node(node)
        network.add_node(node_2)
        network.add_node(node_3)
        
        node.sdo.RESPONSE_TIMEOUT = 5.0 
        node_2.sdo.RESPONSE_TIMEOUT = 5.0
        node_3.sdo.RESPONSE_TIMEOUT = 5.0 
        
        try:
            network.connect(interface='socketcan', channel='can0', bitrate=500000)
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
        node_2.nmt.state = 'PRE-OPERATIONAL'
        node_3.nmt.state = 'PRE-OPERATIONAL'
        print(f"✓after set Node nmt state: {node.nmt.state}")
        print(f"✓after set Node nmt state: {node_2.nmt.state}")
        print(f"✓after set Node nmt state: {node_3.nmt.state}")
        end_op = time.time()
        print(f"Node booted up in {end_op - start_op:.2f} seconds")
        
        # Step 4: Clear Error Log
        print("\n[4] Checking and clearing error log...")
        error_log = node.sdo[0x1003]
        error_log_2 = node_2.sdo[0x1003]
        error_log_3 = node_3.sdo[0x1003]
        error_count = len(error_log.values())
        error_count_2 = len(error_log_2.values())
        error_count_3 = len(error_log_3.values())
        if error_count > 0:
            print(f"    ! Found {error_count} errors in log")
            for i, error in enumerate(error_log.values(), 1):
                print(f"      Error {i}: 0x{error.raw:08X}")
        node.sdo[0x1003][0].raw = 0  # Clear error log
        node_2.sdo[0x1003][0].raw = 0  # Clear error log
        node_3.sdo[0x1003][0].raw = 0  # Clear error log
        print("    ✓ Error log cleared")
        if error_count_2 > 0:
            print(f"    ! Found {error_count_2} errors in log")
            for i, error in enumerate(error_log_2.values(), 1):
                print(f"      Error {i}: 0x{error.raw:08X}")
        print("    ✓ Error log cleared")
        if error_count_3 > 0:
            print(f"    ! Found {error_count_3} errors in log")
            for i, error in enumerate(error_log_3.values(), 1):
                print(f"      Error {i}: 0x{error.raw:08X}")
        print("    ✓ Error log cleared")
        
        # Step 5: Configure Communication Parameters
        print("\n[5] Configuring communication parameters...")
        node.sdo[0x1006].raw = 50        # Communication cycle period
        # node.sdo[0x100c].raw = 100       # Guard time
        # node.sdo[0x100d].raw = 3         # Life time factor
        # node.sdo[0x1014].raw = 163       # COB-ID EMCY
        # node.sdo[0x1003][0].raw = 0   # Clear error log
        print("    ✓ Communication parameters set")
        node_2.sdo[0x1006].raw = 50        # Communication cycle period
        node_3.sdo[0x1006].raw = 50        # Communication cycle period

        # Step 6: Start SYNC
        print("\n[6] Starting SYNC transmission (100ms cycle)...")
        network.sync.start(0.1)
        print("    ✓ SYNC started")
        
        # Step 7: Load Configuration
        print("\n[7] Loading node configuration...")
        node.load_configuration()
        node_2.load_configuration()
        node_3.load_configuration()
        print("    ✓ Configuration loaded")
        node.tpdo.read()
        node.rpdo.read()
        node_2.tpdo.read()
        node_2.rpdo.read()
        node_3.tpdo.read()
        node_3.rpdo.read()
        # Step 8: Setup DS402 State Machine
        print("\n[8] Setting up DS402 state machine...")
        node.setup_402_state_machine()
        node_2.setup_402_state_machine()
        node_3.setup_402_state_machine()
        print("    ✓ State machine configured")
        
        # disable unused tpdo
        node.tpdo[2].enabled = False
        node.tpdo[3].enabled = False
        node.tpdo.save()
        
        # disable unused tpdo
        node_2.tpdo[2].enabled = False
        node_2.tpdo[3].enabled = False
        node_2.tpdo.save()
        
         # disable unused tpdo
        node_3.tpdo[2].enabled = False
        node_3.tpdo[3].enabled = False
        node_3.tpdo.save()
        
        # Step 9: Read Device Information
        print("\n[9] Reading device information...")
        device_name = node.sdo[0x1008].raw
        vendor_id = node.sdo[0x1018][1].raw
        device_name_2 = node_2.sdo[0x1008].raw
        vendor_id_2 = node_2.sdo[0x1018][1].raw
        device_name_3 = node_3.sdo[0x1008].raw
        vendor_id_3 = node_3.sdo[0x1018][1].raw
        print(f"    Device Name: {device_name}")
        print(f"    Vendor ID: 0x{vendor_id:08X}")
        print(f"    Device Name: {device_name_2}")
        print(f"    Vendor ID: 0x{vendor_id_2:08X}")
        print(f"    Device Name: {device_name_3}")
        print(f"    Vendor ID: 0x{vendor_id_3:08X}")
        vel_variable = 'Velocity value calculated'
        pos_variable = 'Position value calculated'
        status_variable = 'Status word'
        ctl_variable = 'Control word'
        velocity_variable = 'Target velocity'
        
        node.tpdo.read()
         
        print("    ✓ RPDO1 configured: Controlword + Mode + Target Velocity")
        
        # Step 12: Reset from Fault (if needed)
        print("\n[12] Checking fault status...")
        node.nmt.state = 'OPERATIONAL'
        node_2.nmt.state = 'OPERATIONAL'
        node_3.nmt.state = 'OPERATIONAL'
        node.tpdo[1].wait_for_reception(timeout=1.0)
        node_2.tpdo[1].wait_for_reception(timeout=1.0)
        node_3.tpdo[1].wait_for_reception(timeout=1.0)
        if node.is_faulted() or node_2.is_faulted() or node_3.is_faulted():
            print("    ! Node is in FAULT state, attempting reset...")
            # Send fault reset command
            node.rpdo[1]['Controlword'].raw = 0x80
            node.rpdo[1].transmit()
            time.sleep(0.1)
            node.rpdo[1]['Controlword'].raw = 0x00
            node.rpdo[1].transmit()
            time.sleep(0.2)
            print("    ✓ Fault reset command sent")
            node_2.rpdo[1]['Controlword'].raw = 0x80
            node_2.rpdo[1].transmit()
            time.sleep(0.1)
            node_2.rpdo[1]['Controlword'].raw = 0x00
            node_2.rpdo[1].transmit()
            time.sleep(0.2)
            print("    ✓ Fault reset command sent")
            node_3.rpdo[1]['Controlword'].raw = 0x80
            node_3.rpdo[1].transmit()
            time.sleep(0.1)
            node_3.rpdo[1]['Controlword'].raw = 0x00
            node_3.rpdo[1].transmit()
            time.sleep(0.2)
            print("    ✓ Fault reset command sent")
        else:
            print("    ✓ No fault detected")
        
        print(f"✓ Current state before: {node.state}")
        print(f"✓after set Node nmt state: {node.nmt.state}")
        
        node.state = 'READY TO SWITCH ON'
        node.state = 'SWITCHED ON'
        node_2.state = 'READY TO SWITCH ON'
        node_2.state = 'SWITCHED ON'
        node_3.state = 'READY TO SWITCH ON'
        node_3.state = 'SWITCHED ON'    
        # node.rpdo.export('database.dbc')

        # -----------------------------------------------------------------------------------------

        print('Node booted up')

        timeout = time.time() + 15
        node.state = 'READY TO SWITCH ON'
        node_2.state = 'READY TO SWITCH ON'
        node_3.state = 'READY TO SWITCH ON'
        while node.state != 'READY TO SWITCH ON' or node_2.state != 'READY TO SWITCH ON' or node_3.state != 'READY TO SWITCH ON':
            if time.time() > timeout:
                raise Exception('Timeout when trying to change state')
            time.sleep(0.001)

        timeout = time.time() + 15
        node.state = 'SWITCHED ON'
        node_2.state = 'SWITCHED ON'
        node_3.state = 'SWITCHED ON'
        while node.state != 'SWITCHED ON' or node_2.state != 'SWITCHED ON' or node_3.state != 'SWITCHED ON':
            if time.time() > timeout:
                raise Exception('Timeout when trying to change state')
            time.sleep(0.001)

        timeout = time.time() + 15
        node.state = 'OPERATION ENABLED'
        node_2.state = 'OPERATION ENABLED'
        node_3.state = 'OPERATION ENABLED'
        while node.state != 'OPERATION ENABLED' or node_2.state != 'OPERATION ENABLED' or node_3.state != 'OPERATION ENABLED':
            if time.time() > timeout:
                raise Exception('Timeout when trying to change state')
            time.sleep(0.001)

        print(f'Node Status {node.state}')
        print(f'Node Status {node_2.state}')
        print(f'Node Status {node_3.state}')
        # exit(1)
        # Step 14: Set Velocity Mode
        print("\n[14] Setting operation mode to PROFILED VELOCITY...")
        # Options: 'VELOCITY' or 'PROFILED VELOCITY'
        # PROFILED VELOCITY uses acceleration/deceleration ramps
        node.op_mode = 'PROFILED VELOCITY'
        node_2.op_mode = 'PROFILED VELOCITY'
        node_3.op_mode = 'PROFILED VELOCITY'
        print(f"    ✓ Operation mode: {node.op_mode}")
        print(f"    ✓ Operation mode: {node_2.op_mode}")
        print(f"    ✓ Operation mode: {node_3.op_mode}")
        # Step 15: Configure Velocity Parameters (optional)
        print("\n[15] Configuring velocity parameters...")
        # These are typical DS402 velocity mode objects
        try:
            # Profile acceleration (0x6083) - units depend on drive configuration
            if 0x6083 in node.sdo.keys():
                node.sdo[0x6083].raw = 6553600  # Acceleration
                print("    ✓ Profile acceleration set")
            
            # Profile deceleration (0x6084)
            if 0x6084 in node.sdo.keys():
                node.sdo[0x6084].raw = 6553600  # Deceleration
                print("    ✓ Profile deceleration set")
            
            # Quick stop deceleration (0x6085)
            if 0x6085 in node.sdo.keys():
                node.sdo[0x6085].raw = 20000000  # Quick stop deceleration
                print("    ✓ Quick stop deceleration set")

            if 0x6083 in node_2.sdo.keys():
                node_2.sdo[0x6083].raw = 6553600  # Acceleration
                print("    ✓ Profile acceleration set")
            if 0x6084 in node_2.sdo.keys():
                node_2.sdo[0x6084].raw = 6553600  # Deceleration
                print("    ✓ Profile deceleration set")
            if 0x6085 in node_2.sdo.keys():
                node_2.sdo[0x6085].raw = 20000000  # Quick stop deceleration
                print("    ✓ Quick stop deceleration set")

            if 0x6083 in node_3.sdo.keys():
                node_3.sdo[0x6083].raw = 6553600  # Acceleration
                print("    ✓ Profile acceleration set")
            if 0x6084 in node_3.sdo.keys():
                node_3.sdo[0x6084].raw = 6553600  # Deceleration
                print("    ✓ Profile deceleration set")
            if 0x6085 in node_3.sdo.keys():
                node_3.sdo[0x6085].raw = 20000000  # Quick stop deceleration
                print("    ✓ Quick stop deceleration set")
                
        except Exception as e:
            print(f"    ! Warning: Could not set all parameters - {e}")
        
        # Step 16: Start Node Guarding
        print("\n[16] Starting node guarding...")
        node.nmt.start_node_guarding(1)
        print("    ✓ Node guarding started (10ms)")
        node_2.nmt.start_node_guarding(1)
        print("    ✓ Node guarding started (10ms)")
        node_3.nmt.start_node_guarding(1)
        print("    ✓ Node guarding started (10ms)")    
        # Step 17: Velocity Control Loop
        print("\n[17] Starting velocity control demonstration...")
        print("="*60)
        print("VELOCITY CONTROL ACTIVE")
        print("Press Ctrl+C to stop")
        print("="*60)
        print(f"before run node state: {node.state}")
        print(f"before run node state: {node_2.state}")
        print(f"before run node state: {node_3.state}")
        # Define velocity profile sequence
        velocity_sequence = [
            # (0, 2.0, "Initial position - zero velocity"),
            
            (-327680, 10.0, "Increase to 65536 rpm"),
             
        ]
        velocity_sequence_2 = [
            # (0, 2.0, "Initial position - zero velocity"),
            
            (-327680, 20.0, "Increase to 65536 rpm"),
             
        ]
        velocity_sequence_3 = [
            # (0, 2.0, "Initial position - zero velocity"),
            
            (-327680, 30.0, "Increase to 65536 rpm"),
             
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
                    
                    # Read actual velocity
                    actual_vel = node.tpdo[4][vel_variable].raw
                    actual_pos = node.tpdo[4][pos_variable].raw

                    statusword = node.tpdo[1][status_variable].raw
                    
                    target_val = node.rpdo[3][velocity_variable].raw
                    
                    # Print status every 10 samples (~100ms)
                    # sample_count += 1
                    # if sample_count % 10 == 0:
                    print(f" Target vel {target_val},  Actual: {actual_vel:+5d} rpm | Actual pos: {actual_pos:+5d} | "
                              f"Status: 0x{statusword:04X} | "
                              f"State: {node.state}")
                    
                    time.sleep(0.1)
                    
                except Exception as e:
                    print(f"  ! Error reading feedback: {e}")
                    break
        
        for target_vel, duration, description in velocity_sequence_2:
            print(f"\n{description}")
            print(f"  Target: {target_vel:+5d} rpm | Duration: {duration:.1f}s")
            
            # Set target velocity via RPDO (using object dictionary index 0x60FF)
            # CRITICAL: Must send BOTH Controlword and Target Velocity in SAME RPDO
            node_2.rpdo[3][ctl_variable].raw = 0x000F  # OPERATION ENABLED command
            node_2.rpdo[3][velocity_variable].raw = target_vel
            node_2.rpdo[3].transmit()  # Atomic transmission
            
            # Monitor actual velocity for the duration
            start_time = time.time()
            sample_count = 0
            
            while time.time() - start_time < duration:
                try:
                    network.check()
                    
                    # Wait for TPDO with velocity feedback
                    node_2.tpdo[1].wait_for_reception(timeout=0.5)
                    
                    # Read actual velocity
                    actual_vel = node_2.tpdo[4][vel_variable].raw
                    actual_pos = node_2.tpdo[4][pos_variable].raw
                    statusword = node_2.tpdo[1][status_variable].raw
                    
                    target_val = node_2.rpdo[3][velocity_variable].raw
                    
                    # Print status every 10 samples (~100ms)
                    # sample_count += 1
                    # if sample_count % 10 == 0:
                    print(f" Target vel {target_val},  Actual: {actual_vel:+5d} rpm | Actual pos: {actual_pos:+5d} | "
                              f"Status: 0x{statusword:04X} | "
                              f"State: {node_2.state}")
                    
                    time.sleep(0.1)
                    
                except Exception as e:
                    print(f"  ! Error reading feedback: {e}")
                    break
        
        for target_vel, duration, description in velocity_sequence_3:
            print(f"\n{description}")
            print(f"  Target: {target_vel:+5d} rpm | Duration: {duration:.1f}s")
            
            # Set target velocity via RPDO (using object dictionary index 0x60FF)
            # CRITICAL: Must send BOTH Controlword and Target Velocity in SAME RPDO
            node_3.rpdo[3][ctl_variable].raw = 0x000F  # OPERATION ENABLED command
            node_3.rpdo[3][velocity_variable].raw = target_vel
            node_3.rpdo[3].transmit()  # Atomic transmission
            
            # Monitor actual velocity for the duration
            start_time = time.time()
            sample_count = 0
            
            while time.time() - start_time < duration:
                try:
                    network.check()
                    
                    # Wait for TPDO with velocity feedback
                    node_3.tpdo[1].wait_for_reception(timeout=0.5)
                    
                    # Read actual velocity
                    actual_vel = node_3.tpdo[4][vel_variable].raw
                    actual_pos = node_3.tpdo[4][pos_variable].raw
                    statusword = node_3.tpdo[1][status_variable].raw
                    
                    target_val = node_3.rpdo[3][velocity_variable].raw
                    
                    # Print status every 10 samples (~100ms)
                    # sample_count += 1
                    # if sample_count % 10 == 0:
                    print(f" Target vel {target_val},  Actual: {actual_vel:+5d} rpm | Actual pos: {actual_pos:+5d} | "
                              f"Status: 0x{statusword:04X} | "
                              f"State: {node_3.state}")
                    
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
        node_2.rpdo[3][ctl_variable].raw = 0x000F  # Maintain OPERATION ENABLED
        node_2.rpdo[3][velocity_variable].raw = 0
        node_2.rpdo[3].transmit()  # Atomic transmission
        time.sleep(1.0)
        print("    ✓ Motor stopped")
        node_3.rpdo[3][ctl_variable].raw = 0x000F  # Maintain OPERATION ENABLED
        node_3.rpdo[3][velocity_variable].raw = 0
        node_3.rpdo[3].transmit()  # Atomic transmission
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
