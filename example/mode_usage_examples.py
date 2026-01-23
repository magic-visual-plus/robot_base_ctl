"""
Mode Usage Examples

This file demonstrates how to use different operation modes with the AxisInterface.
"""

import os
import time
import sys
from robot_base_ctl.motor.moons.axis_interface import OperationMode
from robot_base_ctl.motor.moons.canopen402_axis import Canopen402Axis
"""
Mode Usage Examples

This file demonstrates how to use different operation modes with the AxisInterface.
"""

 
def example_velocity_mode(axis: Canopen402Axis):
    """Example: Velocity control mode."""
    print("=== Velocity Mode Example ===")
    
    # Switch to velocity mode
    axis.set_mode(OperationMode.PROFILED_VELOCITY)
    # Or: axis.set_mode("PROFILED_VELOCITY")
    
    # Check if mode is supported
    if not axis.is_mode_supported(OperationMode.PROFILED_VELOCITY):
        print("Velocity mode not supported!")
        return
    
    # Set velocity profile parameters
    axis.set_velocity_profile(acc=1000000, dec=1000000, quick_stop_dec=2000000)
    
    # Set target velocity
    axis.set_target_velocity(1000)  # 1000 rpm
    
    # Read velocity feedback
    for i in range(10):
        statusword, actual_vel = axis.read_velocity_feedback(timeout_s=0.5)
        print(f"Status: 0x{statusword:04X}, Actual Velocity: {actual_vel}")
        time.sleep(0.1)
    
    # Stop
    axis.stop()


def example_position_mode(axis: Canopen402Axis):
    """Example: Position control mode."""
    print("=== Position Mode Example ===")
    
    # Switch to position mode
    axis.set_mode(OperationMode.PROFILED_POSITION)
    
    # Check if mode is supported
    if not axis.is_mode_supported(OperationMode.PROFILED_POSITION):
        print("Position mode not supported!")
        return
    
    # Set position profile parameters
    axis.set_position_profile(vel=50000, acc=1000000, dec=1000000)
    
    # Move to absolute position
    axis.set_target_position(100000, relative=False)  # 100000 counts
    
    # Wait for target reached
    if axis.wait_for_target_reached(timeout_s=5.0):
        print("Target position reached!")
    else:
        print("Timeout waiting for target")
    
    # Read position feedback
    statusword, actual_pos = axis.read_position_feedback(timeout_s=0.5)
    print(f"Status: 0x{statusword:04X}, Actual Position: {actual_pos}")
    
    # Move relative position
    axis.set_target_position(50000, relative=True)  # Move 50000 counts forward
    axis.wait_for_target_reached(timeout_s=5.0)


def example_torque_mode(axis: Canopen402Axis):
    """Example: Torque control mode."""
    print("=== Torque Mode Example ===")
    
    # Switch to torque mode
    axis.set_mode(OperationMode.PROFILED_TORQUE)
    
    # Check if mode is supported
    if not axis.is_mode_supported(OperationMode.PROFILED_TORQUE):
        print("Torque mode not supported!")
        return
    
    # Set target torque
    axis.set_target_torque(500)  # 500 units (device-specific)
    
    # Read torque feedback
    for i in range(10):
        statusword, actual_torque = axis.read_torque_feedback(timeout_s=0.5)
        print(f"Status: 0x{statusword:04X}, Actual Torque: {actual_torque}")
        time.sleep(0.1)
    
    # Stop
    axis.set_target_torque(0)


def example_homing_mode(axis: Canopen402Axis):
    """Example: Homing mode."""
    print("=== Homing Mode Example ===")
    
    # Switch to homing mode
    axis.set_mode(OperationMode.HOMING)
    
    # Check if mode is supported
    if not axis.is_mode_supported(OperationMode.HOMING):
        print("Homing mode not supported!")
        return
    
    # Start homing with method 35 (typical: negative limit switch)
    axis.start_homing(method=35)
    
    # Wait for homing to complete
    timeout = 30.0  # 30 seconds timeout
    start_time = time.time()
    while time.time() - start_time < timeout:
        if axis.is_homing_complete():
            print("Homing completed successfully!")
            break
        time.sleep(0.1)
    else:
        print("Homing timeout!")


def example_mode_switching(axis: Canopen402Axis):
    """Example: Switching between different modes."""
    print("=== Mode Switching Example ===")
    
    # Get current mode
    current_mode = axis.get_mode()
    print(f"Current mode: {current_mode.to_string()}")
    
    # List all supported modes
    all_modes = [
        OperationMode.PROFILED_POSITION,
        OperationMode.VELOCITY,
        OperationMode.PROFILED_VELOCITY,
        OperationMode.PROFILED_TORQUE,
        OperationMode.HOMING,
    ]
    
    print("\nSupported modes:")
    for mode in all_modes:
        supported = axis.is_mode_supported(mode)
        print(f"  {mode.to_string()}: {'✓' if supported else '✗'}")
    
    # Switch modes
    print("\nSwitching to velocity mode...")
    axis.switch_mode(OperationMode.PROFILED_VELOCITY)
    print(f"Current mode: {axis.get_mode().to_string()}")
    
    print("\nSwitching to position mode...")
    axis.switch_mode(OperationMode.PROFILED_POSITION)
    print(f"Current mode: {axis.get_mode().to_string()}")


if __name__ == "__main__":
    # Get the directory of this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Build relative path to EDS file
    eds_path = os.path.join(script_dir, '..', 'motor', 'moons', 'CANOPEN-EDS-MBDV-Servo-SingleAxis-V1.1.1.eds')
    eds_path = os.path.normpath(eds_path)
    
    # Initialize axis (adjust parameters as needed)
    axis = Canopen402Axis(
        node_id=2,
        eds_path=eds_path,
        channel='can1',
        bitrate=50000,
        rpdo_cmd_no=3,
        tpdo_fb_no=1,
        tpdo_vel_no=3,
    )
    
    try:
        # Connect and enable
        axis.connect()
        axis.boot()
        axis.enable()
        
        # Run examples (uncomment as needed)
        # example_velocity_mode(axis)
        example_position_mode(axis)
        # example_torque_mode(axis)
        # example_homing_mode(axis)
        # example_mode_switching(axis)
        
    finally:
        axis.shutdown()
