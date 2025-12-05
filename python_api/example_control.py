#!/usr/bin/env python3
"""
BDX Robot Control Example

Example of real-time robot control using the BDX API.
Demonstrates:
- Telemetry streaming
- Motor position control
- Emergency stop handling

Author: BDX_SIm Project
"""

import time
import signal
import sys
from bdx_robot import BDXRobot, Telemetry, list_ports

# Global robot instance for signal handler
robot: BDXRobot = None


def signal_handler(sig, frame):
    """Handle Ctrl+C for clean shutdown."""
    print("\n\nEmergency stop triggered!")
    if robot and robot.connected:
        robot.emergency_stop()
        robot.disconnect()
    sys.exit(0)


def telemetry_callback(telemetry: Telemetry):
    """Called on each telemetry update."""
    # Print IMU data
    imu = telemetry.imu
    print(f"\rIMU: acc=[{imu.accel.x:+6.2f}, {imu.accel.y:+6.2f}, {imu.accel.z:+6.2f}] "
          f"gyro=[{imu.gyro.x:+6.2f}, {imu.gyro.y:+6.2f}, {imu.gyro.z:+6.2f}]  ", end='')


def main():
    global robot

    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)

    # Find port
    ports = list_ports()
    if not ports:
        print("No USB devices found!")
        return

    port = ports[0]
    print(f"Connecting to {port}...")

    # Create robot instance
    robot = BDXRobot(port)

    if not robot.connect():
        print("Failed to connect!")
        return

    try:
        # Get initial status
        status = robot.get_status()
        if status:
            print(f"Connected! {status.motor_count} motors, "
                  f"loop freq: {status.loop_freq_hz} Hz")

        # Setup streaming callback
        robot.set_telemetry_callback(telemetry_callback)

        # Enable streaming at control loop rate
        robot.enable_streaming(True, interval_ms=0)

        # Enable motors
        print("\nEnabling motors...")
        robot.enable_motors()
        time.sleep(0.5)

        # Example: Simple position control sequence
        print("\nStarting control loop (Ctrl+C to stop)...")
        print("-" * 60)

        positions = [0] * 11  # Initial positions for 11 motors

        while True:
            # Example: Sinusoidal motion for motor 1
            t = time.time()
            import math
            positions[0] = int(1000 * math.sin(t * 2 * math.pi * 0.5))  # 0.5 Hz

            # Send position commands
            pos_dict = {i+1: positions[i] for i in range(len(positions))}
            robot.set_motor_positions(pos_dict)

            # Control rate limiting
            time.sleep(0.002)  # 500 Hz update rate

    except Exception as e:
        print(f"\nError: {e}")

    finally:
        # Clean shutdown
        print("\n\nShutting down...")
        robot.disable_motors()
        robot.enable_streaming(False)
        robot.disconnect()


if __name__ == '__main__':
    main()
