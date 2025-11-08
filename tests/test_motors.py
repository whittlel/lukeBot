#!/usr/bin/env python3
"""
Test script for motor control via Arduino.
Tests high-level motion commands.
"""

import sys
import time
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from motor_control.arduino_interface import ArduinoInterface
from motor_control.motion_planner import MotionPlanner
import yaml


def main():
    """Test motor control."""
    print("Testing Motor Control...")
    
    # Load robot config
    config_path = Path(__file__).parent.parent / "config" / "robot_config.yaml"
    with open(config_path, 'r') as f:
        robot_config = yaml.safe_load(f)
    
    # Initialize Arduino interface
    arduino_config = robot_config.get('motors', {}).get('arduino', {})
    arduino = ArduinoInterface(
        port=arduino_config.get('port'),
        baud_rate=arduino_config.get('baud_rate', 115200),
        timeout=arduino_config.get('timeout', 1.0)
    )
    
    # Initialize motion planner
    motion_planner = MotionPlanner(arduino, config=robot_config)
    
    # Connect to Arduino
    if not arduino.connect():
        print("ERROR: Failed to connect to Arduino")
        return
    
    print("Arduino connected. Testing motors...")
    print("Commands:")
    print("  w - Move forward")
    print("  s - Move backward")
    print("  a - Turn left")
    print("  d - Turn right")
    print("  Space - Stop")
    print("  q - Quit")
    
    try:
        while True:
            # Read user input
            command = input("\nEnter command (w/s/a/d/space/q): ").strip().lower()
            
            if command == 'q':
                break
            elif command == 'w':
                print("Moving forward...")
                motion_planner.move_forward(50)
                time.sleep(2)
                motion_planner.stop()
            elif command == 's':
                print("Moving backward...")
                motion_planner.move_backward(50)
                time.sleep(2)
                motion_planner.stop()
            elif command == 'a':
                print("Turning left...")
                motion_planner.turn_left(90.0)
                time.sleep(2)
                motion_planner.stop()
            elif command == 'd':
                print("Turning right...")
                motion_planner.turn_right(90.0)
                time.sleep(2)
                motion_planner.stop()
            elif command == ' ' or command == 'space':
                print("Stopping...")
                motion_planner.stop()
            else:
                print("Unknown command")
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        motion_planner.stop()
        arduino.disconnect()
        print("Motor test completed")


if __name__ == "__main__":
    main()

