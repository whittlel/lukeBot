#!/usr/bin/env python3
"""
Automated functional test for mecanum wheels.
Tests each wheel individually, then tests motion patterns.
"""

import sys
import time
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from motor_control.arduino_interface import ArduinoInterface
from motor_control.motion_planner import MotionPlanner
import yaml


def test_individual_wheels(arduino):
    """Test each wheel individually."""
    print("\n" + "="*50)
    print("TESTING INDIVIDUAL WHEELS")
    print("="*50)

    wheels = [
        ("Front Left", "FL", 50),
        ("Front Right", "FR", 50),
        ("Rear Left", "RL", 50),
        ("Rear Right", "RR", 50)
    ]

    for wheel_name, wheel_id, speed in wheels:
        print(f"\n--- Testing {wheel_name} Wheel ({wheel_id}) ---")

        # Test forward
        print(f"  Spinning {wheel_name} FORWARD at speed {speed}...")
        arduino.send_command(f"WHEEL:{wheel_id}:{speed}")
        time.sleep(1.5)

        # Stop
        arduino.send_command(f"WHEEL:{wheel_id}:0")
        time.sleep(0.5)

        # Test backward
        print(f"  Spinning {wheel_name} BACKWARD at speed {-speed}...")
        arduino.send_command(f"WHEEL:{wheel_id}:{-speed}")
        time.sleep(1.5)

        # Stop
        arduino.send_command(f"WHEEL:{wheel_id}:0")
        time.sleep(1.0)

        print(f"  {wheel_name} wheel test complete ✓")

    print("\nIndividual wheel tests COMPLETE ✓")


def test_motion_patterns(motion_planner):
    """Test various motion patterns."""
    print("\n" + "="*50)
    print("TESTING MOTION PATTERNS")
    print("="*50)

    test_speed = 50
    test_duration = 2.0

    patterns = [
        ("Forward", lambda: motion_planner.move_forward(test_speed)),
        ("Backward", lambda: motion_planner.move_backward(test_speed)),
        ("Turn Left", lambda: motion_planner.turn_left(45.0)),
        ("Turn Right", lambda: motion_planner.turn_right(45.0)),
    ]

    for pattern_name, pattern_func in patterns:
        print(f"\n--- Testing {pattern_name} ---")
        print(f"  Executing {pattern_name.lower()} for {test_duration}s...")
        pattern_func()
        time.sleep(test_duration)
        motion_planner.stop()
        print(f"  {pattern_name} test complete ✓")
        time.sleep(1.0)

    print("\nMotion pattern tests COMPLETE ✓")


def main():
    """Run functional wheel tests."""
    print("="*50)
    print("FUNCTIONAL WHEEL TEST FOR LUKEBOT")
    print("="*50)
    print("\nThis test will:")
    print("1. Test each wheel individually (forward/backward)")
    print("2. Test motion patterns (forward, backward, strafe, turn)")
    print("\nMake sure the robot is on blocks or in a safe area!")
    print("\nStarting test in 3 seconds...")
    time.sleep(3)

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

    # Connect to Arduino
    if not arduino.connect():
        print("ERROR: Failed to connect to Arduino")
        print("Please check:")
        print("  - Arduino is connected via USB")
        print("  - Correct firmware is uploaded")
        print("  - Port permissions (try: sudo chmod 666 /dev/ttyACM0)")
        return 1

    print(f"\n✓ Arduino connected successfully")

    try:
        # Test 1: Individual wheels
        test_individual_wheels(arduino)

        # Test 2: Motion patterns
        motion_planner = MotionPlanner(arduino, config=robot_config)
        test_motion_patterns(motion_planner)

        # All tests complete
        print("\n" + "="*50)
        print("ALL TESTS COMPLETE ✓")
        print("="*50)
        print("\nFunctional wheel test finished successfully!")
        return 0

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        return 1
    except Exception as e:
        print(f"\n\nERROR during testing: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        # Ensure all motors are stopped
        arduino.send_command("STOP")
        time.sleep(0.1)
        arduino.disconnect()
        print("\n✓ Motors stopped and Arduino disconnected")


if __name__ == "__main__":
    sys.exit(main())
