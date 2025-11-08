"""
Unit tests for motor control module.
"""

import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from motor_control.arduino_interface import ArduinoInterface
from motor_control.mecanum_controller import MecanumController
from motor_control.motion_planner import MotionPlanner


class TestMecanumController(unittest.TestCase):
    """Test Mecanum controller."""
    
    def test_controller_initialization(self):
        """Test controller initialization."""
        controller = MecanumController(
            wheel_base=0.20,
            track_width=0.20,
            wheel_radius=0.05
        )
        self.assertIsNotNone(controller)
        self.assertEqual(controller.wheel_base, 0.20)
        self.assertEqual(controller.track_width, 0.20)
        self.assertEqual(controller.wheel_radius, 0.05)
    
    def test_inverse_kinematics(self):
        """Test inverse kinematics."""
        controller = MecanumController()
        
        # Test forward movement
        fl, fr, rl, rr = controller.inverse_kinematics(1.0, 0.0, 0.0)
        self.assertIsInstance(fl, float)
        self.assertIsInstance(fr, float)
        self.assertIsInstance(rl, float)
        self.assertIsInstance(rr, float)
    
    def test_forward_kinematics(self):
        """Test forward kinematics."""
        controller = MecanumController()
        
        # Test forward kinematics
        vx, vy, omega = controller.forward_kinematics(1.0, 1.0, 1.0, 1.0)
        self.assertIsInstance(vx, float)
        self.assertIsInstance(vy, float)
        self.assertIsInstance(omega, float)
    
    def test_normalize_speeds(self):
        """Test speed normalization."""
        controller = MecanumController()
        
        # Test normalization
        fl, fr, rl, rr = controller.normalize_wheel_speeds(100.0, 50.0, 75.0, 25.0)
        self.assertAlmostEqual(max(abs(fl), abs(fr), abs(rl), abs(rr)), 1.0, places=5)


class TestArduinoInterface(unittest.TestCase):
    """Test Arduino interface."""
    
    def test_interface_initialization(self):
        """Test interface initialization."""
        interface = ArduinoInterface(port=None, baud_rate=115200)
        self.assertIsNotNone(interface)
        self.assertEqual(interface.baud_rate, 115200)
    
    def test_find_arduino_port(self):
        """Test Arduino port detection."""
        interface = ArduinoInterface()
        port = interface.find_arduino_port()
        # Port may be None if Arduino not connected
        if port is not None:
            self.assertIsInstance(port, str)


if __name__ == "__main__":
    unittest.main()

