"""
Motion Planner for high-level motion commands.
Converts high-level commands to Arduino commands.
"""

from typing import Optional
from .arduino_interface import ArduinoInterface
from .mecanum_controller import MecanumController


class MotionPlanner:
    """High-level motion planner that sends commands to Arduino."""
    
    def __init__(self, arduino_interface: ArduinoInterface, config=None):
        """
        Initialize motion planner.
        
        Args:
            arduino_interface: ArduinoInterface instance
            config: Configuration dictionary
        """
        self.arduino = arduino_interface
        self.config = config or {}
        
        # Default motion settings
        self.default_speed = self.config.get('default_speed', 50)
        self.default_turn_speed = self.config.get('default_turn_speed', 50)
        self.max_speed = self.config.get('max_speed', 100)
        
        # Initialize Mecanum controller (for future use when kinematics are implemented)
        robot_config = self.config.get('robot', {})
        dimensions = robot_config.get('dimensions', {})
        wheels = robot_config.get('wheels', {})
        
        self.mecanum = MecanumController(
            wheel_base=dimensions.get('wheel_base', 0.20),
            track_width=dimensions.get('track_width', 0.20),
            wheel_radius=wheels.get('radius', 0.05)
        )
    
    def move_forward(self, speed: Optional[int] = None, distance: Optional[float] = None) -> bool:
        """
        Move robot forward.
        
        Args:
            speed: Speed (0-100), None uses default
            distance: Distance to move (meters), None for continuous
        
        Returns:
            True if command sent successfully
        """
        if speed is None:
            speed = self.default_speed
        speed = max(0, min(100, speed))
        
        # TODO: Implement distance-based movement when odometry is available
        if distance is not None:
            print(f"[INFO] Distance-based movement not yet implemented")
        
        return self.arduino.move_forward(speed)
    
    def move_backward(self, speed: Optional[int] = None, distance: Optional[float] = None) -> bool:
        """
        Move robot backward.
        
        Args:
            speed: Speed (0-100), None uses default
            distance: Distance to move (meters), None for continuous
        
        Returns:
            True if command sent successfully
        """
        if speed is None:
            speed = self.default_speed
        speed = max(0, min(100, speed))
        
        # TODO: Implement distance-based movement when odometry is available
        if distance is not None:
            print(f"[INFO] Distance-based movement not yet implemented")
        
        return self.arduino.move_backward(speed)
    
    def turn_left(self, angle: Optional[float] = None, speed: Optional[int] = None) -> bool:
        """
        Turn robot left.
        
        Args:
            angle: Angle in degrees, None for continuous
            speed: Turn speed (0-100), None uses default
        
        Returns:
            True if command sent successfully
        """
        if speed is None:
            speed = self.default_turn_speed
        speed = max(0, min(100, speed))
        
        if angle is None:
            angle = 90.0  # Default turn angle
        
        # TODO: Implement angle-based turning when odometry is available
        return self.arduino.turn_left(angle)
    
    def turn_right(self, angle: Optional[float] = None, speed: Optional[int] = None) -> bool:
        """
        Turn robot right.
        
        Args:
            angle: Angle in degrees, None for continuous
            speed: Turn speed (0-100), None uses default
        
        Returns:
            True if command sent successfully
        """
        if speed is None:
            speed = self.default_turn_speed
        speed = max(0, min(100, speed))
        
        if angle is None:
            angle = 90.0  # Default turn angle
        
        # TODO: Implement angle-based turning when odometry is available
        return self.arduino.turn_right(angle)
    
    def stop(self) -> bool:
        """Stop robot movement."""
        return self.arduino.stop()
    
    def move_velocity(self, vx: float, vy: float, omega: float) -> bool:
        """
        Move robot with velocity commands.
        TODO: Implement when Mecanum kinematics are ready.
        
        Args:
            vx: Forward velocity (m/s)
            vy: Lateral velocity (m/s)
            omega: Angular velocity (rad/s)
        
        Returns:
            True if command sent successfully
        """
        # TODO: Convert velocity to wheel speeds using Mecanum kinematics
        # For now, use simple forward/backward/turn commands
        print(f"[INFO] Velocity-based movement not yet implemented")
        return False

