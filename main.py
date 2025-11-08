#!/usr/bin/env python3
"""
Main entry point for LukeBot SLAM robot.
Orchestrates camera, SLAM, and motor control modules.
"""

import sys
import time
import signal
import yaml
from pathlib import Path
import cv2
import numpy as np

# Add src to path
sys.path.insert(0, str(Path(__file__).parent / "src"))

from camera.oakd_camera import OakDCamera
from slam.slam_engine import SLAMEngine
from motor_control.arduino_interface import ArduinoInterface
from motor_control.motion_planner import MotionPlanner
from utils.logger import setup_logger
from utils.data_structures import RobotPose


class LukeBot:
    """Main robot controller."""
    
    def __init__(self, config_dir=None):
        """
        Initialize LukeBot.
        
        Args:
            config_dir: Path to configuration directory
        """
        if config_dir is None:
            config_dir = Path(__file__).parent / "config"
        
        # Load configurations
        camera_config_path = Path(config_dir) / "camera_config.yaml"
        slam_config_path = Path(config_dir) / "slam_config.yaml"
        robot_config_path = Path(config_dir) / "robot_config.yaml"
        
        with open(camera_config_path, 'r') as f:
            camera_config = yaml.safe_load(f)
        
        with open(slam_config_path, 'r') as f:
            slam_config = yaml.safe_load(f)
        
        with open(robot_config_path, 'r') as f:
            robot_config = yaml.safe_load(f)
        
        # Setup logger
        self.logger = setup_logger("lukebot", log_file="data/logs/lukebot.log")
        
        # Initialize components
        self.logger.info("Initializing LukeBot...")
        
        # Camera
        self.logger.info("Initializing camera...")
        self.camera = OakDCamera(config=camera_config)
        
        # SLAM
        self.logger.info("Initializing SLAM...")
        self.slam = SLAMEngine(config=slam_config)
        
        # Arduino interface
        self.logger.info("Initializing Arduino interface...")
        arduino_config = robot_config.get('motors', {}).get('arduino', {})
        self.arduino = ArduinoInterface(
            port=arduino_config.get('port'),
            baud_rate=arduino_config.get('baud_rate', 115200),
            timeout=arduino_config.get('timeout', 1.0)
        )
        
        # Motion planner
        self.logger.info("Initializing motion planner...")
        self.motion_planner = MotionPlanner(
            self.arduino,
            config=robot_config
        )
        
        # Control flags
        self.running = False
        self.paused = False
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        self.logger.info("Received shutdown signal, stopping...")
        self.stop()
        sys.exit(0)
    
    def start(self):
        """Start the robot."""
        self.logger.info("Starting LukeBot...")
        
        # Connect to Arduino
        if not self.arduino.connect():
            self.logger.error("Failed to connect to Arduino")
            return False
        
        # Start camera
        if not self.camera.start():
            self.logger.error("Failed to start camera")
            self.arduino.disconnect()
            return False
        
        self.running = True
        self.logger.info("LukeBot started successfully")
        
        # Main loop
        self.run()
        
        return True
    
    def stop(self):
        """Stop the robot."""
        self.logger.info("Stopping LukeBot...")
        
        self.running = False
        
        # Stop motors
        self.motion_planner.stop()
        
        # Stop camera
        self.camera.stop()
        
        # Disconnect Arduino
        self.arduino.disconnect()
        
        # Save map
        self.slam.save_map()
        
        self.logger.info("LukeBot stopped")
    
    def run(self):
        """Main robot loop."""
        self.logger.info("Entering main loop...")
        
        frame_count = 0
        last_fps_time = time.time()
        fps = 0.0
        
        try:
            while self.running:
                if self.paused:
                    time.sleep(0.1)
                    continue
                
                # Get camera data
                data = self.camera.get_all_data()
                
                if data['rgb'] is None:
                    time.sleep(0.01)
                    continue
                
                # Process frame for SLAM
                pose = self.slam.process_frame(data['rgb'], data['depth'])
                
                if pose is not None:
                    self.logger.debug(f"Pose: x={pose.x:.2f}, y={pose.y:.2f}, theta={pose.theta:.2f}")
                
                # Draw detections on frame
                frame = self.camera.draw_detections(data['rgb'], data['detections'])
                
                # Display FPS
                frame_count += 1
                current_time = time.time()
                if current_time - last_fps_time >= 1.0:
                    fps = frame_count / (current_time - last_fps_time)
                    frame_count = 0
                    last_fps_time = current_time
                
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Display pose
                if pose is not None:
                    pose_text = f"Pose: ({pose.x:.2f}, {pose.y:.2f}, {np.degrees(pose.theta):.1f}°)"
                    cv2.putText(frame, pose_text, (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Display frame
                cv2.imshow("LukeBot - Camera Feed", frame)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.stop()
                    break
                elif key == ord('p'):
                    self.paused = not self.paused
                    self.logger.info(f"Paused: {self.paused}")
                elif key == ord('w'):
                    # Move forward
                    self.motion_planner.move_forward(50)
                elif key == ord('s'):
                    # Move backward
                    self.motion_planner.move_backward(50)
                elif key == ord('a'):
                    # Turn left
                    self.motion_planner.turn_left(90.0)
                elif key == ord('d'):
                    # Turn right
                    self.motion_planner.turn_right(90.0)
                elif key == ord(' '):
                    # Stop
                    self.motion_planner.stop()
                
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            self.logger.info("Keyboard interrupt received")
        except Exception as e:
            self.logger.error(f"Error in main loop: {e}", exc_info=True)
        finally:
            cv2.destroyAllWindows()
            self.stop()


def main():
    """Main entry point."""
    print("=" * 50)
    print("LukeBot - SLAM Robot")
    print("=" * 50)
    print("Controls:")
    print("  q - Quit")
    print("  p - Pause/Resume")
    print("  w - Move forward")
    print("  s - Move backward")
    print("  a - Turn left")
    print("  d - Turn right")
    print("  Space - Stop")
    print("=" * 50)
    
    robot = LukeBot()
    robot.start()


if __name__ == "__main__":
    main()

