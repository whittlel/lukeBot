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

# Add parent to path for proper package imports
sys.path.insert(0, str(Path(__file__).parent))

from src.camera.oakd_camera import OakDCamera
from src.slam.slam_engine import SLAMEngine
from src.slam.exploration_planner import ExplorationPlanner
from src.slam.obstacle_avoidance import ObstacleAvoidance
from src.motor_control.arduino_interface import ArduinoInterface
from src.motor_control.motion_planner import MotionPlanner
from src.motor_control.path_planner import PathPlanner
from src.utils.logger import setup_logger
from src.utils.data_structures import RobotPose


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
        
        # Set camera intrinsics in SLAM after camera starts
        # (Will be set in start() method after camera initialization)
        
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
        
        # Path planner
        self.logger.info("Initializing path planner...")
        self.path_planner = PathPlanner(config=robot_config)
        
        # Exploration planner
        self.logger.info("Initializing exploration planner...")
        self.exploration_planner = ExplorationPlanner(config=slam_config)
        
        # Obstacle avoidance
        self.logger.info("Initializing obstacle avoidance...")
        self.obstacle_avoidance = ObstacleAvoidance(config=slam_config)
        
        # Control flags
        self.running = False
        self.paused = False
        self.autonomous_mode = True
        self.current_path = None
        self.current_waypoint_index = 0
        self.exploration_target = None
        
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
        
        # Set camera intrinsics in SLAM
        camera_matrix = self.camera.get_camera_intrinsics()
        dist_coeffs = self.camera.get_distortion_coeffs()
        if camera_matrix is not None:
            self.slam.set_camera_intrinsics(camera_matrix, dist_coeffs)
            self.slam.map_builder.set_camera_intrinsics(camera_matrix, dist_coeffs)
            self.logger.info("Camera intrinsics set in SLAM")
        
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
                
                # Update obstacle avoidance with YOLO detections
                camera_matrix = self.camera.get_camera_intrinsics()
                self.obstacle_avoidance.update_dynamic_obstacles(
                    data['detections'], data['depth'], camera_matrix
                )
                
                # Process frame for SLAM with IMU data
                pose = self.slam.process_frame(data['rgb'], data['depth'], data.get('imu'))
                
                # Check for emergency stop
                if pose is not None and self.obstacle_avoidance.should_emergency_stop(pose):
                    self.motion_planner.stop()
                    self.logger.warning("Emergency stop triggered!")
                
                if pose is not None:
                    self.logger.debug(f"Pose: x={pose.x:.2f}, y={pose.y:.2f}, theta={pose.theta:.2f}")
                    
                    # Autonomous exploration mode
                    if self.autonomous_mode:
                        self._autonomous_exploration(pose, data['rgb'])
                
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
                elif key == ord('e'):
                    # Toggle autonomous exploration mode
                    self.autonomous_mode = not self.autonomous_mode
                    self.logger.info(f"Autonomous mode: {self.autonomous_mode}")
                    if not self.autonomous_mode:
                        self.motion_planner.stop()
                        self.current_path = None
                
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            self.logger.info("Keyboard interrupt received")
        except Exception as e:
            self.logger.error(f"Error in main loop: {e}", exc_info=True)
        finally:
            cv2.destroyAllWindows()
            self.stop()
    
    def _autonomous_exploration(self, pose: RobotPose, rgb_image: np.ndarray):
        """Autonomous exploration logic."""
        # Get occupancy grid
        occupancy_grid = self.slam.map_builder.occupancy_grid
        if occupancy_grid is None:
            return
        
        # Check if we need a new exploration target
        if self.exploration_target is None or self.current_path is None or \
           self.current_waypoint_index >= len(self.current_path):
            
            # Get next exploration waypoint
            grid_resolution = self.slam.map_builder.grid_resolution
            grid_origin = self.slam.map_builder.grid_origin
            prob_free = self.slam.map_builder.prob_free
            prob_occupied = self.slam.map_builder.prob_occupied
            prob_unknown = self.slam.map_builder.prob_unknown
            
            waypoint = self.exploration_planner.get_exploration_waypoint(
                occupancy_grid, pose, grid_resolution, grid_origin,
                prob_free, prob_occupied, prob_unknown
            )
            
            if waypoint is None:
                self.logger.info("No more frontiers to explore")
                self.autonomous_mode = False
                return
            
            self.exploration_target = waypoint
            
            # Plan path to waypoint
            self.current_path = self.path_planner.plan_path(
                pose, waypoint, occupancy_grid, grid_resolution, grid_origin,
                prob_free, prob_occupied
            )
            
            if self.current_path is None:
                self.logger.warning("Could not plan path to exploration target")
                self.exploration_target = None
                return
            
            self.current_waypoint_index = 0
            self.logger.info(f"New exploration target: {waypoint}, path length: {len(self.current_path)}")
        
        # Follow current path
        if self.current_path and self.current_waypoint_index < len(self.current_path):
            target = self.current_path[self.current_waypoint_index]
            
            # Calculate distance to waypoint
            distance = np.sqrt((target[0] - pose.x)**2 + (target[1] - pose.y)**2)
            
            if distance < 0.2:  # Reached waypoint
                self.current_waypoint_index += 1
                if self.current_waypoint_index < len(self.current_path):
                    target = self.current_path[self.current_waypoint_index]
                else:
                    # Reached exploration target
                    self.exploration_target = None
                    return
            
            # Calculate direction to waypoint
            dx = target[0] - pose.x
            dy = target[1] - pose.y
            
            # Check for obstacles in path
            if self.obstacle_avoidance.check_collision(pose, occupancy_grid,
                                                      self.slam.map_builder.grid_resolution,
                                                      self.slam.map_builder.grid_origin):
                # Try to find safe direction
                safe_dir = self.obstacle_avoidance.get_safe_direction(
                    pose, occupancy_grid, self.slam.map_builder.grid_resolution,
                    self.slam.map_builder.grid_origin
                )
                
                if safe_dir is None:
                    # No safe direction, replan
                    self.current_path = None
                    self.exploration_target = None
                    return
                else:
                    dx, dy = safe_dir
            
            # Calculate angle to waypoint
            target_angle = np.arctan2(dy, dx)
            angle_diff = target_angle - pose.theta
            
            # Normalize angle to [-pi, pi]
            while angle_diff > np.pi:
                angle_diff -= 2 * np.pi
            while angle_diff < -np.pi:
                angle_diff += 2 * np.pi
            
            # Move towards waypoint
            if abs(angle_diff) > 0.2:  # Need to turn
                if angle_diff > 0:
                    self.motion_planner.turn_left(np.degrees(abs(angle_diff)))
                else:
                    self.motion_planner.turn_right(np.degrees(abs(angle_diff)))
            else:  # Move forward
                speed = min(50, int(distance * 20))  # Adjust speed based on distance
                self.motion_planner.move_forward(speed)


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
    print("  e - Toggle autonomous exploration mode")
    print("=" * 50)
    
    robot = LukeBot()
    robot.start()


if __name__ == "__main__":
    main()

