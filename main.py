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
from src.slam.map_overlay_visualizer import MapOverlayVisualizer
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
        
        # Camera (with built-in panoramic view support)
        self.logger.info("Initializing camera...")
        self.camera = OakDCamera(config=camera_config)
        self.use_panoramic_view = True  # Toggle for panoramic display
        
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

        # Map overlay visualizer
        self.logger.info("Initializing map overlay visualizer...")
        self.map_overlay_viz = MapOverlayVisualizer(config=slam_config)

        # Control flags
        self.running = False
        self.paused = False
        self.autonomous_mode = False  # Start with autonomous mode OFF for safety
        self.current_path = None
        self.current_waypoint_index = 0
        self.exploration_target = None

        # Visualization modes
        # 0: Normal view (panoramic with detections)
        # 1: SLAM debug view (features, tracking, depth)
        # 2: Side-by-side (panoramic vs SLAM)
        # 3: Multi-panel debug view (4 panels)
        # 4: Map view (top-down occupancy grid with robot and trajectory)
        self.visualization_mode = 0
        
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
                # Note: SLAM uses main camera for pose estimation (stable center view)
                # Panoramic view is used for visualization and enhanced obstacle detection
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

                # Prepare display frame based on visualization mode
                frame = None
                window_title = "LukeBot"

                if self.visualization_mode == 0:
                    # Mode 0: Normal view (panoramic with detections)
                    frame_with_detections = self.camera.draw_detections(data['rgb'], data['detections'])

                    if self.use_panoramic_view:
                        frame = self.camera.create_panoramic_view(frame_with_detections, add_labels=True)
                    else:
                        frame = frame_with_detections

                    window_title = "LukeBot - Camera Feed"

                elif self.visualization_mode == 1:
                    # Mode 1: SLAM debug view (features, tracking, depth)
                    camera_matrix = self.camera.get_camera_intrinsics()
                    frame = self.slam.get_slam_annotated_view(camera_matrix)

                    if frame is None:
                        frame = data['rgb'].copy()

                    window_title = "LukeBot - SLAM Debug View"

                elif self.visualization_mode == 2:
                    # Mode 2: Side-by-side (panoramic vs SLAM)
                    # Left: Panoramic view
                    frame_with_detections = self.camera.draw_detections(data['rgb'], data['detections'])
                    panoramic_view = self.camera.create_panoramic_view(frame_with_detections, add_labels=True)

                    # Right: SLAM annotated view
                    camera_matrix = self.camera.get_camera_intrinsics()
                    slam_view = self.slam.get_slam_annotated_view(camera_matrix)

                    if slam_view is None:
                        slam_view = data['rgb'].copy()

                    # Create side-by-side view
                    frame = self.slam.visualizer.create_side_by_side_view(
                        panoramic_view,
                        slam_view,
                        title_left="Panoramic View (Reality)",
                        title_right="SLAM View (What SLAM Sees)"
                    )

                    window_title = "LukeBot - Reality vs SLAM"

                elif self.visualization_mode == 3:
                    # Mode 3: Multi-panel debug view (4 panels)
                    camera_matrix = self.camera.get_camera_intrinsics()
                    frame = self.slam.get_multi_panel_debug_view(camera_matrix)

                    if frame is None:
                        frame = data['rgb'].copy()

                    window_title = "LukeBot - Multi-Panel Debug"

                elif self.visualization_mode == 4:
                    # Mode 4: Top-down map view with robot and trajectory
                    occupancy_grid = self.slam.map_builder.occupancy_grid
                    grid_origin = self.slam.map_builder.grid_origin
                    grid_resolution = self.slam.map_builder.grid_resolution
                    trajectory = self.slam.map_builder.trajectory

                    frame = self.map_overlay_viz.create_map_view(
                        occupancy_grid,
                        pose,
                        grid_origin,
                        grid_resolution,
                        trajectory=trajectory,
                        show_grid=True,
                        show_scale=True
                    )

                    window_title = "LukeBot - Map View"

                if frame is None:
                    continue

                # Display FPS
                frame_count += 1
                current_time = time.time()
                if current_time - last_fps_time >= 1.0:
                    fps = frame_count / (current_time - last_fps_time)
                    frame_count = 0
                    last_fps_time = current_time

                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # Display pose and odometry source (for modes 0 and 1)
                if self.visualization_mode in [0, 1] and pose is not None:
                    pose_text = f"Pose: ({pose.x:.2f}, {pose.y:.2f}, {np.degrees(pose.theta):.1f}°)"
                    cv2.putText(frame, pose_text, (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                    # Display odometry source
                    odom_source = self.slam.get_odometry_source()
                    odom_color = (0, 255, 255) if odom_source == "depth" else (255, 0, 255)  # Cyan for depth, Magenta for visual
                    cv2.putText(frame, f"Odometry: {odom_source.upper()}", (10, 90),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, odom_color, 2)

                # Display mode indicator
                mode_names = [
                    "NORMAL VIEW",
                    "SLAM DEBUG",
                    "SIDE-BY-SIDE",
                    "MULTI-PANEL",
                    "MAP VIEW"
                ]
                mode_text = f"Mode: {mode_names[self.visualization_mode]} (Press 'm' to change)"
                cv2.putText(frame, mode_text, (10, frame.shape[0] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

                # Display frame
                cv2.imshow(window_title, frame)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.stop()
                    break
                elif key == ord('p'):
                    self.paused = not self.paused
                    self.logger.info(f"Paused: {self.paused}")
                elif key == ord('w'):
                    # Move forward (slow but enough torque)
                    self.motion_planner.move_forward(20)
                elif key == ord('s'):
                    # Move backward (slow but enough torque)
                    self.motion_planner.move_backward(20)
                elif key == ord('a'):
                    # Turn left (continuous until space pressed)
                    self.motion_planner.turn_left(2.0)
                elif key == ord('d'):
                    # Turn right (continuous until space pressed)
                    self.motion_planner.turn_right(2.0)
                elif key == ord('t'):
                    # TEST: Controlled slow turn with auto-stop for IMU testing
                    print("[TEST] Starting controlled turn (3 seconds at speed=25)...")
                    self.motion_planner.turn_left(45.0, speed=25)
                    # Note: 'time' module is already imported at top of file
                    cv2.waitKey(3000)  # Wait 3 seconds (using cv2.waitKey instead of time.sleep to keep UI responsive)
                    self.motion_planner.stop()
                    print("[TEST] Turn complete, motors stopped")
                elif key == ord('r'):
                    # TEST: LONG rotation test for IMU integration
                    print("[TEST] Starting LONG rotation test (10 seconds at speed=25)...")
                    self.motion_planner.turn_left(360.0, speed=25)
                    cv2.waitKey(10000)  # Wait 10 seconds
                    self.motion_planner.stop()
                    print("[TEST] Long rotation complete, motors stopped")
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
                elif key == ord('v'):
                    # Toggle panoramic view
                    self.use_panoramic_view = not self.use_panoramic_view
                    self.logger.info(f"Panoramic view: {self.use_panoramic_view}")
                    print(f"View mode: {'PANORAMIC (~105° FOV)' if self.use_panoramic_view else 'SINGLE CAMERA (69° FOV)'}")
                elif key == ord('m'):
                    # Cycle through visualization modes
                    self.visualization_mode = (self.visualization_mode + 1) % 5
                    mode_names = ["NORMAL VIEW", "SLAM DEBUG", "SIDE-BY-SIDE", "MULTI-PANEL", "MAP VIEW"]
                    self.logger.info(f"Visualization mode: {mode_names[self.visualization_mode]}")
                    print(f"Visualization mode: {mode_names[self.visualization_mode]}")
                    # Close old window to prevent multiple windows
                    cv2.destroyAllWindows()
                elif key == ord('o'):
                    # Toggle odometry mode for debugging
                    current_mode = self.slam.odometry_mode
                    if current_mode == 'hybrid':
                        self.slam.odometry_mode = 'visual'
                        print("Odometry: VISUAL-ONLY (faster, for testing)")
                    elif current_mode == 'visual':
                        self.slam.odometry_mode = 'depth'
                        print("Odometry: DEPTH-ONLY")
                    else:
                        self.slam.odometry_mode = 'hybrid'
                        print("Odometry: HYBRID (depth + visual fallback)")
                    self.logger.info(f"Odometry mode: {self.slam.odometry_mode}")

                time.sleep(0.05)  # Increased from 0.01 to 0.05 to reduce loop rate
        
        except KeyboardInterrupt:
            self.logger.info("Keyboard interrupt received")
        except Exception as e:
            self.logger.error(f"Error in main loop: {e}", exc_info=True)
        finally:
            cv2.destroyAllWindows()
            self.stop()
    
    def _autonomous_exploration(self, pose: RobotPose, rgb_image: np.ndarray):
        """Simple autonomous exploration - move forward and turn when obstacles detected."""
        import random

        # Get depth data to check for obstacles
        data = self.camera.get_all_data()
        depth_frame = data.get('depth')

        obstacle_detected = False
        detection_reason = ""

        # Method 1: Check YOLO detections
        for obstacle in self.obstacle_avoidance.dynamic_obstacles:
            obs_x, obs_y, obs_z = obstacle['position']
            distance = np.sqrt(obs_z**2 + obs_x**2)

            if distance < 0.25:  # Obstacle within 25cm (very small space)
                obstacle_detected = True
                detection_reason = f"YOLO detection at {distance:.2f}m"
                break

        # Method 2: Check depth data in front center region
        if not obstacle_detected and depth_frame is not None:
            h, w = depth_frame.shape
            # Check center region at horizon level (avoid detecting floor)
            # Use upper-middle portion of the frame
            center_region = depth_frame[int(h*0.35):int(h*0.65), int(w*0.35):int(w*0.65)]

            if center_region.size > 0:
                # Depth values are already in meters!
                # Filter out invalid depths (0 or very large values)
                valid_depths = center_region[(center_region > 0.05) & (center_region < 10.0)]  # 5cm to 10m range

                if valid_depths.size > 50:  # Need at least 50 valid pixels
                    # Use 10th percentile instead of minimum to be less sensitive to noise
                    min_depth = np.percentile(valid_depths, 10)  # 10th percentile depth
                    median_depth = np.median(valid_depths)

                    self.logger.info(f"Depth: p10={min_depth:.2f}m, median={median_depth:.2f}m, valid_pixels={valid_depths.size}")

                    # Use 10th percentile depth for obstacle detection (very small space)
                    if min_depth < 0.25:  # Obstacle within 25cm
                        obstacle_detected = True
                        detection_reason = f"Depth: {min_depth:.2f}m"
                else:
                    self.logger.debug(f"Not enough valid depth pixels: {valid_depths.size}/{center_region.size}")

        if obstacle_detected:
            self.logger.info(f"Obstacle detected ({detection_reason}), turning...")

            # Randomly turn left or right to explore better
            turn_direction = random.choice(['left', 'right'])
            turn_angle = random.randint(5, 10)  # Random turn between 5-10 degrees (extremely slow)

            if turn_direction == 'left':
                self.motion_planner.turn_left(turn_angle)
            else:
                self.motion_planner.turn_right(turn_angle)

            time.sleep(1.2)
            self.motion_planner.stop()
            time.sleep(0.3)
        else:
            # Move forward slowly but with enough torque for feature tracking
            self.logger.debug("Path clear, moving forward")
            self.motion_planner.move_forward(20)  # Slow but enough torque to move
            time.sleep(1.5)
            self.motion_planner.stop()
            time.sleep(0.3)


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
    print("  a - Turn left (continuous, press Space to stop)")
    print("  d - Turn right (continuous, press Space to stop)")
    print("  t - TEST: Controlled 3-second left turn (for IMU testing)")
    print("  r - TEST: Long 10-second rotation (for IMU integration testing)")
    print("  Space - Stop")
    print("  e - Toggle autonomous exploration mode")
    print("  v - Toggle panoramic view (105° FOV)")
    print("  m - Cycle visualization modes (Normal/SLAM/Side-by-Side/Multi-Panel)")
    print("  o - Toggle odometry mode (Hybrid/Visual/Depth)")
    print("=" * 50)
    print("\nVisualization Modes:")
    print("  0: Normal view - Panoramic camera with detections")
    print("  1: SLAM debug - Features, tracking, and depth overlay")
    print("  2: Side-by-side - Panoramic (left) vs SLAM view (right)")
    print("  3: Multi-panel - 4-panel debug view with all visualizations")
    print("  4: Map view - Top-down occupancy grid with robot and trajectory")
    print("=" * 50)
    print("\nOdometry Modes:")
    print("  Hybrid: Depth ICP + Visual fallback (default)")
    print("  Visual: Feature-based only (faster for testing)")
    print("  Depth: ICP-based only")
    print("=" * 50)
    
    robot = LukeBot()
    robot.start()


if __name__ == "__main__":
    main()

