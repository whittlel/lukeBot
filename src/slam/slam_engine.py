"""
SLAM Engine - Main orchestrator for SLAM.
Combines visual odometry and map building.
"""

import numpy as np
import logging
from typing import Optional
from datetime import datetime
from ..utils.data_structures import RobotPose, KeyFrame
from .visual_odometry import VisualOdometry
from .depth_odometry import DepthOdometry
from .map_builder import MapBuilder
from .slam_visualizer import SLAMVisualizer


class SLAMEngine:
    """Main SLAM engine that orchestrates visual odometry and mapping."""

    def __init__(self, config=None):
        """
        Initialize SLAM engine.

        Args:
            config: Configuration dictionary
        """
        self.config = config or {}
        slam_config = self.config.get('slam', {})
        performance_config = slam_config.get('performance', {})

        # Setup logger
        self.logger = logging.getLogger(__name__)

        # Initialize components
        self.visual_odometry = VisualOdometry(config=slam_config)
        self.depth_odometry = DepthOdometry(config=slam_config)
        self.map_builder = MapBuilder(config=slam_config)
        self.visualizer = SLAMVisualizer(config=slam_config)

        # Odometry mode: 'depth', 'visual', or 'hybrid'
        self.odometry_mode = slam_config.get('odometry_mode', 'hybrid')

        # Performance settings
        self.process_rate = performance_config.get('process_rate', 10)  # Hz
        self.keyframe_interval = performance_config.get('keyframe_interval', 5)
        self.min_movement = performance_config.get('min_movement', 0.1)  # meters

        # Frame counter
        self.frame_count = 0
        self.last_processed_frame = 0

        # Current pose
        self.current_pose = RobotPose(0.0, 0.0, 0.0)

        # Keyframes
        self.keyframes = []

        # Store last processed frame data for visualization
        self.last_rgb_frame = None
        self.last_depth_frame = None

        # Store previous frame features for tracking visualization
        self.prev_frame_keypoints = None
        self.prev_frame_descriptors = None

        # Track which odometry method was used last
        self.last_odometry_source = "none"

        # Hysteresis for mode switching (prevents rapid switching)
        self.depth_consecutive_failures = 0
        self.visual_consecutive_failures = 0
        self.hysteresis_threshold = 3  # Require 3 consecutive failures before switching modes
        self.preferred_mode = "depth" if self.odometry_mode == "hybrid" else self.odometry_mode

        # IMU buffer for continuous integration between keyframes
        self.imu_buffer = []
    
    def set_camera_intrinsics(self, camera_matrix, dist_coeffs=None):
        """Set camera intrinsics for visual and depth odometry."""
        self.visual_odometry.set_camera_intrinsics(camera_matrix, dist_coeffs)
        self.depth_odometry.set_camera_intrinsics(camera_matrix, dist_coeffs)
    
    def process_frame(self, rgb_image: np.ndarray, depth_image: Optional[np.ndarray] = None,
                     imu_data: Optional[dict] = None) -> Optional[RobotPose]:
        """
        Process a new frame for SLAM.

        Args:
            rgb_image: RGB image from camera
            depth_image: Depth map from camera (optional)
            imu_data: IMU data dictionary (optional)

        Returns:
            Current pose estimate or None if processing failed
        """
        self.frame_count += 1

        # Store frames for visualization
        self.last_rgb_frame = rgb_image
        self.last_depth_frame = depth_image

        # Buffer IMU data even for skipped frames (for continuous integration)
        # Note: imu_data is now a list of samples from camera, extend to add all samples
        if imu_data is not None:
            if isinstance(imu_data, list):
                self.imu_buffer.extend(imu_data)
            else:
                self.imu_buffer.append(imu_data)

        # Check if we should process this frame
        frames_since_last = self.frame_count - self.last_processed_frame
        if frames_since_last < self.keyframe_interval:
            return self.current_pose

        try:
            # Store previous frame features before processing new frame
            self.prev_frame_keypoints = self.visual_odometry.prev_keypoints
            self.prev_frame_descriptors = self.visual_odometry.prev_descriptors

            # Estimate pose change using selected odometry method
            delta_pose = None
            odometry_source = ""

            if self.odometry_mode == 'depth':
                # Depth-only mode
                if depth_image is not None:
                    delta_pose = self.depth_odometry.estimate_pose(depth_image, rgb_image, self.imu_buffer)
                    odometry_source = "depth"

            elif self.odometry_mode == 'visual':
                # Visual-only mode (legacy)
                delta_pose = self.visual_odometry.estimate_pose(rgb_image, depth_image, imu_data)
                odometry_source = "visual"

            elif self.odometry_mode == 'hybrid':
                # Hybrid mode with hysteresis: Try preferred mode first, switch only after consecutive failures
                if self.preferred_mode == "depth" and depth_image is not None:
                    depth_pose = self.depth_odometry.estimate_pose(depth_image, rgb_image, self.imu_buffer)
                    if depth_pose is not None:
                        delta_pose = depth_pose
                        odometry_source = "depth"
                        # Reset failure counters on success
                        self.depth_consecutive_failures = 0
                        self.visual_consecutive_failures = 0
                    else:
                        # Depth failed
                        self.depth_consecutive_failures += 1

                        # Switch to visual if depth has failed consecutively
                        if self.depth_consecutive_failures >= self.hysteresis_threshold:
                            self.logger.warning(f"Switching preferred mode from depth to visual after {self.depth_consecutive_failures} consecutive failures")
                            self.preferred_mode = "visual"
                            # Try visual as fallback this frame
                            visual_pose = self.visual_odometry.estimate_pose(rgb_image, depth_image, imu_data)
                            if visual_pose is not None:
                                delta_pose = visual_pose
                                odometry_source = "visual"
                                self.visual_consecutive_failures = 0
                                self.logger.info("Visual odometry succeeded after mode switch")
                        else:
                            # Still within hysteresis threshold, try visual as temporary fallback
                            visual_pose = self.visual_odometry.estimate_pose(rgb_image, depth_image, imu_data)
                            if visual_pose is not None:
                                delta_pose = visual_pose
                                odometry_source = "visual (temp)"  # Indicate temporary fallback

                elif self.preferred_mode == "visual":
                    visual_pose = self.visual_odometry.estimate_pose(rgb_image, depth_image, imu_data)
                    if visual_pose is not None:
                        delta_pose = visual_pose
                        odometry_source = "visual"
                        # Reset failure counters on success
                        self.visual_consecutive_failures = 0
                        self.depth_consecutive_failures = 0
                    else:
                        # Visual failed
                        self.visual_consecutive_failures += 1

                        # Switch back to depth if visual has failed consecutively and depth is available
                        if self.visual_consecutive_failures >= self.hysteresis_threshold and depth_image is not None:
                            self.logger.warning(f"Switching preferred mode from visual to depth after {self.visual_consecutive_failures} consecutive failures")
                            self.preferred_mode = "depth"
                            # Try depth as fallback this frame
                            depth_pose = self.depth_odometry.estimate_pose(depth_image, rgb_image, self.imu_buffer)
                            if depth_pose is not None:
                                delta_pose = depth_pose
                                odometry_source = "depth"
                                self.depth_consecutive_failures = 0
                                self.logger.info("Depth odometry succeeded after mode switch")
                        else:
                            # Still within hysteresis threshold, try depth as temporary fallback
                            if depth_image is not None:
                                depth_pose = self.depth_odometry.estimate_pose(depth_image, rgb_image, self.imu_buffer)
                                if depth_pose is not None:
                                    delta_pose = depth_pose
                                    odometry_source = "depth (temp)"  # Indicate temporary fallback

            if delta_pose is not None:
                # Track odometry source for visualization
                self.last_odometry_source = odometry_source

                # Update cumulative pose using the active odometry method
                # Use substring match to handle "depth", "depth (temp)", etc.
                if "depth" in odometry_source:
                    self.current_pose = self.depth_odometry.update_pose(delta_pose)
                else:
                    self.current_pose = self.visual_odometry.update_pose(delta_pose)

                # Check if movement is significant enough for keyframe
                movement = np.sqrt(delta_pose.x**2 + delta_pose.y**2)
                if movement >= self.min_movement:
                    # Create keyframe
                    keyframe = KeyFrame(
                        frame_id=self.frame_count,
                        image=rgb_image.copy(),
                        depth=depth_image.copy() if depth_image is not None else None,
                        pose=self.current_pose,
                        features=self.visual_odometry.prev_keypoints,
                        descriptors=self.visual_odometry.prev_descriptors
                    )
                    self.keyframes.append(keyframe)

                # Update map
                self.map_builder.update_occupancy_grid(self.current_pose, depth_image)

                # Auto-save map if needed
                self.map_builder.auto_save_if_needed()

            # Clear IMU buffer after processing keyframe
            self.imu_buffer = []

            self.last_processed_frame = self.frame_count
            return self.current_pose

        except Exception as e:
            print(f"[ERROR] SLAM processing error: {e}")
            return None
    
    def get_current_pose(self) -> RobotPose:
        """Get current pose estimate."""
        return self.current_pose

    def get_odometry_source(self) -> str:
        """Get the odometry method used in last frame."""
        return self.last_odometry_source

    def get_odometry_diagnostics(self) -> dict:
        """
        Get odometry diagnostics for debugging.

        Returns:
            Dictionary with diagnostic information
        """
        return {
            'odometry_mode': self.odometry_mode,
            'last_source': self.last_odometry_source,
            'preferred_mode': self.preferred_mode if self.odometry_mode == 'hybrid' else self.odometry_mode,
            'depth_failures': self.depth_consecutive_failures,
            'visual_failures': self.visual_consecutive_failures,
            'hysteresis_threshold': self.hysteresis_threshold,
            'frame_count': self.frame_count
        }
    
    def get_map_image(self) -> Optional[np.ndarray]:
        """Get current map as image."""
        return self.map_builder.get_map_image()
    
    def save_map(self, filename: Optional[str] = None):
        """Save current map."""
        self.map_builder.save_map(filename)
    
    def load_map(self, filename: str):
        """Load map from file."""
        self.map_builder.load_map(filename)
    
    def reset(self):
        """Reset SLAM engine to initial state."""
        self.visual_odometry.reset_pose()
        self.current_pose = RobotPose(0.0, 0.0, 0.0)
        self.frame_count = 0
        self.last_processed_frame = 0
        self.keyframes = []

    # Visualization methods
    def get_visualization_data(self):
        """
        Get all data needed for visualization.

        Returns:
            Dictionary with visualization data
        """
        # Use immediate previous frame for tracking visualization (not keyframe)
        # This shows frame-to-frame tracking instead of keyframe-to-keyframe
        prev_keypoints = self.prev_frame_keypoints
        prev_descriptors = self.prev_frame_descriptors

        return {
            'rgb_image': self.last_rgb_frame,
            'depth_image': self.last_depth_frame,
            'keypoints': self.visual_odometry.prev_keypoints,
            'prev_keypoints': prev_keypoints,
            'descriptors': self.visual_odometry.prev_descriptors,
            'prev_descriptors': prev_descriptors,
            'pose': self.current_pose,
            'keyframes': self.keyframes
        }

    def get_slam_annotated_view(self, camera_matrix: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """
        Get SLAM annotated view showing features, depth, and tracking.

        Args:
            camera_matrix: Camera intrinsics for 3D visualization

        Returns:
            Annotated SLAM view image
        """
        viz_data = self.get_visualization_data()

        if viz_data['rgb_image'] is None:
            return None

        # Get matches if we have previous frame
        matches = None
        if viz_data['prev_keypoints'] is not None and viz_data['prev_descriptors'] is not None:
            # Get previous frame descriptors
            prev_desc = viz_data['prev_descriptors']
            curr_desc = viz_data['descriptors']

            if prev_desc is not None and curr_desc is not None:
                # Match features between consecutive frames
                matches = self.visual_odometry.match_features(prev_desc, curr_desc)

        return self.visualizer.create_slam_annotated_view(
            viz_data['rgb_image'],
            viz_data['depth_image'],
            viz_data['keypoints'],
            viz_data['prev_keypoints'],
            matches,
            viz_data['pose'],
            camera_matrix
        )

    def get_multi_panel_debug_view(self, camera_matrix: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """
        Get multi-panel debug view with all visualization panels.

        Args:
            camera_matrix: Camera intrinsics for 3D visualization

        Returns:
            Multi-panel debug view image
        """
        viz_data = self.get_visualization_data()

        if viz_data['rgb_image'] is None:
            return None

        # Get matches if we have previous frame
        matches = None
        if viz_data['prev_keypoints'] is not None and viz_data['prev_descriptors'] is not None:
            prev_desc = viz_data['prev_descriptors']
            curr_desc = viz_data['descriptors']

            if prev_desc is not None and curr_desc is not None:
                matches = self.visual_odometry.match_features(prev_desc, curr_desc)

        return self.visualizer.create_multi_panel_debug_view(
            viz_data['rgb_image'],
            viz_data['depth_image'],
            viz_data['keypoints'],
            viz_data['prev_keypoints'],
            matches,
            viz_data['pose'],
            camera_matrix,
            max_depth_range=1.5,
            odometry_diagnostics=self.get_odometry_diagnostics()
        )

