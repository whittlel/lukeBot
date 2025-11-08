"""
SLAM Engine - Main orchestrator for SLAM.
Combines visual odometry and map building.
"""

import numpy as np
from typing import Optional
from datetime import datetime
from ..utils.data_structures import RobotPose, KeyFrame
from .visual_odometry import VisualOdometry
from .map_builder import MapBuilder


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
        
        # Initialize components
        self.visual_odometry = VisualOdometry(config=slam_config)
        self.map_builder = MapBuilder(config=slam_config)
        
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
    
    def process_frame(self, rgb_image: np.ndarray, depth_image: Optional[np.ndarray] = None) -> Optional[RobotPose]:
        """
        Process a new frame for SLAM.
        
        Args:
            rgb_image: RGB image from camera
            depth_image: Depth map from camera (optional)
        
        Returns:
            Current pose estimate or None if processing failed
        """
        self.frame_count += 1
        
        # Check if we should process this frame
        frames_since_last = self.frame_count - self.last_processed_frame
        if frames_since_last < self.keyframe_interval:
            return self.current_pose
        
        try:
            # Estimate pose change using visual odometry
            delta_pose = self.visual_odometry.estimate_pose(rgb_image, depth_image)
            
            if delta_pose is not None:
                # Update cumulative pose
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
            
            self.last_processed_frame = self.frame_count
            return self.current_pose
            
        except Exception as e:
            print(f"[ERROR] SLAM processing error: {e}")
            return None
    
    def get_current_pose(self) -> RobotPose:
        """Get current pose estimate."""
        return self.current_pose
    
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

