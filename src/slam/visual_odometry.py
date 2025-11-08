"""
Visual Odometry using stereo camera data.
Estimates robot pose from stereo images.
"""

import cv2
import numpy as np
from typing import Optional, Tuple, List
from ..utils.data_structures import RobotPose, KeyFrame


class VisualOdometry:
    """Visual odometry using stereo camera images."""
    
    def __init__(self, config=None):
        """
        Initialize visual odometry.
        
        Args:
            config: Configuration dictionary
        """
        self.config = config or {}
        vo_config = self.config.get('visual_odometry', {})
        
        # Feature detector settings
        self.detector_type = vo_config.get('detector_type', 'ORB')
        self.max_features = vo_config.get('max_features', 1000)
        self.quality_level = vo_config.get('quality_level', 0.01)
        
        # Initialize feature detector
        if self.detector_type == 'ORB':
            self.detector = cv2.ORB_create(nfeatures=self.max_features)
        elif self.detector_type == 'SIFT':
            self.detector = cv2.SIFT_create(nfeatures=self.max_features)
        elif self.detector_type == 'SURF':
            # SURF is not available in opencv-python by default
            self.detector = cv2.ORB_create(nfeatures=self.max_features)
            print("[WARNING] SURF not available, using ORB instead")
        else:
            self.detector = cv2.ORB_create(nfeatures=self.max_features)
        
        # Feature matcher settings
        self.matcher_type = vo_config.get('matcher_type', 'BF')
        self.norm_type = vo_config.get('norm_type', 'NORM_HAMMING')
        self.ratio_test = vo_config.get('ratio_test', 0.7)
        
        # Initialize matcher
        if self.matcher_type == 'BF':
            if self.detector_type == 'ORB':
                norm = cv2.NORM_HAMMING
            else:
                norm = cv2.NORM_L2
            self.matcher = cv2.BFMatcher(norm, crossCheck=False)
        else:
            self.matcher = cv2.BFMatcher(cv2.NORM_H2, crossCheck=False)
        
        # RANSAC settings
        self.ransac_threshold = vo_config.get('ransac_threshold', 1.0)
        self.ransac_confidence = vo_config.get('ransac_confidence', 0.99)
        self.ransac_max_iterations = vo_config.get('ransac_max_iterations', 2000)
        
        # Stereo settings
        stereo_config = self.config.get('stereo', {})
        self.stereo = cv2.StereoBM_create(
            numDisparities=stereo_config.get('num_disparities', 64),
            blockSize=stereo_config.get('block_size', 15)
        )
        
        # Previous frame
        self.prev_frame = None
        self.prev_keypoints = None
        self.prev_descriptors = None
        self.prev_pose = RobotPose(0.0, 0.0, 0.0)  # Start at origin
        
        # Camera intrinsics (placeholder - should be calibrated)
        self.camera_matrix = None
        self.dist_coeffs = None
        
    def detect_features(self, image: np.ndarray) -> Tuple[List[cv2.KeyPoint], np.ndarray]:
        """
        Detect features in image.
        
        Args:
            image: Grayscale image
        
        Returns:
            Tuple of (keypoints, descriptors)
        """
        if len(image.shape) == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        keypoints, descriptors = self.detector.detectAndCompute(image, None)
        return keypoints, descriptors
    
    def match_features(self, desc1: np.ndarray, desc2: np.ndarray) -> List[cv2.DMatch]:
        """
        Match features between two frames.
        
        Args:
            desc1: Descriptors from first frame
            desc2: Descriptors from second frame
        
        Returns:
            List of good matches
        """
        if desc1 is None or desc2 is None or len(desc1) == 0 or len(desc2) == 0:
            return []
        
        # Match features
        matches = self.matcher.knnMatch(desc1, desc2, k=2)
        
        # Apply ratio test (Lowe's ratio test)
        good_matches = []
        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < self.ratio_test * n.distance:
                    good_matches.append(m)
        
        return good_matches
    
    def estimate_pose(self, curr_image: np.ndarray, curr_depth: Optional[np.ndarray] = None) -> Optional[RobotPose]:
        """
        Estimate pose change from previous frame.
        
        Args:
            curr_image: Current RGB image
            curr_depth: Current depth map (optional)
        
        Returns:
            Estimated pose change (delta pose) or None if estimation fails
        """
        if len(curr_image.shape) == 3:
            curr_gray = cv2.cvtColor(curr_image, cv2.COLOR_BGR2GRAY)
        else:
            curr_gray = curr_image
        
        # Detect features
        curr_keypoints, curr_descriptors = self.detect_features(curr_gray)
        
        if self.prev_frame is None:
            # First frame - store and return identity
            self.prev_frame = curr_gray
            self.prev_keypoints = curr_keypoints
            self.prev_descriptors = curr_descriptors
            return RobotPose(0.0, 0.0, 0.0)  # No motion
        
        # Match features
        matches = self.match_features(self.prev_descriptors, curr_descriptors)
        
        if len(matches) < 10:  # Need minimum matches
            # Not enough matches, update previous frame and return None
            self.prev_frame = curr_gray
            self.prev_keypoints = curr_keypoints
            self.prev_descriptors = curr_descriptors
            return None
        
        # Extract matched points
        prev_pts = np.float32([self.prev_keypoints[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        curr_pts = np.float32([curr_keypoints[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
        
        # Estimate motion using essential matrix or homography
        # For now, use simple translation estimation
        # TODO: Implement proper essential matrix estimation with camera intrinsics
        
        # Simple translation estimation (placeholder)
        if self.camera_matrix is not None and curr_depth is not None:
            # Use depth for 3D reconstruction
            # TODO: Implement proper 3D pose estimation
            pass
        
        # Estimate homography (planar motion assumption)
        homography, mask = cv2.findHomography(
            prev_pts, curr_pts,
            cv2.RANSAC,
            self.ransac_threshold,
            maxIters=self.ransac_max_iterations
        )
        
        if homography is None:
            # Update previous frame
            self.prev_frame = curr_gray
            self.prev_keypoints = curr_keypoints
            self.prev_descriptors = curr_descriptors
            return None
        
        # Extract translation and rotation from homography
        # This is a simplified approach - proper VO would use essential matrix
        # For now, use simple translation estimation
        dx = homography[0, 2] / 100.0  # Scale factor (placeholder)
        dy = homography[1, 2] / 100.0
        dtheta = np.arctan2(homography[1, 0], homography[0, 0])
        
        # Update previous frame
        self.prev_frame = curr_gray
        self.prev_keypoints = curr_keypoints
        self.prev_descriptors = curr_descriptors
        
        return RobotPose(dx, dy, dtheta)
    
    def update_pose(self, delta_pose: RobotPose) -> RobotPose:
        """
        Update cumulative pose from delta pose.
        
        Args:
            delta_pose: Delta pose from odometry
        
        Returns:
            Updated cumulative pose
        """
        # Update pose (simple addition for now - should account for rotation)
        self.prev_pose.x += delta_pose.x * np.cos(self.prev_pose.theta) - delta_pose.y * np.sin(self.prev_pose.theta)
        self.prev_pose.y += delta_pose.x * np.sin(self.prev_pose.theta) + delta_pose.y * np.cos(self.prev_pose.theta)
        self.prev_pose.theta += delta_pose.theta
        
        return self.prev_pose
    
    def get_current_pose(self) -> RobotPose:
        """Get current pose estimate."""
        return self.prev_pose
    
    def reset_pose(self, pose: Optional[RobotPose] = None):
        """Reset pose to initial position or specified pose."""
        if pose is None:
            self.prev_pose = RobotPose(0.0, 0.0, 0.0)
        else:
            self.prev_pose = pose

