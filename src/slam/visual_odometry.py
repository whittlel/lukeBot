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
        self.prev_depth = None
        
        # Camera intrinsics (will be set from camera)
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # IMU data for visual-inertial odometry
        self.use_imu = vo_config.get('use_imu', True)
        self.prev_imu_data = None
        self.imu_gravity = np.array([0.0, 0.0, -9.81])  # Gravity vector
        self.imu_alpha = vo_config.get('imu_alpha', 0.9)  # IMU fusion weight
        
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
    
    def set_camera_intrinsics(self, camera_matrix: np.ndarray, dist_coeffs: Optional[np.ndarray] = None):
        """Set camera intrinsics from calibration."""
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
    
    def estimate_pose(self, curr_image: np.ndarray, curr_depth: Optional[np.ndarray] = None, 
                     imu_data: Optional[dict] = None) -> Optional[RobotPose]:
        """
        Estimate pose change from previous frame using visual odometry with optional depth and IMU.
        
        Args:
            curr_image: Current RGB image
            curr_depth: Current depth map (optional)
            imu_data: IMU data dictionary with 'accel', 'gyro', 'mag', 'timestamp' (optional)
        
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
            self.prev_depth = curr_depth
            self.prev_imu_data = imu_data
            return RobotPose(0.0, 0.0, 0.0)  # No motion
        
        # Match features
        matches = self.match_features(self.prev_descriptors, curr_descriptors)
        
        if len(matches) < 10:  # Need minimum matches
            # Not enough matches, update previous frame and return None
            self.prev_frame = curr_gray
            self.prev_keypoints = curr_keypoints
            self.prev_descriptors = curr_descriptors
            self.prev_depth = curr_depth
            self.prev_imu_data = imu_data
            return None
        
        # Extract matched points
        prev_pts = np.float32([self.prev_keypoints[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        curr_pts = np.float32([curr_keypoints[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
        
        # Try 3D pose estimation with depth if available
        if self.camera_matrix is not None and curr_depth is not None and self.prev_depth is not None:
            delta_pose = self._estimate_pose_3d(prev_pts, curr_pts, curr_depth, self.prev_depth)
            if delta_pose is not None:
                # Fuse with IMU if available
                if self.use_imu and imu_data is not None and self.prev_imu_data is not None:
                    delta_pose = self._fuse_imu(delta_pose, imu_data, self.prev_imu_data)
                
                # Update previous frame
                self.prev_frame = curr_gray
                self.prev_keypoints = curr_keypoints
                self.prev_descriptors = curr_descriptors
                self.prev_depth = curr_depth
                self.prev_imu_data = imu_data
                return delta_pose
        
        # Fallback to 2D homography estimation
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
            self.prev_depth = curr_depth
            self.prev_imu_data = imu_data
            return None
        
        # Extract translation and rotation from homography
        # This is a simplified approach - proper VO would use essential matrix
        dx = homography[0, 2] / 100.0  # Scale factor (placeholder)
        dy = homography[1, 2] / 100.0
        dtheta = np.arctan2(homography[1, 0], homography[0, 0])
        
        delta_pose = RobotPose(dx, dy, dtheta)
        
        # Fuse with IMU if available
        if self.use_imu and imu_data is not None and self.prev_imu_data is not None:
            delta_pose = self._fuse_imu(delta_pose, imu_data, self.prev_imu_data)
        
        # Update previous frame
        self.prev_frame = curr_gray
        self.prev_keypoints = curr_keypoints
        self.prev_descriptors = curr_descriptors
        self.prev_depth = curr_depth
        self.prev_imu_data = imu_data
        
        return delta_pose
    
    def _estimate_pose_3d(self, prev_pts: np.ndarray, curr_pts: np.ndarray,
                         curr_depth: np.ndarray, prev_depth: np.ndarray) -> Optional[RobotPose]:
        """Estimate pose using 3D point correspondences from depth."""
        if self.camera_matrix is None:
            return None
        
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        # Convert 2D points to 3D using depth
        prev_3d = []
        curr_3d = []
        
        for i, (prev_pt, curr_pt) in enumerate(zip(prev_pts, curr_pts)):
            px, py = int(prev_pt[0, 0]), int(prev_pt[0, 1])
            cx_pt, cy_pt = int(curr_pt[0, 0]), int(curr_pt[0, 1])
            
            # Check bounds
            if (0 <= py < prev_depth.shape[0] and 0 <= px < prev_depth.shape[1] and
                0 <= cy_pt < curr_depth.shape[0] and 0 <= cx_pt < curr_depth.shape[1]):
                
                d_prev = prev_depth[py, px]
                d_curr = curr_depth[cy_pt, cx_pt]
                
                # Valid depth values
                if d_prev > 0.1 and d_prev < 10.0 and d_curr > 0.1 and d_curr < 10.0:
                    # Convert to 3D
                    x_prev = (px - cx) * d_prev / fx
                    y_prev = (py - cy) * d_prev / fy
                    z_prev = d_prev
                    
                    x_curr = (cx_pt - cx) * d_curr / fx
                    y_curr = (cy_pt - cy) * d_curr / fy
                    z_curr = d_curr
                    
                    prev_3d.append([x_prev, y_prev, z_prev])
                    curr_3d.append([x_curr, y_curr, z_curr])
        
        if len(prev_3d) < 4:
            return None
        
        prev_3d = np.array(prev_3d)
        curr_3d = np.array(curr_3d)
        
        # Use ICP or PnP to estimate transformation
        # For now, use simple centroid-based estimation
        prev_centroid = np.mean(prev_3d, axis=0)
        curr_centroid = np.mean(curr_3d, axis=0)
        
        # Translation
        translation = curr_centroid - prev_centroid
        
        # Simple rotation estimation (for planar motion, mainly around Z-axis)
        # Project to XY plane for 2D motion
        dx = translation[0]
        dy = translation[1]
        
        # Estimate rotation from point correspondences
        # Use SVD for rotation estimation
        H = (prev_3d - prev_centroid).T @ (curr_3d - curr_centroid)
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # Extract rotation angle (for 2D motion)
        dtheta = np.arctan2(R[1, 0], R[0, 0])
        
        return RobotPose(dx, dy, dtheta)
    
    def _fuse_imu(self, visual_pose: RobotPose, curr_imu: dict, prev_imu: dict) -> RobotPose:
        """Fuse IMU data with visual odometry estimate."""
        if curr_imu is None or prev_imu is None:
            return visual_pose
        
        if curr_imu.get('gyro') is None or prev_imu.get('gyro') is None:
            return visual_pose
        
        # Get gyroscope data for rotation
        curr_gyro = curr_imu['gyro']
        prev_gyro = prev_imu['gyro']
        
        # Estimate angular velocity (simplified)
        # For 2D motion, use Z-axis rotation
        if hasattr(curr_gyro, 'z') and hasattr(prev_gyro, 'z'):
            # Time difference (assume 30 FPS = 0.033s)
            dt = 0.033
            gyro_z = (curr_gyro.z + prev_gyro.z) / 2.0
            imu_dtheta = gyro_z * dt
            
            # Fuse rotation estimates
            fused_theta = self.imu_alpha * imu_dtheta + (1 - self.imu_alpha) * visual_pose.theta
            
            return RobotPose(visual_pose.x, visual_pose.y, fused_theta)
        
        return visual_pose
    
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

