"""
Depth-based Odometry using ICP (Iterative Closest Point).
Uses point cloud registration for pose estimation in low-texture environments.
"""

import cv2
import numpy as np
from typing import Optional, Tuple
from ..utils.data_structures import RobotPose


class DepthOdometry:
    """Depth-based odometry using ICP point cloud registration."""

    def __init__(self, config=None):
        """
        Initialize depth odometry.

        Args:
            config: Configuration dictionary
        """
        self.config = config or {}
        depth_config = self.config.get('depth_odometry', {})

        # ICP settings
        self.max_iterations = depth_config.get('icp_max_iterations', 20)
        self.convergence_threshold = depth_config.get('icp_convergence', 0.001)
        self.max_correspondence_distance = depth_config.get('max_correspondence_distance', 0.1)  # meters

        # Point cloud sampling
        self.sample_rate = depth_config.get('sample_rate', 10)  # Sample every Nth pixel
        self.min_depth = depth_config.get('min_depth', 0.1)  # meters
        self.max_depth = depth_config.get('max_depth', 5.0)  # meters
        self.min_points = depth_config.get('min_points', 100)  # Minimum points for ICP

        # Camera intrinsics
        self.camera_matrix = None
        self.dist_coeffs = None

        # Previous frame data
        self.prev_point_cloud = None
        self.prev_pose = RobotPose(0.0, 0.0, 0.0)

        # IMU integration
        self.use_imu = depth_config.get('use_imu', True)
        self.imu_alpha = depth_config.get('imu_alpha', 0.7)  # Weight for IMU vs ICP

    def set_camera_intrinsics(self, camera_matrix: np.ndarray, dist_coeffs: Optional[np.ndarray] = None):
        """Set camera intrinsics for depth unprojection."""
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

    def depth_to_point_cloud(self, depth_image: np.ndarray,
                            rgb_image: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Convert depth image to 3D point cloud.

        Args:
            depth_image: Depth map in meters
            rgb_image: Optional RGB image for colored point cloud

        Returns:
            Nx3 or Nx6 array of points (x, y, z) or (x, y, z, r, g, b)
        """
        if self.camera_matrix is None:
            raise ValueError("Camera intrinsics not set. Call set_camera_intrinsics() first.")

        # Ensure camera_matrix is numpy array
        if not isinstance(self.camera_matrix, np.ndarray):
            camera_matrix = np.array(self.camera_matrix)
        else:
            camera_matrix = self.camera_matrix

        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]

        h, w = depth_image.shape
        points = []

        # Check if RGB image dimensions match depth
        rgb_h, rgb_w = (0, 0)
        if rgb_image is not None:
            if len(rgb_image.shape) == 3:
                rgb_h, rgb_w = rgb_image.shape[:2]
            else:
                rgb_h, rgb_w = rgb_image.shape

        # Sample points at intervals
        for y in range(0, h, self.sample_rate):
            for x in range(0, w, self.sample_rate):
                # Bounds check
                if y >= h or x >= w:
                    continue

                depth = depth_image[y, x]

                # Valid depth check
                if depth > self.min_depth and depth < self.max_depth:
                    # Convert to 3D
                    z = depth
                    px = (x - cx) * z / fx
                    py = (y - cy) * z / fy

                    if rgb_image is not None:
                        # Scale coordinates if RGB has different size
                        if rgb_h > 0 and rgb_w > 0:
                            rgb_y = int(y * rgb_h / h)
                            rgb_x = int(x * rgb_w / w)

                            # Bounds check for RGB
                            if rgb_y < rgb_h and rgb_x < rgb_w:
                                r, g, b = rgb_image[rgb_y, rgb_x]
                                points.append([px, py, z, r, g, b])
                            else:
                                points.append([px, py, z])
                        else:
                            points.append([px, py, z])
                    else:
                        points.append([px, py, z])

        return np.array(points) if len(points) > 0 else np.array([]).reshape(0, 3)

    def icp(self, source: np.ndarray, target: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Perform ICP (Iterative Closest Point) registration.

        Args:
            source: Source point cloud (Nx3)
            target: Target point cloud (Nx3)

        Returns:
            Tuple of (rotation matrix 3x3, translation vector 3x1, final error)
        """
        if len(source) < self.min_points or len(target) < self.min_points:
            return np.eye(3), np.zeros((3, 1)), float('inf')

        # Only use XYZ coordinates
        source_xyz = source[:, :3]
        target_xyz = target[:, :3]

        # Initialize transformation
        R = np.eye(3)
        t = np.zeros((3, 1))

        prev_error = float('inf')

        for iteration in range(self.max_iterations):
            # Apply current transformation to source
            transformed_source = (R @ source_xyz.T + t).T

            # Find nearest neighbors using vectorized operations
            # Compute all pairwise distances at once (faster but memory intensive for large clouds)
            # Use broadcasting to compute distances more efficiently
            src_indices = []
            tgt_indices = []
            distances = []

            # Process in chunks to save memory
            chunk_size = min(len(transformed_source), 500)
            for i in range(0, len(transformed_source), chunk_size):
                chunk = transformed_source[i:i+chunk_size]
                # Compute distances for this chunk
                dists = np.linalg.norm(target_xyz[:, np.newaxis, :] - chunk[np.newaxis, :, :], axis=2)
                min_dists = np.min(dists, axis=0)
                min_indices = np.argmin(dists, axis=0)

                # Filter by max correspondence distance
                valid = min_dists < self.max_correspondence_distance
                valid_indices = np.where(valid)[0]

                for j, is_valid in enumerate(valid):
                    if is_valid:
                        src_indices.append(i + j)
                        tgt_indices.append(min_indices[j])
                        distances.append(min_dists[j])

            if len(src_indices) < self.min_points:
                break

            # Calculate mean error
            mean_error = np.mean(distances)

            # Check convergence
            if abs(prev_error - mean_error) < self.convergence_threshold:
                break

            prev_error = mean_error

            # Extract corresponding points
            src_matched = source_xyz[src_indices]
            tgt_matched = target_xyz[tgt_indices]

            # Compute centroids
            src_centroid = np.mean(src_matched, axis=0)
            tgt_centroid = np.mean(tgt_matched, axis=0)

            # Center the point clouds
            src_centered = src_matched - src_centroid
            tgt_centered = tgt_matched - tgt_centroid

            # Compute rotation using SVD
            H = src_centered.T @ tgt_centered
            U, S, Vt = np.linalg.svd(H)
            R_new = Vt.T @ U.T

            # Ensure proper rotation matrix (det = 1)
            if np.linalg.det(R_new) < 0:
                Vt[-1, :] *= -1
                R_new = Vt.T @ U.T

            # Compute translation
            t_new = tgt_centroid.reshape(3, 1) - R_new @ src_centroid.reshape(3, 1)

            # Update transformation
            R = R_new @ R
            t = R_new @ t + t_new

        return R, t, prev_error

    def estimate_pose(self, depth_image: np.ndarray,
                     rgb_image: Optional[np.ndarray] = None,
                     imu_data: Optional[dict] = None) -> Optional[RobotPose]:
        """
        Estimate pose change using ICP on depth point clouds.

        Args:
            depth_image: Current depth map in meters
            rgb_image: Optional RGB image
            imu_data: Optional IMU data dictionary

        Returns:
            Delta pose or None if estimation fails
        """
        try:
            # Convert depth to point cloud
            current_cloud = self.depth_to_point_cloud(depth_image, rgb_image)

            if len(current_cloud) < self.min_points:
                # Not enough points - not ready yet
                self.prev_point_cloud = current_cloud
                print(f"[DEPTH] Not enough points: {len(current_cloud)} < {self.min_points}")
                return None

            if self.prev_point_cloud is None or len(self.prev_point_cloud) < self.min_points:
                # First frame - no previous cloud to compare
                self.prev_point_cloud = current_cloud
                print(f"[DEPTH] First frame initialized with {len(current_cloud)} points")
                return None

            # Perform ICP: align current cloud to previous cloud
            R, t, error = self.icp(current_cloud, self.prev_point_cloud)

            # Diagnostic logging
            print(f"[DEPTH] ICP: points={len(current_cloud)}, error={error:.3f}m")

            # Check if ICP succeeded
            if error == float('inf') or error > 2.0:  # Relaxed tolerance for indoor movement
                # ICP failed
                print(f"[DEPTH] ICP FAILED: error={error:.3f}m > threshold")
                self.prev_point_cloud = current_cloud
                return None

            # Extract 2D pose from 3D transformation
            # Camera frame: X=right, Y=down, Z=forward
            # Robot frame: X=forward, Y=left
            dx = t[2, 0]   # Forward/backward (camera Z-axis)
            dy = -t[0, 0]  # Left/right (negative camera X-axis)

            # Extract rotation around Z-axis (yaw)
            dtheta = np.arctan2(R[1, 0], R[0, 0])

            # Create delta pose
            delta_pose = RobotPose(dx, dy, dtheta)

            # Diagnostic logging
            print(f"[DEPTH] Delta: dx={dx:.3f}m, dy={dy:.3f}m, dtheta={np.degrees(dtheta):.1f}deg")

            # Fuse with IMU if available
            if self.use_imu and imu_data is not None:
                delta_pose = self._fuse_imu(delta_pose, imu_data)

            # Update previous cloud
            self.prev_point_cloud = current_cloud

            return delta_pose

        except Exception as e:
            print(f"[ERROR] Depth odometry error: {e}")
            import traceback
            traceback.print_exc()
            return None

    def _fuse_imu(self, depth_pose: RobotPose, imu_data: dict) -> RobotPose:
        """Fuse IMU data with depth odometry estimate."""
        if imu_data is None or imu_data.get('gyro') is None:
            return depth_pose

        # Get gyroscope data for rotation
        gyro = imu_data['gyro']

        # For 2D motion, use Z-axis rotation
        if hasattr(gyro, 'z'):
            # Time difference (assume 30 FPS = 0.033s)
            dt = 0.033
            imu_dtheta = gyro.z * dt

            # Fuse rotation estimates (trust IMU more for rotation)
            fused_theta = self.imu_alpha * imu_dtheta + (1 - self.imu_alpha) * depth_pose.theta

            return RobotPose(depth_pose.x, depth_pose.y, fused_theta)

        return depth_pose

    def update_pose(self, delta_pose: RobotPose) -> RobotPose:
        """
        Update cumulative pose from delta pose.

        Args:
            delta_pose: Delta pose from odometry

        Returns:
            Updated cumulative pose
        """
        # Update pose accounting for rotation
        cos_theta = np.cos(self.prev_pose.theta)
        sin_theta = np.sin(self.prev_pose.theta)

        self.prev_pose.x += delta_pose.x * cos_theta - delta_pose.y * sin_theta
        self.prev_pose.y += delta_pose.x * sin_theta + delta_pose.y * cos_theta
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
        self.prev_point_cloud = None
