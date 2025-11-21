"""
Map Builder for SLAM.
Constructs and stores maps from SLAM data.
"""

import numpy as np
import cv2
from pathlib import Path
from typing import Optional, Tuple
import yaml
from datetime import datetime

from ..utils.data_structures import RobotPose, MapPoint


class MapBuilder:
    """Builds and stores maps from SLAM data."""
    
    def __init__(self, config=None):
        """
        Initialize map builder.
        
        Args:
            config: Configuration dictionary
        """
        self.config = config or {}
        mapping_config = self.config.get('mapping', {})
        storage_config = self.config.get('storage', {})
        
        # Map type
        self.map_type = mapping_config.get('map_type', 'occupancy_grid')
        
        # Occupancy grid settings
        self.grid_resolution = mapping_config.get('grid_resolution', 0.05)  # meters per cell
        self.grid_size = tuple(mapping_config.get('grid_size', [100, 100]))
        self.grid_origin = tuple(mapping_config.get('grid_origin', [50, 50]))
        
        # Probabilistic mapping
        self.prob_free = mapping_config.get('prob_free', 0.4)
        self.prob_occupied = mapping_config.get('prob_occupied', 0.6)
        self.prob_unknown = mapping_config.get('prob_unknown', 0.5)
        
        # Storage settings
        self.auto_save = storage_config.get('auto_save', True)
        self.save_interval = storage_config.get('save_interval', 60)  # seconds
        self.map_directory = Path(storage_config.get('map_directory', 'data/maps'))
        self.map_directory.mkdir(parents=True, exist_ok=True)
        
        # Initialize occupancy grid
        if self.map_type == 'occupancy_grid':
            # grid_size is [width, height], but numpy needs (rows, cols) = (height, width)
            self.occupancy_grid = np.full((self.grid_size[1], self.grid_size[0]),
                                         self.prob_unknown, dtype=np.float32)
            self.trajectory = []  # List of robot poses
        else:
            self.occupancy_grid = None
        
        # Point cloud
        self.point_cloud = []
        
        # Camera intrinsics (for depth projection)
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Last save time
        self.last_save_time = datetime.now()
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to grid coordinates.

        Args:
            x, y: World coordinates (meters)

        Returns:
            Grid coordinates (row, col) for numpy array indexing
        """
        # NumPy arrays use [row, col] indexing where:
        # - row corresponds to Y axis (vertical)
        # - col corresponds to X axis (horizontal)
        col = int(self.grid_origin[0] + x / self.grid_resolution)
        row = int(self.grid_origin[1] + y / self.grid_resolution)
        return row, col
    
    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """
        Convert grid coordinates to world coordinates.

        Args:
            row, col: Grid coordinates (numpy array indices)

        Returns:
            World coordinates (x, y) in meters
        """
        # Inverse of world_to_grid: col->x, row->y
        x = (col - self.grid_origin[0]) * self.grid_resolution
        y = (row - self.grid_origin[1]) * self.grid_resolution
        return x, y
    
    def set_camera_intrinsics(self, camera_matrix: np.ndarray, dist_coeffs: Optional[np.ndarray] = None):
        """Set camera intrinsics for depth projection."""
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
    
    def update_occupancy_grid(self, pose: RobotPose, depth_map: Optional[np.ndarray] = None,
                             camera_matrix: Optional[np.ndarray] = None):
        """
        Update occupancy grid with new pose and sensor data.
        
        Args:
            pose: Current robot pose
            depth_map: Depth map from camera
            camera_matrix: Camera intrinsics matrix (optional, uses stored if None)
        """
        if self.occupancy_grid is None:
            return
        
        # Use provided camera matrix or stored one
        if camera_matrix is not None:
            self.camera_matrix = camera_matrix
        
        # Add pose to trajectory
        self.trajectory.append(pose)

        # Update grid at robot position (free space)
        robot_row, robot_col = self.world_to_grid(pose.x, pose.y)
        # Check bounds: grid shape is (rows, cols) = (height, width)
        if 0 <= robot_row < self.occupancy_grid.shape[0] and 0 <= robot_col < self.occupancy_grid.shape[1]:
            # Robot position is free
            self.occupancy_grid[robot_row, robot_col] = self.prob_free
        
        # Process depth map to update obstacles
        if depth_map is not None and self.camera_matrix is not None:
            self._update_grid_from_depth(pose, depth_map)
    
    def _update_grid_from_depth(self, pose: RobotPose, depth_map: np.ndarray):
        """Update occupancy grid from depth map using raycasting."""
        if self.camera_matrix is None:
            return

        # Debug logging for pose
        if not hasattr(self, '_map_update_count'):
            self._map_update_count = 0

        if self._map_update_count == 0:
            print(f"[MAP] Starting map updates with robot at pose({pose.x:.2f}, {pose.y:.2f}, {np.degrees(pose.theta):.1f}deg)")

        self._map_update_count += 1

        # Ensure camera_matrix is a numpy array
        camera_matrix = self.camera_matrix
        if not isinstance(camera_matrix, np.ndarray):
            camera_matrix = np.array(camera_matrix)

        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]

        # Sample depth points (every Nth pixel for performance)
        step = max(1, min(depth_map.shape[0] // 50, depth_map.shape[1] // 50))

        # Pre-compute rotation for this pose
        cos_theta = np.cos(pose.theta)
        sin_theta = np.sin(pose.theta)

        # Log pose every 20 updates
        if self._map_update_count % 20 == 1:
            print(f"[MAP] Update #{self._map_update_count}: robot at ({pose.x:.2f}, {pose.y:.2f}, {np.degrees(pose.theta):.1f}deg) cos={cos_theta:.3f} sin={sin_theta:.3f}")

        for y in range(0, depth_map.shape[0], step):
            for x in range(0, depth_map.shape[1], step):
                depth = depth_map[y, x]

                # Skip invalid depth
                # Minimum 0.3m to avoid detecting robot chassis/camera mount
                if depth < 0.3 or depth > 10.0:
                    continue

                # Convert pixel to 3D point in camera frame
                # Camera frame: X right, Y down, Z forward
                x_cam = (x - cx) * depth / fx
                y_cam = (y - cy) * depth / fy
                z_cam = depth

                # Transform to world frame (robot frame)
                # Robot frame: X forward, Y left, Z up
                # Camera is mounted on robot, assume camera forward = robot forward
                # For 2D mapping, we project to XY plane

                # Transform to world coordinates
                # Proper 2D rotation: camera forward(z)->robot_x, camera right(x)->-robot_y
                x_world = pose.x + z_cam * cos_theta + x_cam * sin_theta
                y_world = pose.y + z_cam * sin_theta - x_cam * cos_theta
                
                # Convert to grid coordinates
                obs_row, obs_col = self.world_to_grid(x_world, y_world)

                # Diagnostic logging for first few points (only once per session)
                if not hasattr(self, '_debug_logged'):
                    if not hasattr(self, '_debug_count'):
                        self._debug_count = 0

                    if self._debug_count < 3:  # Log first 3 points only
                        print(f"[MAP] Depth point #{self._debug_count}: " +
                              f"cam_px({x}, {y}, depth={depth:.2f}m) → " +
                              f"cam3d({x_cam:.2f}, {y_cam:.2f}, {z_cam:.2f}m) → " +
                              f"world({x_world:.2f}, {y_world:.2f}m) → " +
                              f"grid(row={obs_row}, col={obs_col}) | " +
                              f"Robot: pose({pose.x:.2f}, {pose.y:.2f}m, {np.degrees(pose.theta):.1f}deg)")
                        self._debug_count += 1
                        if self._debug_count >= 3:
                            self._debug_logged = True
                            print("[MAP] Coordinate transformation diagnostics complete")

                # Raycast from robot to obstacle
                robot_row, robot_col = self.world_to_grid(pose.x, pose.y)

                # Mark cells along ray as free
                self._raycast_free_space(robot_row, robot_col, obs_row, obs_col)

                # Mark obstacle cell as occupied
                # Check bounds using actual grid shape (rows, cols)
                if 0 <= obs_row < self.occupancy_grid.shape[0] and 0 <= obs_col < self.occupancy_grid.shape[1]:
                    # Update with probabilistic occupancy
                    current_prob = self.occupancy_grid[obs_row, obs_col]
                    if current_prob < 0.5:  # Unknown or free
                        self.occupancy_grid[obs_row, obs_col] = min(1.0, current_prob + self.prob_occupied * 0.1)
                    else:  # Already occupied
                        self.occupancy_grid[obs_row, obs_col] = min(1.0, current_prob + self.prob_occupied * 0.05)
    
    def _raycast_free_space(self, row0: int, col0: int, row1: int, col1: int):
        """Mark cells along ray as free space using Bresenham's line algorithm.

        Args:
            row0, col0: Starting grid cell (robot position)
            row1, col1: Ending grid cell (obstacle position)
        """
        drow = abs(row1 - row0)
        dcol = abs(col1 - col0)
        srow = 1 if row0 < row1 else -1
        scol = 1 if col0 < col1 else -1
        err = dcol - drow

        row, col = row0, col0

        while True:
            # Mark cell as free (but don't mark the obstacle itself)
            if ((row, col) != (row1, col1) and
                0 <= row < self.occupancy_grid.shape[0] and
                0 <= col < self.occupancy_grid.shape[1]):

                current_prob = self.occupancy_grid[row, col]
                # Conservative free space marking - don't aggressively clear occupied cells
                if current_prob >= 0.6:  # Definitely occupied - DON'T clear it
                    # Once marked as occupied, keep it (persistent obstacles)
                    pass
                elif current_prob >= 0.5:  # Unknown - mark as free
                    self.occupancy_grid[row, col] = max(0.0, current_prob - 0.10)
                else:  # Already free - reinforce it slightly
                    self.occupancy_grid[row, col] = max(0.0, current_prob - 0.02)

            if row == row1 and col == col1:
                break

            e2 = 2 * err
            if e2 > -drow:
                err -= drow
                col += scol
            if e2 < dcol:
                err += dcol  # Fixed: was 'drow', should be 'dcol'
                row += srow
    
    def add_point_cloud_point(self, point: MapPoint):
        """Add point to point cloud."""
        self.point_cloud.append(point)
    
    def save_map(self, filename: Optional[str] = None):
        """
        Save map to file.
        
        Args:
            filename: Output filename (None for auto-generated)
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"lukebot_map_{timestamp}.yaml"
        
        map_path = self.map_directory / filename
        
        # Save occupancy grid
        if self.occupancy_grid is not None:
            # Save as image
            img = ((1.0 - self.occupancy_grid) * 255).astype(np.uint8)
            img_path = map_path.with_suffix('.png')
            cv2.imwrite(str(img_path), img)
            
            # Save metadata as YAML
            metadata = {
                'map_type': 'occupancy_grid',
                'resolution': self.grid_resolution,
                'size': list(self.grid_size),
                'origin': list(self.grid_origin),
                'image_file': img_path.name,
                'trajectory': [(p.x, p.y, p.theta) for p in self.trajectory]
            }
            
            yaml_path = map_path.with_suffix('.yaml')
            with open(yaml_path, 'w') as f:
                yaml.dump(metadata, f, default_flow_style=False)
            
            print(f"[INFO] Map saved to {yaml_path}")
    
    def load_map(self, filename: str):
        """
        Load map from file.
        
        Args:
            filename: Map filename
        """
        map_path = self.map_directory / filename
        
        # Load YAML metadata
        yaml_path = map_path.with_suffix('.yaml')
        if yaml_path.exists():
            with open(yaml_path, 'r') as f:
                metadata = yaml.safe_load(f)
            
            # Load image
            img_path = self.map_directory / metadata['image_file']
            if img_path.exists():
                img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
                self.occupancy_grid = 1.0 - (img.astype(np.float32) / 255.0)
                
                # Update settings
                self.grid_resolution = metadata['resolution']
                self.grid_size = tuple(metadata['size'])
                self.grid_origin = tuple(metadata['origin'])
                
                # Load trajectory
                if 'trajectory' in metadata:
                    self.trajectory = [RobotPose(x, y, theta) for x, y, theta in metadata['trajectory']]
                
                print(f"[INFO] Map loaded from {yaml_path}")
    
    def get_map_image(self) -> Optional[np.ndarray]:
        """Get occupancy grid as image for visualization."""
        if self.occupancy_grid is None:
            return None
        
        # Convert to uint8 image
        img = ((1.0 - self.occupancy_grid) * 255).astype(np.uint8)
        return img
    
    def should_auto_save(self) -> bool:
        """Check if map should be auto-saved."""
        if not self.auto_save:
            return False
        
        now = datetime.now()
        elapsed = (now - self.last_save_time).total_seconds()
        return elapsed >= self.save_interval
    
    def auto_save_if_needed(self):
        """Auto-save map if needed."""
        if self.should_auto_save():
            self.save_map()
            self.last_save_time = datetime.now()

