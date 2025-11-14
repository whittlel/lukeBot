"""
Obstacle avoidance module with YOLO integration for dynamic obstacles.
Handles static and dynamic obstacle avoidance.
"""

import numpy as np
from typing import List, Tuple, Optional, Dict
from ..utils.data_structures import RobotPose


class ObstacleAvoidance:
    """Obstacle avoidance with YOLO integration."""
    
    def __init__(self, config=None):
        """
        Initialize obstacle avoidance.
        
        Args:
            config: Configuration dictionary
        """
        self.config = config or {}
        avoidance_config = self.config.get('obstacle_avoidance', {})
        
        # Safety parameters
        self.min_safe_distance = avoidance_config.get('min_safe_distance', 0.5)  # meters
        self.emergency_stop_distance = avoidance_config.get('emergency_stop_distance', 0.3)  # meters
        self.obstacle_inflation_radius = avoidance_config.get('obstacle_inflation_radius', 0.2)  # meters
        
        # YOLO integration
        self.use_yolo = avoidance_config.get('use_yolo', True)
        self.human_class_id = avoidance_config.get('human_class_id', 0)  # COCO person class (class 0)
        self.yolo_confidence_threshold = avoidance_config.get('yolo_confidence_threshold', 0.5)
        
        # Dynamic obstacles (from YOLO)
        self.dynamic_obstacles: List[Dict] = []
    
    def update_dynamic_obstacles(self, detections: List, depth_map: Optional[np.ndarray] = None,
                                 camera_matrix: Optional[np.ndarray] = None):
        """
        Update dynamic obstacles from YOLO detections.
        
        Args:
            detections: List of YOLO detections
            depth_map: Depth map for 3D position estimation
            camera_matrix: Camera intrinsics matrix
        """
        self.dynamic_obstacles = []
        
        if not self.use_yolo or not detections:
            return
        
        if depth_map is None or camera_matrix is None:
            return

        # Ensure camera_matrix is a numpy array
        if not isinstance(camera_matrix, np.ndarray):
            camera_matrix = np.array(camera_matrix)

        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        
        for detection in detections:
            # Check if it's a person (human) - COCO class 0 is person
            if hasattr(detection, 'label'):
                label_id = int(detection.label)
                # COCO dataset: class 0 = person
                if label_id != 0:  # Only track humans
                    continue
            
            # Check confidence
            if hasattr(detection, 'confidence'):
                if detection.confidence < self.yolo_confidence_threshold:
                    continue
            
            # Get bounding box center
            if hasattr(detection, 'xmin') and hasattr(detection, 'ymin') and \
               hasattr(detection, 'xmax') and hasattr(detection, 'ymax'):
                x_center = int((detection.xmin + detection.xmax) / 2 * depth_map.shape[1])
                y_center = int((detection.ymin + detection.ymax) / 2 * depth_map.shape[0])
                
                # Get depth at center
                if 0 <= y_center < depth_map.shape[0] and 0 <= x_center < depth_map.shape[1]:
                    depth = depth_map[y_center, x_center]
                    
                    if depth > 0.1 and depth < 10.0:
                        # Convert to 3D position (in camera frame)
                        x_cam = (x_center - cx) * depth / fx
                        y_cam = (y_center - cy) * depth / fy
                        z_cam = depth
                        
                        # Store as dynamic obstacle
                        self.dynamic_obstacles.append({
                            'position': (x_cam, y_cam, z_cam),
                            'depth': depth,
                            'confidence': detection.confidence if hasattr(detection, 'confidence') else 1.0,
                            'type': 'human'
                        })
    
    def check_collision(self, pose: RobotPose, occupancy_grid: np.ndarray,
                       grid_resolution: float,
                       grid_origin: Tuple[int, int],
                       prob_occupied: float = 0.6) -> bool:
        """
        Check if robot pose is in collision with obstacles.
        
        Args:
            pose: Robot pose to check
            occupancy_grid: Occupancy grid
            grid_resolution: Grid resolution in meters
            grid_origin: Grid origin (i, j)
            prob_occupied: Probability threshold for occupied space
        
        Returns:
            True if in collision, False otherwise
        """
        # Check static obstacles in occupancy grid
        i = int(grid_origin[0] + pose.y / grid_resolution)
        j = int(grid_origin[1] + pose.x / grid_resolution)
        
        if 0 <= i < occupancy_grid.shape[0] and 0 <= j < occupancy_grid.shape[1]:
            if occupancy_grid[i, j] >= prob_occupied:
                return True
        
        # Check dynamic obstacles
        for obstacle in self.dynamic_obstacles:
            # Calculate distance to obstacle (in XY plane for 2D)
            obs_x, obs_y, obs_z = obstacle['position']
            # Transform to robot frame (simplified - assume camera forward = robot forward)
            distance = np.sqrt(obs_z**2 + obs_x**2)  # Distance in XY plane
            
            if distance < self.min_safe_distance:
                return True
        
        return False
    
    def get_safe_direction(self, current_pose: RobotPose,
                         occupancy_grid: np.ndarray,
                         grid_resolution: float,
                         grid_origin: Tuple[int, int],
                         prob_occupied: float = 0.6) -> Optional[Tuple[float, float]]:
        """
        Get safe direction to move (local obstacle avoidance).
        
        Args:
            current_pose: Current robot pose
            occupancy_grid: Occupancy grid
            grid_resolution: Grid resolution in meters
            grid_origin: Grid origin (i, j)
            prob_occupied: Probability threshold for occupied space
        
        Returns:
            Safe direction vector (vx, vy), or None if no safe direction
        """
        # Check immediate surroundings
        directions = [
            (1.0, 0.0),   # Forward
            (0.707, 0.707),   # Forward-left
            (0.707, -0.707),  # Forward-right
            (0.0, 1.0),   # Left
            (0.0, -1.0),  # Right
            (-0.707, 0.707),  # Back-left
            (-0.707, -0.707), # Back-right
            (-1.0, 0.0),  # Backward
        ]
        
        check_distance = self.min_safe_distance
        
        for vx, vy in directions:
            # Rotate direction by robot orientation
            cos_theta = np.cos(current_pose.theta)
            sin_theta = np.sin(current_pose.theta)
            
            vx_rot = vx * cos_theta - vy * sin_theta
            vy_rot = vx * sin_theta + vy * cos_theta
            
            # Check position
            check_x = current_pose.x + vx_rot * check_distance
            check_y = current_pose.y + vy_rot * check_distance
            
            # Check if safe
            i = int(grid_origin[0] + check_y / grid_resolution)
            j = int(grid_origin[1] + check_x / grid_resolution)
            
            if 0 <= i < occupancy_grid.shape[0] and 0 <= j < occupancy_grid.shape[1]:
                if occupancy_grid[i, j] < prob_occupied:
                    # Check dynamic obstacles
                    safe = True
                    for obstacle in self.dynamic_obstacles:
                        obs_x, obs_y, obs_z = obstacle['position']
                        dist = np.sqrt((check_x - obs_x)**2 + (check_y - obs_y)**2)
                        if dist < self.min_safe_distance:
                            safe = False
                            break
                    
                    if safe:
                        return (vx_rot, vy_rot)
        
        return None
    
    def should_emergency_stop(self, current_pose: RobotPose) -> bool:
        """
        Check if robot should emergency stop.
        
        Args:
            current_pose: Current robot pose
        
        Returns:
            True if emergency stop needed
        """
        for obstacle in self.dynamic_obstacles:
            obs_x, obs_y, obs_z = obstacle['position']
            distance = np.sqrt(obs_z**2 + obs_x**2)
            
            if distance < self.emergency_stop_distance:
                return True
        
        return False

