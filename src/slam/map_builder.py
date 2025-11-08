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
            self.occupancy_grid = np.full(self.grid_size, self.prob_unknown, dtype=np.float32)
            self.trajectory = []  # List of robot poses
        else:
            self.occupancy_grid = None
        
        # Point cloud
        self.point_cloud = []
        
        # Last save time
        self.last_save_time = datetime.now()
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to grid coordinates.
        
        Args:
            x, y: World coordinates (meters)
        
        Returns:
            Grid coordinates (i, j)
        """
        i = int(self.grid_origin[0] + x / self.grid_resolution)
        j = int(self.grid_origin[1] + y / self.grid_resolution)
        return i, j
    
    def grid_to_world(self, i: int, j: int) -> Tuple[float, float]:
        """
        Convert grid coordinates to world coordinates.
        
        Args:
            i, j: Grid coordinates
        
        Returns:
            World coordinates (x, y) in meters
        """
        x = (i - self.grid_origin[0]) * self.grid_resolution
        y = (j - self.grid_origin[1]) * self.grid_resolution
        return x, y
    
    def update_occupancy_grid(self, pose: RobotPose, depth_map: Optional[np.ndarray] = None):
        """
        Update occupancy grid with new pose and sensor data.
        
        Args:
            pose: Current robot pose
            depth_map: Depth map from camera
        """
        if self.occupancy_grid is None:
            return
        
        # Add pose to trajectory
        self.trajectory.append(pose)
        
        # Update grid at robot position (free space)
        i, j = self.world_to_grid(pose.x, pose.y)
        if 0 <= i < self.grid_size[0] and 0 <= j < self.grid_size[1]:
            # Robot position is free
            self.occupancy_grid[i, j] = self.prob_free
        
        # TODO: Process depth map to update obstacles
        # For now, just mark robot path as free
        if depth_map is not None:
            # TODO: Project depth points to grid and mark obstacles
            pass
    
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

