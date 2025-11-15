"""
Frontier-based exploration planner for autonomous mapping.
Detects unexplored frontiers and selects best targets for exploration.
"""

import numpy as np
from typing import List, Tuple, Optional
from ..utils.data_structures import RobotPose
from ..utils.logger import setup_logger


class ExplorationPlanner:
    """Frontier-based exploration planner."""
    
    def __init__(self, config=None):
        """
        Initialize exploration planner.

        Args:
            config: Configuration dictionary
        """
        self.config = config or {}
        exploration_config = self.config.get('exploration', {})

        # Frontier detection parameters
        self.min_frontier_size = exploration_config.get('min_frontier_size', 3)  # Reduced from 5 to 3
        self.max_frontier_distance = exploration_config.get('max_frontier_distance', 10.0)
        self.exploration_radius = exploration_config.get('exploration_radius', 2.0)

        # Information gain parameters
        self.distance_weight = exploration_config.get('distance_weight', 0.3)
        self.information_weight = exploration_config.get('information_weight', 0.7)

        # Logger
        self.logger = setup_logger("exploration_planner", log_file="data/logs/lukebot.log")
    
    def find_frontiers(self, occupancy_grid: np.ndarray, 
                      prob_free: float = 0.4, 
                      prob_occupied: float = 0.6,
                      prob_unknown: float = 0.5) -> List[Tuple[int, int]]:
        """
        Find frontiers (boundaries between free and unknown space).
        
        Args:
            occupancy_grid: Occupancy grid (2D array)
            prob_free: Probability threshold for free space
            prob_occupied: Probability threshold for occupied space
            prob_unknown: Probability threshold for unknown space
        
        Returns:
            List of frontier cell coordinates (i, j)
        """
        frontiers = []
        height, width = occupancy_grid.shape
        
        # Check each cell
        for i in range(1, height - 1):
            for j in range(1, width - 1):
                # Current cell must be free
                if occupancy_grid[i, j] < prob_free:
                    # Check neighbors for unknown cells
                    is_frontier = False
                    
                    for di in [-1, 0, 1]:
                        for dj in [-1, 0, 1]:
                            if di == 0 and dj == 0:
                                continue
                            
                            ni, nj = i + di, j + dj
                            if 0 <= ni < height and 0 <= nj < width:
                                # If neighbor is unknown, this is a frontier
                                if prob_unknown - 0.1 < occupancy_grid[ni, nj] < prob_unknown + 0.1:
                                    is_frontier = True
                                    break
                        
                        if is_frontier:
                            break
                    
                    if is_frontier:
                        frontiers.append((i, j))
        
        return frontiers
    
    def cluster_frontiers(self, frontiers: List[Tuple[int, int]], 
                         cluster_distance: float = 2.0) -> List[List[Tuple[int, int]]]:
        """
        Cluster nearby frontiers together.
        
        Args:
            frontiers: List of frontier cell coordinates
            cluster_distance: Maximum distance for clustering (in grid cells)
        
        Returns:
            List of frontier clusters
        """
        if not frontiers:
            return []
        
        clusters = []
        used = set()
        
        for frontier in frontiers:
            if frontier in used:
                continue
            
            # Start new cluster
            cluster = [frontier]
            used.add(frontier)
            
            # Find nearby frontiers
            for other in frontiers:
                if other in used:
                    continue
                
                # Calculate distance
                dist = np.sqrt((frontier[0] - other[0])**2 + (frontier[1] - other[1])**2)
                if dist <= cluster_distance:
                    cluster.append(other)
                    used.add(other)
            
            if len(cluster) >= self.min_frontier_size:
                clusters.append(cluster)
        
        return clusters
    
    def select_best_frontier(self, frontier_clusters: List[List[Tuple[int, int]]],
                           current_pose: RobotPose,
                           grid_resolution: float,
                           grid_origin: Tuple[int, int]) -> Optional[Tuple[float, float]]:
        """
        Select best frontier to explore based on distance and information gain.
        
        Args:
            frontier_clusters: List of frontier clusters
            current_pose: Current robot pose
            grid_resolution: Grid resolution in meters
            grid_origin: Grid origin (i, j)
        
        Returns:
            Target waypoint (x, y) in world coordinates, or None
        """
        if not frontier_clusters:
            return None
        
        best_score = -np.inf
        best_waypoint = None
        
        for cluster in frontier_clusters:
            # Calculate cluster center
            cluster_i = np.mean([f[0] for f in cluster])
            cluster_j = np.mean([f[1] for f in cluster])
            
            # Convert to world coordinates
            x = (cluster_j - grid_origin[1]) * grid_resolution
            y = (cluster_i - grid_origin[0]) * grid_resolution
            
            # Calculate distance from current pose
            distance = np.sqrt((x - current_pose.x)**2 + (y - current_pose.y)**2)
            
            # Skip if too far
            if distance > self.max_frontier_distance:
                continue
            
            # Information gain (size of cluster)
            information_gain = len(cluster)
            
            # Score = weighted combination of distance and information
            # Lower distance and higher information = better
            distance_score = 1.0 / (1.0 + distance)
            information_score = information_gain / 100.0  # Normalize
            
            score = self.distance_weight * distance_score + self.information_weight * information_score
            
            if score > best_score:
                best_score = score
                best_waypoint = (x, y)
        
        return best_waypoint
    
    def get_exploration_waypoint(self, occupancy_grid: np.ndarray,
                                current_pose: RobotPose,
                                grid_resolution: float,
                                grid_origin: Tuple[int, int],
                                prob_free: float = 0.4,
                                prob_occupied: float = 0.6,
                                prob_unknown: float = 0.5) -> Optional[Tuple[float, float]]:
        """
        Get next exploration waypoint.
        
        Args:
            occupancy_grid: Occupancy grid
            current_pose: Current robot pose
            grid_resolution: Grid resolution in meters
            grid_origin: Grid origin (i, j)
            prob_free: Probability threshold for free space
            prob_occupied: Probability threshold for occupied space
            prob_unknown: Probability threshold for unknown space
        
        Returns:
            Target waypoint (x, y) in world coordinates, or None
        """
        # Find frontiers
        frontiers = self.find_frontiers(occupancy_grid, prob_free, prob_occupied, prob_unknown)

        self.logger.debug(f"Found {len(frontiers)} frontier cells")

        if not frontiers:
            return None

        # Cluster frontiers
        clusters = self.cluster_frontiers(frontiers)

        self.logger.debug(f"Clustered into {len(clusters)} frontier clusters")

        if not clusters:
            self.logger.debug(f"No clusters large enough (min size: {self.min_frontier_size})")
            return None
        
        # Select best frontier
        waypoint = self.select_best_frontier(clusters, current_pose, grid_resolution, grid_origin)
        
        return waypoint


