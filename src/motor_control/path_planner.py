"""
Path planning using A* algorithm on occupancy grid.
Generates safe paths avoiding obstacles.
"""

import numpy as np
from typing import List, Tuple, Optional, Set
from ..utils.data_structures import RobotPose
from ..utils.logger import setup_logger
import heapq


class PathPlanner:
    """A* path planner for occupancy grid."""
    
    def __init__(self, config=None):
        """
        Initialize path planner.

        Args:
            config: Configuration dictionary
        """
        self.config = config or {}
        path_config = self.config.get('path_planning', {})

        # Safety parameters - reduced for exploration
        self.obstacle_inflation = path_config.get('obstacle_inflation', 0.05)  # meters (reduced from 0.2)
        self.safety_margin = path_config.get('safety_margin', 0.05)  # meters (reduced from 0.15)

        # A* parameters
        self.heuristic_weight = path_config.get('heuristic_weight', 1.0)
        self.max_path_length = path_config.get('max_path_length', 50.0)  # meters

        # Logger
        self.logger = setup_logger("path_planner", log_file="data/logs/lukebot.log")
    
    def world_to_grid(self, x: float, y: float, grid_resolution: float,
                     grid_origin: Tuple[int, int]) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates."""
        i = int(grid_origin[0] + y / grid_resolution)
        j = int(grid_origin[1] + x / grid_resolution)
        return i, j
    
    def grid_to_world(self, i: int, j: int, grid_resolution: float,
                     grid_origin: Tuple[int, int]) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates."""
        x = (j - grid_origin[1]) * grid_resolution
        y = (i - grid_origin[0]) * grid_resolution
        return x, y
    
    def is_valid_cell(self, i: int, j: int, occupancy_grid: np.ndarray,
                     prob_free: float = 0.4, prob_occupied: float = 0.6) -> bool:
        """Check if cell is valid for path planning (free and not too close to obstacles)."""
        if i < 0 or i >= occupancy_grid.shape[0] or j < 0 or j >= occupancy_grid.shape[1]:
            return False

        # Check if cell is free or unknown (be more permissive for exploration)
        # Accept anything that's not definitely occupied (>0.6)
        prob = occupancy_grid[i, j]
        if prob > prob_occupied:
            return False

        # For exploration, don't check inflation at all - be very permissive
        # This allows planning through unknown space
        return True
    
    def heuristic(self, i1: int, j1: int, i2: int, j2: int) -> float:
        """Heuristic function for A* (Euclidean distance)."""
        return np.sqrt((i1 - i2)**2 + (j1 - j2)**2)
    
    def astar(self, start: Tuple[int, int], goal: Tuple[int, int],
             occupancy_grid: np.ndarray,
             grid_resolution: float,
             prob_free: float = 0.4,
             prob_occupied: float = 0.6) -> Optional[List[Tuple[int, int]]]:
        """
        A* path planning algorithm.

        Args:
            start: Start cell (i, j)
            goal: Goal cell (i, j)
            occupancy_grid: Occupancy grid
            grid_resolution: Grid resolution in meters
            prob_free: Probability threshold for free space
            prob_occupied: Probability threshold for occupied space

        Returns:
            List of path cells (i, j), or None if no path found
        """
        goal_prob = occupancy_grid[goal[0], goal[1]]
        if not self.is_valid_cell(goal[0], goal[1], occupancy_grid, prob_free, prob_occupied):
            self.logger.debug(f"Goal cell ({goal[0]},{goal[1]}) is invalid, prob={goal_prob:.2f}")
            return None
        
        # Priority queue: (f_score, g_score, i, j, parent)
        open_set = [(0, 0, start[0], start[1], None)]
        closed_set: Set[Tuple[int, int]] = set()
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start[0], start[1], goal[0], goal[1])}
        
        while open_set:
            current_f, current_g, i, j, parent = heapq.heappop(open_set)
            current = (i, j)
            
            if current in closed_set:
                continue
            
            closed_set.add(current)
            came_from[current] = parent
            
            # Check if goal reached
            if current == goal:
                # Reconstruct path
                path = []
                node = current
                while node is not None:
                    path.append(node)
                    node = came_from.get(node)
                path.reverse()
                return path
            
            # Check neighbors (8-connected)
            for di in [-1, 0, 1]:
                for dj in [-1, 0, 1]:
                    if di == 0 and dj == 0:
                        continue
                    
                    neighbor = (i + di, j + dj)
                    
                    if neighbor in closed_set:
                        continue
                    
                    if not self.is_valid_cell(neighbor[0], neighbor[1], occupancy_grid, 
                                             prob_free, prob_occupied):
                        continue
                    
                    # Calculate cost (diagonal movement costs more)
                    move_cost = 1.414 if di != 0 and dj != 0 else 1.0
                    tentative_g = g_score[current] + move_cost
                    
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        h = self.heuristic(neighbor[0], neighbor[1], goal[0], goal[1])
                        f_score[neighbor] = tentative_g + self.heuristic_weight * h
                        heapq.heappush(open_set, (f_score[neighbor], tentative_g, 
                                                neighbor[0], neighbor[1], current))
        
        return None  # No path found
    
    def plan_path(self, start_pose: RobotPose, goal: Tuple[float, float],
                 occupancy_grid: np.ndarray,
                 grid_resolution: float,
                 grid_origin: Tuple[int, int],
                 prob_free: float = 0.4,
                 prob_occupied: float = 0.6) -> Optional[List[Tuple[float, float]]]:
        """
        Plan path from start pose to goal.

        Args:
            start_pose: Starting robot pose
            goal: Goal position (x, y) in world coordinates
            occupancy_grid: Occupancy grid
            grid_resolution: Grid resolution in meters
            grid_origin: Grid origin (i, j)
            prob_free: Probability threshold for free space
            prob_occupied: Probability threshold for occupied space

        Returns:
            List of waypoints (x, y) in world coordinates, or None if no path found
        """
        # Convert to grid coordinates
        start_i, start_j = self.world_to_grid(start_pose.x, start_pose.y,
                                             grid_resolution, grid_origin)
        goal_i, goal_j = self.world_to_grid(goal[0], goal[1],
                                           grid_resolution, grid_origin)

        # Check bounds
        if (start_i < 0 or start_i >= occupancy_grid.shape[0] or
            start_j < 0 or start_j >= occupancy_grid.shape[1]):
            self.logger.debug(f"Start position out of bounds: ({start_i}, {start_j}), grid shape: {occupancy_grid.shape}")
            return None

        if (goal_i < 0 or goal_i >= occupancy_grid.shape[0] or
            goal_j < 0 or goal_j >= occupancy_grid.shape[1]):
            self.logger.debug(f"Goal position out of bounds: ({goal_i}, {goal_j}), grid shape: {occupancy_grid.shape}")
            return None

        # Check if start/goal are valid
        start_prob = occupancy_grid[start_i, start_j]
        goal_prob = occupancy_grid[goal_i, goal_j]
        self.logger.debug(f"Path planning: start=({start_i},{start_j}) prob={start_prob:.2f}, goal=({goal_i},{goal_j}) prob={goal_prob:.2f}")

        # Plan path using A*
        path_cells = self.astar((start_i, start_j), (goal_i, goal_j),
                               occupancy_grid, grid_resolution, prob_free, prob_occupied)

        if path_cells is None:
            self.logger.debug(f"A* failed to find path from ({start_i},{start_j}) to ({goal_i},{goal_j})")
            return None
        
        # Convert path cells to world coordinates
        waypoints = []
        for i, j in path_cells:
            x, y = self.grid_to_world(i, j, grid_resolution, grid_origin)
            waypoints.append((x, y))
        
        # Simplify path (remove redundant waypoints)
        simplified = self._simplify_path(waypoints, occupancy_grid, grid_resolution, 
                                        grid_origin, prob_occupied)
        
        return simplified
    
    def _simplify_path(self, waypoints: List[Tuple[float, float]],
                      occupancy_grid: np.ndarray,
                      grid_resolution: float,
                      grid_origin: Tuple[int, int],
                      prob_occupied: float) -> List[Tuple[float, float]]:
        """Simplify path by removing redundant waypoints."""
        if len(waypoints) <= 2:
            return waypoints
        
        simplified = [waypoints[0]]
        
        i = 0
        while i < len(waypoints) - 1:
            # Try to skip waypoints
            for j in range(len(waypoints) - 1, i, -1):
                if self._is_line_clear(waypoints[i], waypoints[j], occupancy_grid,
                                      grid_resolution, grid_origin, prob_occupied):
                    simplified.append(waypoints[j])
                    i = j
                    break
            else:
                i += 1
                if i < len(waypoints):
                    simplified.append(waypoints[i])
        
        return simplified
    
    def _is_line_clear(self, p1: Tuple[float, float], p2: Tuple[float, float],
                      occupancy_grid: np.ndarray,
                      grid_resolution: float,
                      grid_origin: Tuple[int, int],
                      prob_occupied: float) -> bool:
        """Check if line between two points is clear of obstacles."""
        # Sample points along line
        num_samples = int(np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2) / grid_resolution)
        num_samples = max(2, num_samples)
        
        for k in range(num_samples + 1):
            t = k / num_samples
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            
            i, j = self.world_to_grid(x, y, grid_resolution, grid_origin)
            
            if not self.is_valid_cell(i, j, occupancy_grid, prob_occupied=prob_occupied):
                return False
        
        return True


