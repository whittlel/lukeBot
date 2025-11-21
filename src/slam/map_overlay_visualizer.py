"""
Map Overlay Visualizer - Top-down map view with navigation overlays.
Provides real-time visualization of occupancy grid, robot position, trajectory, and navigation data.
"""

import cv2
import numpy as np
from typing import Optional, List, Tuple
from ..utils.data_structures import RobotPose


class MapOverlayVisualizer:
    """Visualizer for top-down map view with navigation overlays."""

    def __init__(self, config=None):
        """
        Initialize map overlay visualizer.

        Args:
            config: Configuration dictionary
        """
        self.config = config or {}

        # Visualization settings
        self.map_scale = 8  # pixels per grid cell (increased from 5 for better visibility)
        self.robot_size = 15  # pixels for robot triangle
        self.trajectory_max_points = 200  # Maximum trajectory points to display

        # Colors (BGR format)
        self.color_free = (255, 255, 255)  # White for free space
        self.color_occupied = (0, 0, 0)  # Black for occupied
        self.color_unknown = (128, 128, 128)  # Gray for unknown
        self.color_robot = (0, 255, 0)  # Green for robot
        self.color_trajectory = (255, 0, 0)  # Blue for trajectory
        self.color_grid_lines = (200, 200, 200)  # Light gray for grid

    def world_to_image_coords(self, world_x: float, world_y: float,
                             grid_origin: Tuple[int, int],
                             grid_resolution: float) -> Tuple[int, int]:
        """
        Convert world coordinates to image pixel coordinates.

        Args:
            world_x: X coordinate in meters (world frame)
            world_y: Y coordinate in meters (world frame)
            grid_origin: Origin position in grid cells [x, y]
            grid_resolution: Resolution in meters per cell

        Returns:
            Tuple of (image_x, image_y) in pixels
        """
        # Convert world to grid cells
        grid_x = int(world_x / grid_resolution) + grid_origin[0]
        grid_y = int(world_y / grid_resolution) + grid_origin[1]

        # Convert grid cells to image pixels
        image_x = grid_x * self.map_scale
        # Flip Y axis (image Y increases downward, world Y increases upward)
        image_y = (grid_origin[1] * 2 - grid_y) * self.map_scale

        return (image_x, image_y)

    def draw_occupancy_grid(self, occupancy_grid: np.ndarray) -> np.ndarray:
        """
        Draw occupancy grid as image.

        Args:
            occupancy_grid: Occupancy grid array (values 0-1, 0.5=unknown)

        Returns:
            Image representation of occupancy grid
        """
        if occupancy_grid is None or occupancy_grid.size == 0:
            # Return empty gray image
            return np.full((400, 400, 3), 128, dtype=np.uint8)

        # Create color-mapped image
        h, w = occupancy_grid.shape
        image = np.zeros((h, w, 3), dtype=np.uint8)

        # Map probabilities to colors
        # <0.4 = free (white), >0.6 = occupied (black), 0.4-0.6 = unknown (gray)
        for y in range(h):
            for x in range(w):
                prob = occupancy_grid[y, x]
                if prob < 0.4:
                    # Free space
                    image[y, x] = self.color_free
                elif prob > 0.6:
                    # Occupied
                    image[y, x] = self.color_occupied
                else:
                    # Unknown
                    image[y, x] = self.color_unknown

        # Scale up for visibility
        scaled_image = cv2.resize(image,
                                 (w * self.map_scale, h * self.map_scale),
                                 interpolation=cv2.INTER_NEAREST)

        return scaled_image

    def draw_robot_marker(self, image: np.ndarray, pose: RobotPose,
                         grid_origin: Tuple[int, int],
                         grid_resolution: float) -> np.ndarray:
        """
        Draw robot position and heading as triangle.

        Args:
            image: Image to draw on
            pose: Robot pose (x, y, theta)
            grid_origin: Grid origin in cells
            grid_resolution: Grid resolution in meters per cell

        Returns:
            Image with robot marker
        """
        if pose is None:
            return image

        output = image.copy()

        # Convert pose to image coordinates
        robot_x, robot_y = self.world_to_image_coords(pose.x, pose.y, grid_origin, grid_resolution)

        # Check if robot is within image bounds
        h, w = output.shape[:2]
        if robot_x < 0 or robot_x >= w or robot_y < 0 or robot_y >= h:
            return output

        # Draw triangle pointing in heading direction
        # Triangle points relative to center
        forward_dist = self.robot_size
        side_dist = self.robot_size // 2

        # Calculate triangle vertices in robot frame
        # Front point
        front_x = robot_x + int(forward_dist * np.cos(pose.theta))
        front_y = robot_y - int(forward_dist * np.sin(pose.theta))  # Negative because image Y is flipped

        # Back-left point
        back_angle_left = pose.theta + np.pi * 2.5 / 3  # 150 degrees from front
        back_left_x = robot_x + int(side_dist * np.cos(back_angle_left))
        back_left_y = robot_y - int(side_dist * np.sin(back_angle_left))

        # Back-right point
        back_angle_right = pose.theta - np.pi * 2.5 / 3  # -150 degrees from front
        back_right_x = robot_x + int(side_dist * np.cos(back_angle_right))
        back_right_y = robot_y - int(side_dist * np.sin(back_angle_right))

        # Draw filled triangle
        triangle_pts = np.array([[front_x, front_y],
                                [back_left_x, back_left_y],
                                [back_right_x, back_right_y]], dtype=np.int32)
        cv2.fillPoly(output, [triangle_pts], self.color_robot)

        # Draw outline
        cv2.polylines(output, [triangle_pts], True, (0, 128, 0), 2)

        # Draw center dot
        cv2.circle(output, (robot_x, robot_y), 3, (255, 255, 255), -1)

        return output

    def draw_trajectory(self, image: np.ndarray, trajectory: List[RobotPose],
                       grid_origin: Tuple[int, int],
                       grid_resolution: float) -> np.ndarray:
        """
        Draw trajectory path as breadcrumb trail.

        Args:
            image: Image to draw on
            trajectory: List of robot poses along trajectory
            grid_origin: Grid origin in cells
            grid_resolution: Grid resolution in meters per cell

        Returns:
            Image with trajectory overlay
        """
        if trajectory is None or len(trajectory) < 2:
            return image

        output = image.copy()
        h, w = output.shape[:2]

        # Limit trajectory points for performance
        if len(trajectory) > self.trajectory_max_points:
            # Sample evenly from trajectory
            indices = np.linspace(0, len(trajectory) - 1, self.trajectory_max_points, dtype=int)
            trajectory = [trajectory[i] for i in indices]

        # Convert poses to image coordinates
        points = []
        for pose in trajectory:
            x, y = self.world_to_image_coords(pose.x, pose.y, grid_origin, grid_resolution)
            # Only include points within image bounds
            if 0 <= x < w and 0 <= y < h:
                points.append((x, y))

        # Draw trajectory line segments
        for i in range(len(points) - 1):
            # Fading effect: older points are more transparent
            alpha = (i + 1) / len(points)  # 0 to 1
            color = (int(self.color_trajectory[0] * alpha),
                    int(self.color_trajectory[1] * alpha),
                    int(self.color_trajectory[2] * alpha))

            cv2.line(output, points[i], points[i + 1], color, 2)

        # Draw small circles at trajectory points
        for i, point in enumerate(points):
            if i % 5 == 0:  # Draw every 5th point
                alpha = (i + 1) / len(points)
                color = (int(self.color_trajectory[0] * alpha),
                        int(self.color_trajectory[1] * alpha),
                        int(self.color_trajectory[2] * alpha))
                cv2.circle(output, point, 2, color, -1)

        return output

    def draw_grid_lines(self, image: np.ndarray, grid_size: int = 10) -> np.ndarray:
        """
        Draw grid lines for reference.

        Args:
            image: Image to draw on
            grid_size: Grid line spacing in cells

        Returns:
            Image with grid overlay
        """
        output = image.copy()
        h, w = output.shape[:2]

        # Grid spacing in pixels
        spacing = grid_size * self.map_scale

        # Draw vertical lines
        for x in range(0, w, spacing):
            cv2.line(output, (x, 0), (x, h), self.color_grid_lines, 1)

        # Draw horizontal lines
        for y in range(0, h, spacing):
            cv2.line(output, (0, y), (w, y), self.color_grid_lines, 1)

        return output

    def draw_scale_bar(self, image: np.ndarray, grid_resolution: float) -> np.ndarray:
        """
        Draw scale bar showing distance.

        Args:
            image: Image to draw on
            grid_resolution: Grid resolution in meters per cell

        Returns:
            Image with scale bar
        """
        output = image.copy()
        h, w = output.shape[:2]

        # Scale bar properties
        bar_length_meters = 1.0  # 1 meter scale bar
        bar_length_pixels = int(bar_length_meters / grid_resolution * self.map_scale)
        bar_x = w - bar_length_pixels - 20
        bar_y = h - 40
        bar_thickness = 3

        # Draw scale bar
        cv2.line(output, (bar_x, bar_y), (bar_x + bar_length_pixels, bar_y),
                (0, 0, 0), bar_thickness + 2)  # Black outline
        cv2.line(output, (bar_x, bar_y), (bar_x + bar_length_pixels, bar_y),
                (255, 255, 255), bar_thickness)  # White bar

        # Draw end markers
        cv2.line(output, (bar_x, bar_y - 5), (bar_x, bar_y + 5), (255, 255, 255), 2)
        cv2.line(output, (bar_x + bar_length_pixels, bar_y - 5),
                (bar_x + bar_length_pixels, bar_y + 5), (255, 255, 255), 2)

        # Draw label
        cv2.putText(output, f"{bar_length_meters}m", (bar_x, bar_y - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(output, f"{bar_length_meters}m", (bar_x, bar_y - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        return output

    def create_map_view(self, occupancy_grid: np.ndarray,
                       robot_pose: RobotPose,
                       grid_origin: Tuple[int, int],
                       grid_resolution: float,
                       trajectory: Optional[List[RobotPose]] = None,
                       show_grid: bool = True,
                       show_scale: bool = True) -> np.ndarray:
        """
        Create comprehensive map visualization with all overlays.

        Args:
            occupancy_grid: Occupancy grid array
            robot_pose: Current robot pose
            grid_origin: Grid origin in cells [x, y]
            grid_resolution: Grid resolution in meters per cell
            trajectory: Optional trajectory history
            show_grid: Whether to show grid lines
            show_scale: Whether to show scale bar

        Returns:
            Complete map visualization
        """
        # Draw occupancy grid
        map_image = self.draw_occupancy_grid(occupancy_grid)

        # Draw grid lines
        if show_grid:
            map_image = self.draw_grid_lines(map_image, grid_size=10)

        # Draw trajectory
        if trajectory is not None and len(trajectory) > 0:
            map_image = self.draw_trajectory(map_image, trajectory, grid_origin, grid_resolution)

        # Draw robot
        if robot_pose is not None:
            map_image = self.draw_robot_marker(map_image, robot_pose, grid_origin, grid_resolution)

        # Draw scale bar
        if show_scale:
            map_image = self.draw_scale_bar(map_image, grid_resolution)

        # Add title
        cv2.putText(map_image, "Occupancy Map (Top-Down)", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        # Add legend
        legend_y = 60
        cv2.rectangle(map_image, (10, legend_y), (30, legend_y + 15), self.color_free, -1)
        cv2.putText(map_image, "Free", (35, legend_y + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        cv2.rectangle(map_image, (100, legend_y), (120, legend_y + 15), self.color_occupied, -1)
        cv2.rectangle(map_image, (100, legend_y), (120, legend_y + 15), (255, 255, 255), 1)
        cv2.putText(map_image, "Occupied", (125, legend_y + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        cv2.rectangle(map_image, (220, legend_y), (240, legend_y + 15), self.color_unknown, -1)
        cv2.putText(map_image, "Unknown", (245, legend_y + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        return map_image
