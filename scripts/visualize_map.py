#!/usr/bin/env python3
"""
Visualize SLAM map.
Displays occupancy grid and trajectory.
"""

import sys
import cv2
import numpy as np
from pathlib import Path
import yaml

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from slam.map_builder import MapBuilder


def main():
    """Visualize SLAM map."""
    print("Visualizing SLAM Map...")
    
    # Load SLAM config
    config_path = Path(__file__).parent.parent / "config" / "slam_config.yaml"
    with open(config_path, 'r') as f:
        slam_config = yaml.safe_load(f)
    
    # Initialize map builder
    map_builder = MapBuilder(config=slam_config)
    
    # Find latest map
    map_dir = Path("data/maps")
    if not map_dir.exists():
        print("ERROR: No maps directory found")
        return
    
    # Find latest map file
    map_files = list(map_dir.glob("*.yaml"))
    if not map_files:
        print("ERROR: No map files found")
        return
    
    latest_map = max(map_files, key=lambda p: p.stat().st_mtime)
    
    print(f"Loading map: {latest_map.name}")
    map_builder.load_map(latest_map.name)
    
    # Get map image
    map_image = map_builder.get_map_image()
    if map_image is None:
        print("ERROR: Failed to load map image")
        return
    
    # Convert to color image for visualization
    map_color = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)
    
    # Draw trajectory
    if map_builder.trajectory:
        # Draw trajectory line
        prev_pose = None
        for pose in map_builder.trajectory:
            i, j = map_builder.world_to_grid(pose.x, pose.y)
            if 0 <= i < map_builder.grid_size[0] and 0 <= j < map_builder.grid_size[1]:
                if prev_pose is not None:
                    prev_i, prev_j = map_builder.world_to_grid(prev_pose.x, prev_pose.y)
                    if 0 <= prev_i < map_builder.grid_size[0] and 0 <= prev_j < map_builder.grid_size[1]:
                        cv2.line(map_color, (prev_j, prev_i), (j, i), (0, 255, 0), 1)
                cv2.circle(map_color, (j, i), 2, (0, 255, 0), -1)
                prev_pose = pose
        
        # Draw start position
        start_pose = map_builder.trajectory[0]
        i, j = map_builder.world_to_grid(start_pose.x, start_pose.y)
        if 0 <= i < map_builder.grid_size[0] and 0 <= j < map_builder.grid_size[1]:
            cv2.circle(map_color, (j, i), 5, (255, 0, 0), -1)
            cv2.putText(map_color, "START", (j + 5, i), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1)
        
        # Draw current position (last pose)
        if len(map_builder.trajectory) > 0:
            current_pose = map_builder.trajectory[-1]
            i, j = map_builder.world_to_grid(current_pose.x, current_pose.y)
            if 0 <= i < map_builder.grid_size[0] and 0 <= j < map_builder.grid_size[1]:
                # Draw robot as arrow
                arrow_length = 10
                end_x = int(j + arrow_length * np.cos(current_pose.theta))
                end_y = int(i + arrow_length * np.sin(current_pose.theta))
                cv2.arrowedLine(map_color, (j, i), (end_x, end_y), (0, 0, 255), 2)
                cv2.circle(map_color, (j, i), 5, (0, 0, 255), -1)
                cv2.putText(map_color, "CURRENT", (j + 5, i), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)
    
    # Draw frontiers (if available)
    try:
        from slam.exploration_planner import ExplorationPlanner
        exploration_planner = ExplorationPlanner(config=slam_config)
        frontiers = exploration_planner.find_frontiers(
            map_builder.occupancy_grid,
            map_builder.prob_free,
            map_builder.prob_occupied,
            map_builder.prob_unknown
        )
        
        # Draw frontiers
        for i, j in frontiers:
            if 0 <= i < map_builder.grid_size[0] and 0 <= j < map_builder.grid_size[1]:
                cv2.circle(map_color, (j, i), 1, (255, 255, 0), -1)
    except Exception as e:
        print(f"Could not draw frontiers: {e}")
    
    map_image = map_color
    
    # Add legend
    legend_height = 50
    legend = np.zeros((legend_height, map_image.shape[1], 3), dtype=np.uint8)
    cv2.putText(legend, "Free", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(legend, "Occupied", (100, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    cv2.putText(legend, "Unknown", (200, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
    cv2.putText(legend, "Trajectory", (300, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.putText(legend, "Frontiers", (400, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    
    map_with_legend = np.vstack([map_image, legend])
    
    # Resize for display
    display_size = (800, 600)
    map_display = cv2.resize(map_with_legend, display_size)
    
    print("Map loaded. Press 'q' to quit.")
    
    # Display map
    cv2.imshow("SLAM Map", map_display)
    
    while True:
        key = cv2.waitKey(0) & 0xFF
        if key == ord('q'):
            break
    
    cv2.destroyAllWindows()
    print("Map visualization completed")


if __name__ == "__main__":
    main()

