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
    
    # Draw trajectory
    if map_builder.trajectory:
        for i, pose in enumerate(map_builder.trajectory):
            i, j = map_builder.world_to_grid(pose.x, pose.y)
            if 0 <= i < map_builder.grid_size[0] and 0 <= j < map_builder.grid_size[1]:
                cv2.circle(map_image, (j, i), 2, (0, 255, 0), -1)
        
        # Draw start position
        start_pose = map_builder.trajectory[0]
        i, j = map_builder.world_to_grid(start_pose.x, start_pose.y)
        if 0 <= i < map_builder.grid_size[0] and 0 <= j < map_builder.grid_size[1]:
            cv2.circle(map_image, (j, i), 5, (255, 0, 0), -1)
    
    # Resize for display
    display_size = (800, 600)
    map_display = cv2.resize(map_image, display_size)
    
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

