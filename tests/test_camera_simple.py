#!/usr/bin/env python3
"""
Test script for OAK-D camera.
Tests RGB, depth, and YOLO detection.
"""

import sys
import cv2
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from camera.oakd_camera import OakDCamera


def main():
    """Test camera functionality."""
    print("Testing OAK-D Camera...")
    
    # Initialize camera
    camera = OakDCamera()
    
    # Start camera
    if not camera.start():
        print("ERROR: Failed to start camera")
        return
    
    print("Camera started. Press 'q' to quit.")
    
    try:
        while True:
            # Get camera data
            data = camera.get_all_data()
            
            if data['rgb'] is None:
                continue
            
            # Draw detections
            frame = camera.draw_detections(data['rgb'], data['detections'])
            
            # Display FPS
            cv2.putText(frame, f"FPS: {data['fps']:.1f}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Display detections count
            detections_text = f"Detections: {len(data['detections'])}"
            cv2.putText(frame, detections_text, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Display depth info if available
            if data['depth'] is not None:
                depth_text = f"Depth: {data['depth'].shape}"
                cv2.putText(frame, depth_text, (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Display frame
            cv2.imshow("OAK-D Camera Test", frame)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        camera.stop()
        cv2.destroyAllWindows()
        print("Camera test completed")


if __name__ == "__main__":
    main()

