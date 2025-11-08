#!/usr/bin/env python3
"""
Test script for YOLO detection.
Tests YOLO detection on OAK-D camera.
"""

import sys
import cv2
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from camera.oakd_camera import OakDCamera


def main():
    """Test YOLO detection."""
    print("Testing YOLO Detection...")
    
    # Initialize camera
    camera = OakDCamera()
    
    # Start camera
    if not camera.start():
        print("ERROR: Failed to start camera")
        return
    
    print("YOLO detection started. Press 'q' to quit.")
    
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
            
            # Display detections
            detections_text = f"Detections: {len(data['detections'])}"
            cv2.putText(frame, detections_text, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Print detection details
            for i, det in enumerate(data['detections']):
                label_id = int(det.label)
                label = camera.yolo_detector.labels[label_id] if label_id < len(camera.yolo_detector.labels) else f"Class {label_id}"
                confidence = det.confidence
                print(f"Detection {i+1}: {label} ({confidence:.2f})")
            
            # Display frame
            cv2.imshow("YOLO Detection Test", frame)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        camera.stop()
        cv2.destroyAllWindows()
        print("YOLO detection test completed")


if __name__ == "__main__":
    main()

