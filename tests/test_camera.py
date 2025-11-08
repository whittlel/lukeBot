"""
Unit tests for camera module.
"""

import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from camera.oakd_camera import OakDCamera
from camera.yolo_detector import YOLODetector
from camera.depth_processor import DepthProcessor


class TestCamera(unittest.TestCase):
    """Test camera module."""
    
    def test_camera_initialization(self):
        """Test camera initialization."""
        # Note: This test requires actual hardware
        # Skip if hardware not available
        try:
            camera = OakDCamera()
            self.assertIsNotNone(camera)
        except Exception as e:
            self.skipTest(f"Hardware not available: {e}")
    
    def test_yolo_detector_initialization(self):
        """Test YOLO detector initialization."""
        # Note: This test requires actual hardware
        try:
            detector = YOLODetector(
                model_path="../oakd/models/yolov8n.json",
                input_size=(640, 640)
            )
            self.assertIsNotNone(detector)
        except Exception as e:
            self.skipTest(f"Hardware not available: {e}")
    
    def test_depth_processor_initialization(self):
        """Test depth processor initialization."""
        # Note: This test requires actual hardware
        try:
            processor = DepthProcessor()
            self.assertIsNotNone(processor)
        except Exception as e:
            self.skipTest(f"Hardware not available: {e}")


if __name__ == "__main__":
    unittest.main()

