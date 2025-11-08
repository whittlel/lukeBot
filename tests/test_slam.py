"""
Unit tests for SLAM module.
"""

import unittest
import numpy as np
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from slam.slam_engine import SLAMEngine
from slam.visual_odometry import VisualOdometry
from slam.map_builder import MapBuilder
from utils.data_structures import RobotPose


class TestVisualOdometry(unittest.TestCase):
    """Test visual odometry."""
    
    def test_vo_initialization(self):
        """Test VO initialization."""
        vo = VisualOdometry()
        self.assertIsNotNone(vo)
        self.assertIsNotNone(vo.detector)
        self.assertIsNotNone(vo.matcher)
    
    def test_feature_detection(self):
        """Test feature detection."""
        vo = VisualOdometry()
        
        # Create test image
        test_image = np.random.randint(0, 255, (480, 640), dtype=np.uint8)
        
        # Detect features
        keypoints, descriptors = vo.detect_features(test_image)
        self.assertIsInstance(keypoints, list)
        self.assertIsInstance(descriptors, (np.ndarray, type(None)))
    
    def test_pose_estimation(self):
        """Test pose estimation."""
        vo = VisualOdometry()
        
        # Create test images
        img1 = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        img2 = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # Process first frame
        pose1 = vo.estimate_pose(img1)
        self.assertIsNotNone(pose1)
        
        # Process second frame
        pose2 = vo.estimate_pose(img2)
        # May be None if not enough features matched
        if pose2 is not None:
            self.assertIsInstance(pose2, RobotPose)


class TestMapBuilder(unittest.TestCase):
    """Test map builder."""
    
    def test_map_builder_initialization(self):
        """Test map builder initialization."""
        builder = MapBuilder()
        self.assertIsNotNone(builder)
        if builder.occupancy_grid is not None:
            self.assertEqual(builder.occupancy_grid.shape, builder.grid_size)
    
    def test_world_to_grid(self):
        """Test world to grid conversion."""
        builder = MapBuilder()
        i, j = builder.world_to_grid(0.0, 0.0)
        self.assertIsInstance(i, int)
        self.assertIsInstance(j, int)
    
    def test_grid_to_world(self):
        """Test grid to world conversion."""
        builder = MapBuilder()
        x, y = builder.grid_to_world(50, 50)
        self.assertIsInstance(x, float)
        self.assertIsInstance(y, float)
    
    def test_update_occupancy_grid(self):
        """Test occupancy grid update."""
        builder = MapBuilder()
        pose = RobotPose(0.0, 0.0, 0.0)
        builder.update_occupancy_grid(pose)
        self.assertIsNotNone(builder.occupancy_grid)


class TestSLAMEngine(unittest.TestCase):
    """Test SLAM engine."""
    
    def test_slam_initialization(self):
        """Test SLAM engine initialization."""
        slam = SLAMEngine()
        self.assertIsNotNone(slam)
        self.assertIsNotNone(slam.visual_odometry)
        self.assertIsNotNone(slam.map_builder)
    
    def test_process_frame(self):
        """Test frame processing."""
        slam = SLAMEngine()
        
        # Create test image
        test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # Process frame
        pose = slam.process_frame(test_image)
        # May be None if not enough features
        if pose is not None:
            self.assertIsInstance(pose, RobotPose)


if __name__ == "__main__":
    unittest.main()

