"""
Data structures for robot data.
"""

from dataclasses import dataclass
from typing import List, Tuple, Optional
import numpy as np


@dataclass
class RobotPose:
    """Robot pose in 2D space."""
    x: float  # X position (meters)
    y: float  # Y position (meters)
    theta: float  # Orientation (radians)
    
    def to_array(self) -> np.ndarray:
        """Convert to numpy array [x, y, theta]."""
        return np.array([self.x, self.y, self.theta])
    
    @classmethod
    def from_array(cls, pose: np.ndarray):
        """Create from numpy array [x, y, theta]."""
        return cls(x=pose[0], y=pose[1], theta=pose[2])


@dataclass
class Detection:
    """Object detection from YOLO."""
    label: str  # Class label
    confidence: float  # Confidence score (0-1)
    bbox: Tuple[float, float, float, float]  # Bounding box (xmin, ymin, xmax, ymax)
    depth: Optional[float] = None  # Depth in meters (if available)
    
    def center(self) -> Tuple[float, float]:
        """Get center of bounding box."""
        xmin, ymin, xmax, ymax = self.bbox
        return ((xmin + xmax) / 2, (ymin + ymax) / 2)


@dataclass
class MapPoint:
    """Point in the map."""
    x: float  # X coordinate (meters)
    y: float  # Y coordinate (meters)
    z: Optional[float] = None  # Z coordinate (meters, for 3D maps)
    value: float = 0.5  # Occupancy value (0=free, 1=occupied, 0.5=unknown)
    
    def to_array(self) -> np.ndarray:
        """Convert to numpy array."""
        if self.z is not None:
            return np.array([self.x, self.y, self.z])
        return np.array([self.x, self.y])


@dataclass
class KeyFrame:
    """Keyframe for SLAM."""
    frame_id: int
    image: np.ndarray  # RGB image
    depth: Optional[np.ndarray] = None  # Depth map
    pose: Optional[RobotPose] = None  # Pose at this frame
    features: Optional[np.ndarray] = None  # Feature points
    descriptors: Optional[np.ndarray] = None  # Feature descriptors

