"""
Camera module for OAK-D IOT-75 integration.
Handles YOLO detection, RGB, and depth streams.
"""

from .oakd_camera import OakDCamera
from .yolo_detector import YOLODetector
from .depth_processor import DepthProcessor

__all__ = ['OakDCamera', 'YOLODetector', 'DepthProcessor']

