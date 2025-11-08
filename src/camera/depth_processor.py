"""
Depth Processor for OAK-D stereo cameras using DepthAI v2.
Processes stereo images to generate depth maps.
"""

import os
import depthai as dai
import cv2
import numpy as np


class DepthProcessor:
    """Processes stereo camera data to generate depth maps."""
    
    def __init__(self, config=None):
        """
        Initialize depth processor.
        
        Args:
            config: Configuration dictionary with depth settings
        """
        self.config = config or {}
        self.pipeline = None
        self.device = None
        self.running = False
        
        # Depth settings from config
        self.preset = getattr(dai.node.StereoDepth.PresetMode, 
                             self.config.get('preset', 'HIGH_DENSITY'))
        self.left_right_check = self.config.get('left_right_check', True)
        self.subpixel = self.config.get('subpixel', False)
        self.depth_align = self.config.get('depth_align', 'CAM_A')
        self.confidence_threshold = self.config.get('confidence_threshold', 200)
        
        # Camera settings
        self.stereo_resolution = dai.MonoCameraProperties.SensorResolution.THE_400_P
        self.stereo_fps = self.config.get('fps', 30)
        
        # Output queues
        self.q_depth = None
        self.q_left = None
        self.q_right = None
    
    def create_pipeline(self):
        """Create DepthAI pipeline for stereo depth (DepthAI v2 style)."""
        os.environ['DEPTHAI_LEVEL'] = 'warn'
        
        pipeline = dai.Pipeline()
        
        # Create mono cameras for stereo
        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        
        # Configure mono cameras
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        mono_left.setResolution(self.stereo_resolution)
        mono_right.setResolution(self.stereo_resolution)
        mono_left.setFps(self.stereo_fps)
        mono_right.setFps(self.stereo_fps)
        
        # Create stereo depth node
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(self.preset)
        stereo.setLeftRightCheck(self.left_right_check)
        stereo.setSubpixel(self.subpixel)
        
        # Align depth to RGB camera if specified
        if self.depth_align == 'CAM_A':
            stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        
        # Link mono cameras to stereo
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)
        
        # Create depth output
        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)
        
        # Optional: Output left and right images
        xout_left = pipeline.create(dai.node.XLinkOut)
        xout_left.setStreamName("left")
        mono_left.out.link(xout_left.input)
        
        xout_right = pipeline.create(dai.node.XLinkOut)
        xout_right.setStreamName("right")
        mono_right.out.link(xout_right.input)
        
        return pipeline
    
    def start(self):
        """Start the depth processing pipeline."""
        if self.running:
            return False
        
        try:
            self.pipeline = self.create_pipeline()
            self.device = dai.Device(self.pipeline)
            
            # Get output queues (DepthAI v2 style)
            self.q_depth = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            self.q_left = self.device.getOutputQueue(name="left", maxSize=4, blocking=False)
            self.q_right = self.device.getOutputQueue(name="right", maxSize=4, blocking=False)
            
            self.running = True
            return True
        except Exception as e:
            print(f"[ERROR] Failed to start depth processor: {e}")
            return False
    
    def stop(self):
        """Stop the depth processing pipeline."""
        self.running = False
        if self.device:
            self.device.close()
            self.device = None
        self.pipeline = None
    
    def get_depth_frame(self):
        """Get latest depth frame (non-blocking)."""
        if not self.q_depth:
            return None
        
        try:
            in_depth = self.q_depth.tryGet()
            if in_depth is not None:
                depth_frame = in_depth.getFrame()
                return depth_frame
        except Exception as e:
            print(f"[ERROR] Error getting depth frame: {e}")
        
        return None
    
    def get_stereo_frames(self):
        """Get latest left and right stereo frames (non-blocking)."""
        if not self.q_left or not self.q_right:
            return None, None
        
        try:
            in_left = self.q_left.tryGet()
            in_right = self.q_right.tryGet()
            
            left_frame = None
            right_frame = None
            
            if in_left is not None:
                left_frame = in_left.getCvFrame()
            if in_right is not None:
                right_frame = in_right.getCvFrame()
            
            return left_frame, right_frame
        except Exception as e:
            print(f"[ERROR] Error getting stereo frames: {e}")
        
        return None, None
    
    def filter_depth(self, depth_frame, min_depth=0.1, max_depth=10.0):
        """Filter depth frame by min/max depth values."""
        if depth_frame is None:
            return None
        
        depth_frame = depth_frame.astype(np.float32) / 1000.0  # Convert to meters
        depth_frame[depth_frame < min_depth] = 0
        depth_frame[depth_frame > max_depth] = 0
        
        return depth_frame
    
    def depth_to_colormap(self, depth_frame, colormap=cv2.COLORMAP_JET):
        """Convert depth frame to color visualization."""
        if depth_frame is None:
            return None
        
        # Normalize depth to 0-255
        depth_vis = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        depth_colored = cv2.applyColorMap(depth_vis, colormap)
        
        return depth_colored

