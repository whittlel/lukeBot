"""
Panoramic camera module for OAK-D with multi-camera stitching.
Provides panoramic RGB and depth views for enhanced SLAM and pathfinding.
"""

import depthai as dai
import cv2
import numpy as np
from typing import Tuple, Optional, Dict


class PanoramicCamera:
    """
    Captures and stitches images from all three OAK-D cameras (RGB + stereo pair).
    Provides panoramic views for enhanced situational awareness in SLAM/pathfinding.
    """

    def __init__(self, enable_depth: bool = True):
        """
        Initialize panoramic camera.

        Args:
            enable_depth: Whether to compute depth from stereo cameras
        """
        self.pipeline = None
        self.device = None
        self.running = False
        self.enable_depth = enable_depth

        # Camera parameters (will be loaded from device)
        self.rgb_fov = None
        self.left_fov = None
        self.right_fov = None
        self.baseline = None
        self.camera_intrinsics = {}

        # Output queues
        self.q_rgb = None
        self.q_left = None
        self.q_right = None
        self.q_depth = None

    def create_pipeline(self) -> dai.Pipeline:
        """Create DepthAI pipeline with all three cameras and optional depth."""
        pipeline = dai.Pipeline()

        # RGB Camera (center) - CAM_A
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setVideoSize(1280, 720)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(30)

        # Left stereo camera - CAM_B
        cam_left = pipeline.create(dai.node.MonoCamera)
        cam_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        cam_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        cam_left.setFps(30)

        # Right stereo camera - CAM_C
        cam_right = pipeline.create(dai.node.MonoCamera)
        cam_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        cam_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        cam_right.setFps(30)

        # Create outputs for raw camera feeds
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.video.link(xout_rgb.input)

        xout_left = pipeline.create(dai.node.XLinkOut)
        xout_left.setStreamName("left")
        cam_left.out.link(xout_left.input)

        xout_right = pipeline.create(dai.node.XLinkOut)
        xout_right.setStreamName("right")
        cam_right.out.link(xout_right.input)

        # Create stereo depth if enabled
        if self.enable_depth:
            stereo = pipeline.create(dai.node.StereoDepth)
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
            stereo.setLeftRightCheck(True)
            stereo.setSubpixel(False)
            stereo.setExtendedDisparity(True)

            # Link stereo cameras to depth
            cam_left.out.link(stereo.left)
            cam_right.out.link(stereo.right)

            # Create depth output
            xout_depth = pipeline.create(dai.node.XLinkOut)
            xout_depth.setStreamName("depth")
            stereo.depth.link(xout_depth.input)

        return pipeline

    def start(self) -> bool:
        """Start camera streams."""
        if self.running:
            return False

        try:
            self.pipeline = self.create_pipeline()
            self.device = dai.Device(self.pipeline)

            # Get output queues
            self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            self.q_left = self.device.getOutputQueue(name="left", maxSize=4, blocking=False)
            self.q_right = self.device.getOutputQueue(name="right", maxSize=4, blocking=False)

            if self.enable_depth:
                self.q_depth = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)

            # Get camera calibration data
            calib = self.device.readCalibration()

            # Get camera FOVs
            self.rgb_fov = calib.getFov(dai.CameraBoardSocket.CAM_A)
            self.left_fov = calib.getFov(dai.CameraBoardSocket.CAM_B)
            self.right_fov = calib.getFov(dai.CameraBoardSocket.CAM_C)

            # Get baseline
            self.baseline = calib.getBaselineDistance()

            # Get intrinsics
            self.camera_intrinsics['rgb'] = np.array(
                calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A)
            )
            self.camera_intrinsics['left'] = np.array(
                calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B)
            )
            self.camera_intrinsics['right'] = np.array(
                calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_C)
            )

            print(f"[INFO] Panoramic camera started")
            print(f"  RGB FOV: {self.rgb_fov:.1f}°")
            print(f"  Left FOV: {self.left_fov:.1f}°")
            print(f"  Right FOV: {self.right_fov:.1f}°")
            print(f"  Baseline: {self.baseline:.2f} cm")

            self.running = True
            return True
        except Exception as e:
            print(f"[ERROR] Failed to start panoramic camera: {e}")
            return False

    def stop(self):
        """Stop camera streams."""
        self.running = False
        if self.device:
            self.device.close()
            self.device = None
        self.pipeline = None
        self.q_rgb = None
        self.q_left = None
        self.q_right = None
        self.q_depth = None

    def get_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Get frames from all cameras.

        Returns:
            Tuple of (rgb_frame, left_frame, right_frame, depth_frame)
        """
        if not self.running:
            return None, None, None, None

        try:
            rgb_frame = None
            left_frame = None
            right_frame = None
            depth_frame = None

            in_rgb = self.q_rgb.tryGet()
            if in_rgb is not None:
                rgb_frame = in_rgb.getCvFrame()

            in_left = self.q_left.tryGet()
            if in_left is not None:
                left_frame = in_left.getCvFrame()

            in_right = self.q_right.tryGet()
            if in_right is not None:
                right_frame = in_right.getCvFrame()

            if self.enable_depth and self.q_depth is not None:
                in_depth = self.q_depth.tryGet()
                if in_depth is not None:
                    depth_frame = in_depth.getFrame()

            return rgb_frame, left_frame, right_frame, depth_frame
        except Exception as e:
            print(f"[ERROR] Failed to get frames: {e}")
            return None, None, None, None

    def create_panoramic_view(
        self,
        rgb_frame: np.ndarray,
        left_frame: Optional[np.ndarray],
        right_frame: Optional[np.ndarray],
        add_labels: bool = True
    ) -> np.ndarray:
        """
        Create panoramic view from multiple camera feeds.

        Args:
            rgb_frame: RGB frame from center camera
            left_frame: Left stereo camera frame
            right_frame: Right stereo camera frame
            add_labels: Whether to add camera labels

        Returns:
            Stitched panoramic image
        """
        if rgb_frame is None:
            return None

        target_height = rgb_frame.shape[0]
        frames = []

        # Process left camera
        if left_frame is not None:
            left_bgr = cv2.cvtColor(left_frame, cv2.COLOR_GRAY2BGR)
            aspect = left_bgr.shape[1] / left_bgr.shape[0]
            new_width = int(target_height * aspect)
            left_resized = cv2.resize(left_bgr, (new_width, target_height))
            frames.append(left_resized)

        # Add RGB center
        frames.append(rgb_frame)

        # Process right camera
        if right_frame is not None:
            right_bgr = cv2.cvtColor(right_frame, cv2.COLOR_GRAY2BGR)
            aspect = right_bgr.shape[1] / right_bgr.shape[0]
            new_width = int(target_height * aspect)
            right_resized = cv2.resize(right_bgr, (new_width, target_height))
            frames.append(right_resized)

        # Stitch horizontally
        panorama = np.hstack(frames) if len(frames) > 1 else frames[0]

        # Add labels if requested
        if add_labels:
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.7
            thickness = 2

            x_offset = 0
            if left_frame is not None:
                cv2.putText(panorama, "LEFT", (x_offset + 10, 35),
                           font, font_scale, (0, 255, 255), thickness)
                x_offset += left_resized.shape[1]

            cv2.putText(panorama, "RGB", (x_offset + 10, 35),
                       font, font_scale, (0, 255, 0), thickness)
            x_offset += rgb_frame.shape[1]

            if right_frame is not None:
                cv2.putText(panorama, "RIGHT", (x_offset + 10, 35),
                           font, font_scale, (255, 255, 0), thickness)

        return panorama

    def get_panoramic_obstacle_map(
        self,
        rgb_frame: np.ndarray,
        left_frame: Optional[np.ndarray],
        right_frame: Optional[np.ndarray],
        depth_frame: Optional[np.ndarray],
        obstacle_threshold: float = 0.5
    ) -> Dict:
        """
        Create obstacle detection map from panoramic view for pathfinding.

        Args:
            rgb_frame: RGB frame
            left_frame: Left stereo frame
            right_frame: Right stereo frame
            depth_frame: Depth frame
            obstacle_threshold: Distance threshold in meters for obstacles

        Returns:
            Dictionary with obstacle information for pathfinding:
            {
                'center_obstacles': bool,
                'left_obstacles': bool,
                'right_obstacles': bool,
                'min_depth_center': float,
                'min_depth_left': float,
                'min_depth_right': float,
                'panoramic_view': np.ndarray
            }
        """
        result = {
            'center_obstacles': False,
            'left_obstacles': False,
            'right_obstacles': False,
            'min_depth_center': None,
            'min_depth_left': None,
            'min_depth_right': None,
            'panoramic_view': None
        }

        # Create panoramic view
        result['panoramic_view'] = self.create_panoramic_view(
            rgb_frame, left_frame, right_frame, add_labels=True
        )

        # Analyze depth for obstacles
        if depth_frame is not None:
            h, w = depth_frame.shape

            # Divide depth frame into three regions
            # Center region (aligned with RGB camera)
            center_region = depth_frame[:, w//3:2*w//3]

            # Convert to meters
            center_depth_m = center_region.astype(np.float32) / 1000.0

            # Filter valid depths
            valid_center = center_depth_m[(center_depth_m > 0.05) & (center_depth_m < 10.0)]

            if valid_center.size > 0:
                min_depth = np.percentile(valid_center, 10)  # 10th percentile
                result['min_depth_center'] = min_depth
                result['center_obstacles'] = min_depth < obstacle_threshold

        return result

    def get_total_fov(self) -> float:
        """
        Calculate approximate total FOV from all cameras.

        Returns:
            Total horizontal FOV in degrees
        """
        if self.rgb_fov is None:
            return 0.0

        # Approximate total FOV
        # This is a rough estimate; actual FOV depends on camera positioning
        total_fov = self.rgb_fov

        if self.left_fov is not None and self.right_fov is not None:
            # Add partial FOV from stereo cameras (they overlap with RGB)
            total_fov += (self.left_fov + self.right_fov) * 0.3  # ~30% additional coverage

        return total_fov

    def is_running(self) -> bool:
        """Check if camera is running."""
        return self.running
