"""
OAK-D Camera wrapper that combines YOLO detection, RGB, and depth streams.
Uses DepthAI v2 API with a single combined pipeline.
"""

import os
import yaml
import json
import threading
import queue
import time
from pathlib import Path
import depthai as dai
import cv2
import numpy as np

from .yolo_detector import COCO_LABELS


class OakDCamera:
    """Main camera wrapper for OAK-D IOT-75 with YOLO, RGB, and depth."""
    
    def __init__(self, config_path=None, config=None):
        """
        Initialize OAK-D camera.
        
        Args:
            config_path: Path to camera config YAML file
            config: Configuration dictionary (overrides config_path)
        """
        # Load configuration
        if config is None:
            if config_path is None:
                config_path = Path(__file__).parent.parent.parent / "config" / "camera_config.yaml"
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
        
        self.config = config
        self.camera_config = config.get('camera', {})
        self.yolo_config = config.get('yolo', {})
        self.depth_config = config.get('depth', {})
        
        # Model path
        model_path = self.yolo_config.get('model_path', 'data/models/yolov6n/yolov6n_openvino_2022.1_6shave.blob')
        if not Path(model_path).is_absolute():
            project_root = Path(__file__).parent.parent.parent
            model_path = project_root / model_path
        
        self.model_path = self._resolve_model_path(model_path)
        
        # Get input size from config or JSON
        input_size = tuple(self.yolo_config.get('input_size', [640, 352]))
        model_json_path = Path(self.model_path).parent / "yolov6n.json"
        if model_json_path.exists():
            try:
                with open(model_json_path, 'r') as f:
                    model_metadata = json.load(f)
                    input_size_str = model_metadata.get('nn_config', {}).get('input_size', '')
                    if input_size_str:
                        try:
                            w, h = map(int, input_size_str.split('x'))
                            input_size = (w, h)
                        except:
                            pass
            except Exception as e:
                print(f"[WARNING] Could not load model metadata: {e}")
        
        self.input_size = input_size
        self.video_size = (1280, 720)
        self.confidence_threshold = self.yolo_config.get('confidence_threshold', 0.5)
        self.labels = COCO_LABELS
        
        # Pipeline and device
        self.pipeline = None
        self.device = None
        self.running = False
        
        # Output queues
        self.q_video = None
        self.q_nn = None
        self.q_depth = None
        
        # Frame queues
        self.frame_queue = queue.Queue(maxsize=2)
        self.detection_queue = queue.Queue(maxsize=2)
        self.depth_queue = queue.Queue(maxsize=2)
        self.imu_queue = queue.Queue(maxsize=2)
        
        # FPS tracking
        self.fps = 0.0
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.process_thread = None
        
        # Camera intrinsics (will be loaded from device)
        self.camera_intrinsics = None
        self.distortion_coeffs = None
        
        # IMU enable flag
        self.enable_imu = self.depth_config.get('enable_imu', True)
        self.enable_extended_disparity = self.depth_config.get('extended_disparity', True)
    
    def _resolve_model_path(self, model_path):
        """Resolve model path - handle .json files and find .blob files."""
        path = Path(model_path)
        
        # If path doesn't exist, try to find it
        if not path.exists():
            # Try to find .blob file with same name
            if path.suffix == '.json':
                blob_file = path.with_suffix('.blob')
                if blob_file.exists():
                    return str(blob_file)
            # Try looking in data/models
            project_root = Path(__file__).parent.parent.parent
            alt_path = project_root / "data" / "models" / path.name
            if alt_path.exists():
                return str(alt_path)
            # Try in subdirectories
            models_dir = project_root / "data" / "models"
            if models_dir.exists():
                for subdir in models_dir.iterdir():
                    if subdir.is_dir():
                        alt_path = subdir / path.name
                        if alt_path.exists():
                            return str(alt_path)
        
        # Handle .json files that contain blob path
        if path.suffix == '.json' and path.exists():
            try:
                with open(path, 'r') as f:
                    model_info = json.load(f)
                    blob_path = model_info.get('blob', None)
                    if blob_path:
                        blob_file = path.parent / blob_path
                        if blob_file.exists():
                            return str(blob_file)
                        if Path(blob_path).exists():
                            return blob_path
            except Exception:
                pass
            # Fallback: look for .blob file with same name
            blob_file = path.with_suffix('.blob')
            if blob_file.exists():
                return str(blob_file)
        
        return str(path)
    
    def create_pipeline(self):
        """Create combined DepthAI pipeline with YOLO and depth (DepthAI v2 style)."""
        os.environ['DEPTHAI_LEVEL'] = 'warn'
        
        # Verify model path
        model_path = Path(self.model_path)
        if not model_path.exists():
            raise FileNotFoundError(f"Model file not found: {self.model_path}")
        if not model_path.suffix == '.blob':
            raise ValueError(f"Model file must be a .blob file, got: {model_path.suffix}")
        
        pipeline = dai.Pipeline()
        
        # Create RGB camera node (DepthAI v2 style - ColorCamera)
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setPreviewSize(*self.input_size)
        cam.setVideoSize(*self.video_size)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setFps(30)
        
        # Create stereo cameras for depth
        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setFps(30)
        mono_right.setFps(30)
        
        # Create stereo depth node
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(False)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        
        # Enable Extended Disparity Mode for better close-range depth
        if self.enable_extended_disparity:
            stereo.setExtendedDisparity(True)
        
        # Link mono cameras to stereo
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)
        
        # Create YOLO spatial detection network (includes depth information)
        detection_nn = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
        detection_nn.setBlobPath(str(model_path))
        detection_nn.setConfidenceThreshold(self.confidence_threshold)
        detection_nn.input.setBlocking(False)
        detection_nn.setNumClasses(80)
        detection_nn.setCoordinateSize(4)
        # YOLO anchors for COCO
        detection_nn.setAnchors([10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319])
        detection_nn.setAnchorMasks({"side26": [1, 2, 3], "side13": [3, 4, 5]})
        detection_nn.setIouThreshold(0.5)
        
        # Link camera preview to neural network
        cam.preview.link(detection_nn.input)
        
        # Link stereo depth to spatial detection network (for depth in bounding boxes)
        stereo.depth.link(detection_nn.inputDepth)
        
        # Create outputs
        xout_video = pipeline.create(dai.node.XLinkOut)
        xout_video.setStreamName("video")
        cam.video.link(xout_video.input)
        
        xout_nn = pipeline.create(dai.node.XLinkOut)
        xout_nn.setStreamName("nn")
        detection_nn.out.link(xout_nn.input)
        
        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)
        
        # Add IMU node if enabled
        if self.enable_imu:
            imu = pipeline.create(dai.node.IMU)
            imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, 
                                dai.IMUSensor.GYROSCOPE_RAW,
                                dai.IMUSensor.MAGNETOMETER_RAW], 400)
            imu.setBatchReportThreshold(1)
            imu.setMaxBatchReports(10)
            
            xout_imu = pipeline.create(dai.node.XLinkOut)
            xout_imu.setStreamName("imu")
            imu.out.link(xout_imu.input)
        
        return pipeline
    
    def start(self):
        """Start camera streams."""
        if self.running:
            return False
        
        try:
            self.pipeline = self.create_pipeline()
            self.device = dai.Device(self.pipeline)
            
            # Get output queues (DepthAI v2 style)
            self.q_video = self.device.getOutputQueue(name="video", maxSize=4, blocking=False)
            self.q_nn = self.device.getOutputQueue(name="nn", maxSize=4, blocking=False)
            self.q_depth = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            
            if self.enable_imu:
                self.q_imu = self.device.getOutputQueue(name="imu", maxSize=4, blocking=False)
            else:
                self.q_imu = None
            
            # Get camera intrinsics from device
            try:
                calib_data = self.device.readCalibration()
                self.camera_intrinsics = calib_data.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A)
                self.distortion_coeffs = calib_data.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A)
                print(f"[INFO] Loaded camera intrinsics from device")
            except Exception as e:
                print(f"[WARNING] Could not load camera intrinsics: {e}")
                self.camera_intrinsics = None
                self.distortion_coeffs = None
            
            self.running = True
            
            # Start processing thread
            self.process_thread = threading.Thread(target=self._process_frames, daemon=True)
            self.process_thread.start()
            
            return True
        except Exception as e:
            print(f"[ERROR] Failed to start camera: {e}")
            return False
    
    def stop(self):
        """Stop camera streams."""
        self.running = False
        if self.process_thread:
            self.process_thread.join(timeout=2.0)
        if self.device:
            self.device.close()
            self.device = None
        self.pipeline = None
        self.q_video = None
        self.q_nn = None
        self.q_depth = None
        self.q_imu = None
        
        # Clear queues
        while not self.frame_queue.empty():
            try:
                self.frame_queue.get_nowait()
            except:
                pass
        while not self.detection_queue.empty():
            try:
                self.detection_queue.get_nowait()
            except:
                pass
        while not self.depth_queue.empty():
            try:
                self.depth_queue.get_nowait()
            except:
                pass
        while not self.imu_queue.empty():
            try:
                self.imu_queue.get_nowait()
            except:
                pass
    
    def _process_frames(self):
        """Process frames and detections in a separate thread."""
        while self.running:
            try:
                # Get video frame
                in_video = self.q_video.get()
                if in_video is not None:
                    frame = in_video.getCvFrame()
                    
                    # Get detections
                    detections = []
                    in_nn = self.q_nn.tryGet()
                    if in_nn is not None:
                        detections = in_nn.detections
                    
                    # Get depth frame
                    depth_frame = None
                    in_depth = self.q_depth.tryGet()
                    if in_depth is not None:
                        depth_frame = in_depth.getFrame()
                    
                    # Get IMU data
                    imu_data = None
                    if self.q_imu is not None:
                        in_imu = self.q_imu.tryGet()
                        if in_imu is not None:
                            # IMU data comes as IMUData object with packets attribute
                            try:
                                imu_packets = in_imu.packets
                                if imu_packets and len(imu_packets) > 0:
                                    imu_data = {
                                        'accel': None,
                                        'gyro': None,
                                        'mag': None,
                                        'timestamp': None
                                    }
                                    # Iterate through all packets to find different sensor types
                                    # Each packet may contain different sensor data
                                    for imu_packet in imu_packets:
                                        # Check if packet has accelerometer data
                                        if hasattr(imu_packet, 'acceleroMeter'):
                                            imu_data['accel'] = imu_packet.acceleroMeter
                                            if hasattr(imu_packet, 'timestamp'):
                                                imu_data['timestamp'] = imu_packet.timestamp.get()
                                        # Check if packet has gyroscope data
                                        if hasattr(imu_packet, 'gyroscope'):
                                            imu_data['gyro'] = imu_packet.gyroscope
                                        # Check if packet has magnetometer data
                                        if hasattr(imu_packet, 'magneticField'):
                                            imu_data['mag'] = imu_packet.magneticField
                                    
                                    # If no data found, set to None
                                    if imu_data['accel'] is None and imu_data['gyro'] is None and imu_data['mag'] is None:
                                        imu_data = None
                            except (AttributeError, TypeError) as e:
                                # Skip IMU data if we can't parse it
                                imu_data = None
                    
                    # Update FPS
                    self.frame_count += 1
                    current_time = time.time()
                    if current_time - self.last_fps_time >= 1.0:
                        self.fps = self.frame_count / (current_time - self.last_fps_time)
                        self.frame_count = 0
                        self.last_fps_time = current_time
                    
                    # Put frame, detections, and depth in queues (non-blocking)
                    try:
                        self.frame_queue.put_nowait(frame)
                    except queue.Full:
                        try:
                            self.frame_queue.get_nowait()
                            self.frame_queue.put_nowait(frame)
                        except:
                            pass
                    
                    try:
                        self.detection_queue.put_nowait(detections)
                    except queue.Full:
                        try:
                            self.detection_queue.get_nowait()
                            self.detection_queue.put_nowait(detections)
                        except:
                            pass
                    
                    try:
                        self.depth_queue.put_nowait(depth_frame)
                    except queue.Full:
                        try:
                            self.depth_queue.get_nowait()
                            self.depth_queue.put_nowait(depth_frame)
                        except:
                            pass
                    
                    try:
                        self.imu_queue.put_nowait(imu_data)
                    except queue.Full:
                        try:
                            self.imu_queue.get_nowait()
                            self.imu_queue.put_nowait(imu_data)
                        except:
                            pass
                
                time.sleep(0.01)
            except Exception as e:
                if self.running:
                    print(f"[ERROR] Frame processing error: {e}")
                break
    
    def get_rgb_frame(self):
        """Get latest RGB frame."""
        try:
            return self.frame_queue.get_nowait()
        except queue.Empty:
            return None
    
    def get_detections(self):
        """Get latest YOLO detections."""
        try:
            return self.detection_queue.get_nowait()
        except queue.Empty:
            return []
    
    def get_depth_frame(self, raw=False):
        """Get latest depth frame.
        
        Args:
            raw: If True, return raw depth in mm. If False, return processed depth in meters.
        """
        try:
            depth_frame = self.depth_queue.get_nowait()
            if depth_frame is not None:
                if raw:
                    # Return raw depth in mm (uint16)
                    return depth_frame
                else:
                    # Filter depth if needed
                    min_depth = self.depth_config.get('min_depth', 0.1)
                    max_depth = self.depth_config.get('max_depth', 10.0)
                    depth_frame = depth_frame.astype(np.float32) / 1000.0  # Convert to meters
                    depth_frame[depth_frame < min_depth] = 0
                    depth_frame[depth_frame > max_depth] = 0
            return depth_frame
        except queue.Empty:
            return None
    
    def get_imu_data(self):
        """Get latest IMU data."""
        try:
            return self.imu_queue.get_nowait()
        except queue.Empty:
            return None
    
    def get_camera_intrinsics(self):
        """Get camera intrinsics matrix."""
        return self.camera_intrinsics
    
    def get_distortion_coeffs(self):
        """Get distortion coefficients."""
        return self.distortion_coeffs
    
    def get_all_data(self, raw_depth=False):
        """Get synchronized RGB, depth, detections, and IMU data.
        
        Args:
            raw_depth: If True, return raw depth in mm. If False, return processed depth in meters.
        """
        rgb_frame = self.get_rgb_frame()
        depth_frame = self.get_depth_frame(raw=raw_depth)
        detections = self.get_detections()
        imu_data = self.get_imu_data()
        
        return {
            'rgb': rgb_frame,
            'depth': depth_frame,
            'detections': detections,
            'imu': imu_data,
            'fps': self.fps
        }
    
    def draw_detections(self, frame, detections):
        """Draw YOLO detections on frame."""
        if frame is None or not detections:
            return frame
        
        # Scale coordinates from preview size to video size
        scale_x = self.video_size[0] / self.input_size[0]
        scale_y = self.video_size[1] / self.input_size[1]
        
        for detection in detections:
            # Get bounding box coordinates (detections are normalized 0-1)
            x1 = int(detection.xmin * self.input_size[0] * scale_x)
            y1 = int(detection.ymin * self.input_size[1] * scale_y)
            x2 = int(detection.xmax * self.input_size[0] * scale_x)
            y2 = int(detection.ymax * self.input_size[1] * scale_y)
            
            # Get label
            label_id = int(detection.label)
            label = self.labels[label_id] if label_id < len(self.labels) else f"Class {label_id}"
            confidence = detection.confidence
            
            # Draw bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw label
            label_text = f"{label}: {confidence:.2f}"
            (label_width, label_height), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (x1, y1 - label_height - 10), (x1 + label_width, y1), (0, 255, 0), -1)
            cv2.putText(frame, label_text, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        return frame
    
    def is_running(self):
        """Check if camera is running."""
        return self.running
    
    @property
    def yolo_detector(self):
        """Compatibility property for accessing detector-like attributes."""
        return self
