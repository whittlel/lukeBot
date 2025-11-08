"""
YOLO Detector using OAK-D camera with DepthAI v2.
Handles YOLO detection on-device.
"""

import os
import queue
import threading
import time
import depthai as dai
import cv2
import numpy as np
from pathlib import Path

# COCO class labels for YOLO models
COCO_LABELS = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
    "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
    "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]


class YOLODetector:
    """Handles DepthAI pipeline and YOLO detection using DepthAI v2."""
    
    def __init__(self, model_path, labels=None, input_size=(640, 640), confidence_threshold=0.5):
        """
        Initialize YOLO detector.
        
        Args:
            model_path: Path to YOLO model (.blob file or .json with blob path)
            labels: List of class labels (default: COCO_LABELS)
            input_size: Model input size (width, height)
            confidence_threshold: Detection confidence threshold
        """
        self.model_path = self._resolve_model_path(model_path)
        self.labels = labels or COCO_LABELS
        self.input_size = input_size
        self.confidence_threshold = confidence_threshold
        self.video_size = (1280, 720)
        
        self.pipeline = None
        self.device = None
        self.running = False
        self.frame_queue = queue.Queue(maxsize=2)
        self.detection_queue = queue.Queue(maxsize=2)
        self.fps = 0.0
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.process_thread = None
        
    def _resolve_model_path(self, model_path):
        """Resolve model path - handle .json files that contain blob path."""
        path = Path(model_path)
        
        # If path doesn't exist, try to find it
        if not path.exists():
            # Try to find .blob file with same name
            if path.suffix == '.json':
                blob_file = path.with_suffix('.blob')
                if blob_file.exists():
                    return str(blob_file)
            # If still not found, try looking in common locations
            project_root = Path(__file__).parent.parent.parent
            # Try data/models first
            alt_path = project_root / "data" / "models" / path.name
            if alt_path.exists():
                return str(alt_path)
            # Try in subdirectories
            for subdir in (project_root / "data" / "models").iterdir():
                if subdir.is_dir():
                    alt_path = subdir / path.name
                    if alt_path.exists():
                        return str(alt_path)
        
        # Handle .json files that contain blob path
        if path.suffix == '.json' and path.exists():
            import json
            try:
                with open(path, 'r') as f:
                    model_info = json.load(f)
                    blob_path = model_info.get('blob', None)
                    if blob_path:
                        # Try relative to JSON file
                        blob_file = path.parent / blob_path
                        if blob_file.exists():
                            return str(blob_file)
                        # Try absolute path
                        if Path(blob_path).exists():
                            return blob_path
            except Exception:
                pass
            # Fallback: look for .blob file with same name
            blob_file = path.with_suffix('.blob')
            if blob_file.exists():
                return str(blob_file)
        
        # Return the path as-is (should be a .blob file)
        return str(path)
    
    def create_pipeline(self):
        """Create DepthAI pipeline with YOLO model (DepthAI v2 style)."""
        os.environ['DEPTHAI_LEVEL'] = 'warn'
        
        # Verify model path exists
        model_path = Path(self.model_path)
        if not model_path.exists():
            raise FileNotFoundError(f"Model file not found: {self.model_path}")
        
        # Verify it's a .blob file
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
        
        # Create YOLO detection network (DepthAI v2 style)
        detection_nn = pipeline.create(dai.node.YoloDetectionNetwork)
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
        
        # Create video output (for display)
        xout_video = pipeline.create(dai.node.XLinkOut)
        xout_video.setStreamName("video")
        cam.video.link(xout_video.input)
        
        # Create detection output
        xout_nn = pipeline.create(dai.node.XLinkOut)
        xout_nn.setStreamName("nn")
        detection_nn.out.link(xout_nn.input)
        
        return pipeline
    
    def start(self):
        """Start the detection pipeline."""
        if self.running:
            return False
        
        try:
            self.pipeline = self.create_pipeline()
            self.device = dai.Device(self.pipeline)
            
            # Get output queues (DepthAI v2 style)
            self.q_video = self.device.getOutputQueue(name="video", maxSize=4, blocking=False)
            self.q_nn = self.device.getOutputQueue(name="nn", maxSize=4, blocking=False)
            
            self.running = True
            
            # Start processing thread
            self.process_thread = threading.Thread(target=self._process_frames, daemon=True)
            self.process_thread.start()
            
            return True
        except Exception as e:
            print(f"[ERROR] Failed to start YOLO detector: {e}")
            return False
    
    def stop(self):
        """Stop the detection pipeline."""
        self.running = False
        if self.process_thread:
            self.process_thread.join(timeout=2.0)
        if self.device:
            self.device.close()
            self.device = None
        self.pipeline = None
        
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
                    
                    # Update FPS
                    self.frame_count += 1
                    current_time = time.time()
                    if current_time - self.last_fps_time >= 1.0:
                        self.fps = self.frame_count / (current_time - self.last_fps_time)
                        self.frame_count = 0
                        self.last_fps_time = current_time
                    
                    # Put frame and detections in queues (non-blocking)
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
                
                time.sleep(0.01)
            except Exception as e:
                if self.running:
                    print(f"[ERROR] Frame processing error: {e}")
                break
    
    def get_frame(self):
        """Get latest frame (non-blocking)."""
        try:
            return self.frame_queue.get_nowait()
        except queue.Empty:
            return None
    
    def get_detections(self):
        """Get latest detections (non-blocking)."""
        try:
            return self.detection_queue.get_nowait()
        except queue.Empty:
            return []
    
    def get_fps(self):
        """Get current FPS."""
        return self.fps

