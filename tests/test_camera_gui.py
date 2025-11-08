#!/usr/bin/env python3
"""
Tkinter GUI for Testing OAK-D Camera on LukeBot
Displays live camera feed with YOLO detection, depth visualization, and controls.
"""

import os
import sys
import json
import cv2
import numpy as np
import tkinter as tk
from tkinter import ttk
from pathlib import Path
from PIL import Image, ImageTk
import threading
import time
import yaml

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from camera.oakd_camera import OakDCamera, COCO_LABELS


class CameraGUI:
    """Tkinter GUI application for OAK-D camera testing."""
    
    def __init__(self, root):
        self.root = root
        self.root.title("LukeBot - OAK-D Camera Tester")
        self.root.geometry("1400x800")
        
        self.camera = None
        self.running = False
        self.show_depth = False
        self.show_detections = True
        self.confidence_threshold = 0.5
        
        # Depth processing settings (adjustable in real-time)
        self.min_depth = 0.1  # meters
        self.max_depth = 10.0  # meters
        self.median_filter_size = 1  # kernel size (1=no filter, 3, 5, 7, etc.)
        self.bilateral_filter_on = False  # Bilateral filter toggle
        self.use_gaussian_blur = False  # Use Gaussian blur instead of median (faster)
        
        # Load configuration
        self.load_config()
        
        # Setup UI
        self.setup_ui()
        
        # Start update loop
        self.update_frame()
    
    def load_config(self):
        """Load camera configuration."""
        try:
            config_path = Path(__file__).parent.parent / "config" / "camera_config.yaml"
            if config_path.exists():
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                    self.yolo_config = config.get('yolo', {})
                    self.depth_config = config.get('depth', {})
                    self.confidence_threshold = self.yolo_config.get('confidence_threshold', 0.5)
            else:
                self.yolo_config = {}
                self.depth_config = {}
        except Exception as e:
            print(f"[WARNING] Could not load config: {e}")
            self.yolo_config = {}
            self.depth_config = {}
    
    def setup_ui(self):
        """Setup the GUI interface."""
        # Top frame for controls
        control_frame = ttk.Frame(self.root, padding="10")
        control_frame.pack(fill=tk.X)
        
        # Control buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(side=tk.LEFT, padx=5)
        
        self.start_button = ttk.Button(button_frame, text="Start Camera", command=self.start_camera)
        self.start_button.pack(side=tk.LEFT, padx=2)
        
        self.stop_button = ttk.Button(button_frame, text="Stop Camera", command=self.stop_camera, state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT, padx=2)
        
        # Display options
        options_frame = ttk.Frame(control_frame)
        options_frame.pack(side=tk.LEFT, padx=10)
        
        self.detections_var = tk.BooleanVar(value=True)
        self.detections_check = ttk.Checkbutton(
            options_frame,
            text="Show Detections",
            variable=self.detections_var,
            command=self.on_detections_toggled
        )
        self.detections_check.pack(side=tk.LEFT, padx=5)
        
        self.depth_var = tk.BooleanVar(value=False)
        self.depth_check = ttk.Checkbutton(
            options_frame,
            text="Show Depth (New Window)",
            variable=self.depth_var,
            command=self.on_depth_toggled
        )
        self.depth_check.pack(side=tk.LEFT, padx=5)
        
        # Confidence threshold
        threshold_frame = ttk.Frame(control_frame)
        threshold_frame.pack(side=tk.LEFT, padx=10)
        
        ttk.Label(threshold_frame, text="Confidence:").pack(side=tk.LEFT, padx=2)
        self.confidence_var = tk.DoubleVar(value=self.confidence_threshold)
        self.confidence_scale = ttk.Scale(
            threshold_frame,
            from_=0.0,
            to=1.0,
            variable=self.confidence_var,
            orient=tk.HORIZONTAL,
            length=100,
            command=self.on_confidence_changed
        )
        self.confidence_scale.pack(side=tk.LEFT, padx=2)
        
        self.confidence_label = ttk.Label(threshold_frame, text=f"{self.confidence_threshold:.2f}")
        self.confidence_label.pack(side=tk.LEFT, padx=2)
        
        # Depth settings frame (collapsible or always visible)
        depth_settings_frame = ttk.LabelFrame(control_frame, text="Depth Settings", padding="5")
        depth_settings_frame.pack(side=tk.LEFT, padx=10)
        
        # Min depth slider
        min_depth_row = ttk.Frame(depth_settings_frame)
        min_depth_row.pack(fill=tk.X, padx=2, pady=2)
        ttk.Label(min_depth_row, text="Min Depth (m):").pack(side=tk.LEFT, padx=2)
        self.min_depth_var = tk.DoubleVar(value=self.min_depth)
        self.min_depth_scale = ttk.Scale(
            min_depth_row,
            from_=0.05,
            to=2.0,
            variable=self.min_depth_var,
            orient=tk.HORIZONTAL,
            length=120,
            command=self.on_min_depth_changed
        )
        self.min_depth_scale.pack(side=tk.LEFT, padx=2)
        self.min_depth_label = ttk.Label(min_depth_row, text=f"{self.min_depth:.2f}")
        self.min_depth_label.pack(side=tk.LEFT, padx=2)
        
        # Max depth slider
        max_depth_row = ttk.Frame(depth_settings_frame)
        max_depth_row.pack(fill=tk.X, padx=2, pady=2)
        ttk.Label(max_depth_row, text="Max Depth (m):").pack(side=tk.LEFT, padx=2)
        self.max_depth_var = tk.DoubleVar(value=self.max_depth)
        self.max_depth_scale = ttk.Scale(
            max_depth_row,
            from_=1.0,
            to=20.0,
            variable=self.max_depth_var,
            orient=tk.HORIZONTAL,
            length=120,
            command=self.on_max_depth_changed
        )
        self.max_depth_scale.pack(side=tk.LEFT, padx=2)
        self.max_depth_label = ttk.Label(max_depth_row, text=f"{self.max_depth:.1f}")
        self.max_depth_label.pack(side=tk.LEFT, padx=2)
        
        # Median filter slider
        median_row = ttk.Frame(depth_settings_frame)
        median_row.pack(fill=tk.X, padx=2, pady=2)
        ttk.Label(median_row, text="Smoothing Filter:").pack(side=tk.LEFT, padx=2)
        self.median_filter_var = tk.IntVar(value=self.median_filter_size)
        self.median_filter_scale = ttk.Scale(
            median_row,
            from_=1,
            to=9,
            variable=self.median_filter_var,
            orient=tk.HORIZONTAL,
            length=120,
            command=self.on_median_filter_changed
        )
        self.median_filter_scale.pack(side=tk.LEFT, padx=2)
        self.median_filter_label = ttk.Label(median_row, text=f"{self.median_filter_size} (off)" if self.median_filter_size == 1 else f"{self.median_filter_size}")
        self.median_filter_label.pack(side=tk.LEFT, padx=2)
        
        # Status frame
        status_frame = ttk.Frame(self.root, padding="5")
        status_frame.pack(fill=tk.X)
        
        self.status_label = ttk.Label(status_frame, text="Status: Ready", foreground="green")
        self.status_label.pack(side=tk.LEFT, padx=5)
        
        self.fps_label = ttk.Label(status_frame, text="FPS: 0.0")
        self.fps_label.pack(side=tk.LEFT, padx=10)
        
        self.detection_label = ttk.Label(status_frame, text="Detections: 0")
        self.detection_label.pack(side=tk.LEFT, padx=10)
        
        # Main display area
        display_frame = ttk.Frame(self.root, padding="10")
        display_frame.pack(fill=tk.BOTH, expand=True)
        
        # RGB camera display (takes full window now)
        rgb_frame = ttk.LabelFrame(display_frame, text="RGB Camera + YOLO", padding="5")
        rgb_frame.pack(fill=tk.BOTH, expand=True, padx=5)
        
        self.rgb_label = tk.Label(rgb_frame, text="No video feed", bg="black", fg="white")
        self.rgb_label.pack(fill=tk.BOTH, expand=True)
        
        # Depth will be shown in separate window
        self.depth_window = None
        self.depth_label = None
    
    def on_detections_toggled(self):
        """Handle detections toggle."""
        self.show_detections = self.detections_var.get()
    
    def on_depth_toggled(self):
        """Handle depth toggle."""
        self.show_depth = self.depth_var.get()
        if self.show_depth:
            # Create separate window for depth map
            if self.depth_window is None:
                self.depth_window = tk.Toplevel(self.root)
                self.depth_window.title("Depth Map")
                self.depth_window.geometry("800x600")
                
                # Create label for depth display
                depth_frame = ttk.Frame(self.depth_window, padding="10")
                depth_frame.pack(fill=tk.BOTH, expand=True)
                
                self.depth_label = tk.Label(depth_frame, text="No depth feed", bg="black", fg="white")
                self.depth_label.pack(fill=tk.BOTH, expand=True)
                
                # Handle window close
                self.depth_window.protocol("WM_DELETE_WINDOW", self.on_depth_window_close)
            else:
                # Window already exists, just show it
                self.depth_window.deiconify()
        else:
            # Hide depth window
            if self.depth_window is not None:
                self.depth_window.withdraw()
    
    def on_depth_window_close(self):
        """Handle depth window close event."""
        # Uncheck the checkbox and hide window
        self.depth_var.set(False)
        self.show_depth = False
        if self.depth_window is not None:
            self.depth_window.withdraw()
    
    def on_confidence_changed(self, value=None):
        """Handle confidence threshold change."""
        self.confidence_threshold = self.confidence_var.get()
        self.confidence_label.config(text=f"{self.confidence_threshold:.2f}")
        # Update camera confidence threshold if running
        if self.camera:
            self.camera.confidence_threshold = self.confidence_threshold
    
    def on_min_depth_changed(self, value=None):
        """Handle min depth change."""
        self.min_depth = self.min_depth_var.get()
        self.min_depth_label.config(text=f"{self.min_depth:.2f}")
    
    def on_max_depth_changed(self, value=None):
        """Handle max depth change."""
        self.max_depth = self.max_depth_var.get()
        self.max_depth_label.config(text=f"{self.max_depth:.1f}")
    
    def on_median_filter_changed(self, value=None):
        """Handle median filter size change."""
        # Round to odd numbers (1, 3, 5, 7, 9) for median filter
        raw_value = int(self.median_filter_var.get())
        # Map to valid odd numbers
        valid_sizes = [1, 3, 5, 7, 9]
        closest = min(valid_sizes, key=lambda x: abs(x - raw_value))
        self.median_filter_size = closest
        self.median_filter_var.set(closest)
        
        # Update label with filter type info
        if self.median_filter_size == 1:
            label_text = "1 (off)"
        elif self.median_filter_size >= 7:
            label_text = f"{self.median_filter_size} (Gaussian)"
        else:
            label_text = f"{self.median_filter_size} (Median)"
        
        self.median_filter_label.config(text=label_text)
    
    def start_camera(self):
        """Start camera."""
        try:
            self.update_status("Starting camera...", "orange")
            
            # Initialize camera
            self.camera = OakDCamera()
            
            # Update confidence threshold
            if self.camera:
                self.camera.confidence_threshold = self.confidence_threshold
            
            # Start camera
            if self.camera.start():
                self.running = True
                self.start_button.config(state=tk.DISABLED)
                self.stop_button.config(state=tk.NORMAL)
                self.update_status("Camera running", "green")
            else:
                self.update_status("Failed to start camera", "red")
                self.camera = None
        except Exception as e:
            self.update_status(f"Error: {e}", "red")
            if self.camera:
                self.camera.stop()
                self.camera = None
    
    def stop_camera(self):
        """Stop camera."""
        if self.camera:
            self.camera.stop()
            self.camera = None
        
        self.running = False
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        self.update_status("Camera stopped", "orange")
        
        # Clear displays
        self.rgb_label.config(image='', text="No video feed")
        if self.depth_label is not None:
            self.depth_label.config(image='', text="No depth feed")
        
        # Close depth window if open
        if self.depth_window is not None:
            self.depth_window.destroy()
            self.depth_window = None
            self.depth_label = None
    
    def draw_detections(self, frame, detections):
        """Draw YOLO detections on frame with depth information."""
        if not detections or not self.show_detections:
            return frame, 0
        
        frame_height, frame_width = frame.shape[:2]
        
        # Get camera for coordinate scaling
        if not self.camera:
            return frame, 0
        
        # Scale coordinates from preview size to video size
        preview_size = self.camera.input_size
        video_size = self.camera.video_size
        scale_x = video_size[0] / preview_size[0]
        scale_y = video_size[1] / preview_size[1]
        
        detection_count = 0
        
        for detection in detections:
            # Filter by confidence
            if detection.confidence < self.confidence_threshold:
                continue
            
            detection_count += 1
            
            # Get bounding box coordinates (detections are normalized 0-1)
            x1 = int(detection.xmin * preview_size[0] * scale_x)
            y1 = int(detection.ymin * preview_size[1] * scale_y)
            x2 = int(detection.xmax * preview_size[0] * scale_x)
            y2 = int(detection.ymax * preview_size[1] * scale_y)
            
            # Clamp to frame bounds
            x1 = max(0, min(x1, frame_width - 1))
            y1 = max(0, min(y1, frame_height - 1))
            x2 = max(0, min(x2, frame_width - 1))
            y2 = max(0, min(y2, frame_height - 1))
            
            # Get label
            label_id = int(detection.label)
            label = self.camera.labels[label_id] if label_id < len(self.camera.labels) else f"Class {label_id}"
            confidence = detection.confidence
            
            # Get depth information if available (from spatial detection)
            depth_info = ""
            if hasattr(detection, 'spatialCoordinates') and detection.spatialCoordinates:
                coords = detection.spatialCoordinates
                # z is depth in mm, convert to meters
                z_mm = coords.z if hasattr(coords, 'z') else 0
                if z_mm > 0:
                    depth_m = z_mm / 1000.0
                    depth_info = f" | {depth_m:.2f}m"
            
            # Draw bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw label with confidence and depth
            label_text = f"{label}: {confidence:.2f}{depth_info}"
            (text_width, text_height), baseline = cv2.getTextSize(
                label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
            )
            
            # Label background
            cv2.rectangle(
                frame,
                (x1, y1 - text_height - baseline - 5),
                (x1 + text_width, y1),
                (0, 255, 0),
                -1
            )
            
            # Label text
            cv2.putText(
                frame,
                label_text,
                (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                1
            )
        
        return frame, detection_count
    
    def update_frame(self):
        """Update video frames in GUI."""
        if self.camera and self.running:
            try:
                # Get camera data (with raw depth for our filtering)
                data = self.camera.get_all_data(raw_depth=True)
                
                if data['rgb'] is not None:
                    # Draw detections
                    frame = data['rgb'].copy()
                    if self.show_detections:
                        frame, detection_count = self.draw_detections(frame, data['detections'])
                    else:
                        detection_count = len(data['detections'])
                    
                    # Update FPS
                    fps = data['fps']
                    self.fps_label.config(text=f"FPS: {fps:.1f}")
                    self.detection_label.config(text=f"Detections: {detection_count}")
                    
                    # Convert to PIL Image
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    frame_pil = Image.fromarray(frame_rgb)
                    
                    # Resize to fit display - get actual label size
                    display_width = self.rgb_label.winfo_width()
                    display_height = self.rgb_label.winfo_height()
                    
                    # Wait for label to be properly sized if needed
                    if display_width <= 1 or display_height <= 1:
                        self.root.update_idletasks()
                        display_width = self.rgb_label.winfo_width()
                        display_height = self.rgb_label.winfo_height()
                    
                    if display_width > 1 and display_height > 1:
                        img_width, img_height = frame_pil.size
                        scale = min(display_width / img_width, display_height / img_height)
                        new_width = int(img_width * scale)
                        new_height = int(img_height * scale)
                        
                        frame_pil = frame_pil.resize((new_width, new_height), Image.Resampling.LANCZOS)
                    
                    # Convert to PhotoImage
                    frame_tk = ImageTk.PhotoImage(image=frame_pil)
                    
                    # Update label
                    self.rgb_label.config(image=frame_tk, text="")
                    self.rgb_label.image = frame_tk  # Keep a reference
                
                # Update depth display if enabled and window exists
                if self.show_depth and data['depth'] is not None and self.depth_label is not None:
                    try:
                        # Depth frame is raw in mm (uint16) when raw_depth=True
                        raw_depth = data['depth']
                        
                        # Convert to meters
                        depth_meters = raw_depth.astype(np.float32) / 1000.0
                        
                        # Apply min/max depth filtering based on slider values
                        depth_meters[depth_meters < self.min_depth] = 0
                        depth_meters[depth_meters > self.max_depth] = 0
                        
                        # Apply filtering to reduce noise and doubling artifacts
                        if self.median_filter_size > 1:
                            # Convert to uint16 for filtering (preserve precision)
                            depth_uint16 = (depth_meters * 1000).astype(np.uint16)
                            
                            # For large filters (7, 9), use Gaussian blur instead (much faster)
                            if self.median_filter_size >= 7:
                                # Gaussian blur is faster and similar effect for large kernels
                                kernel_size = self.median_filter_size
                                # Ensure odd kernel size
                                if kernel_size % 2 == 0:
                                    kernel_size += 1
                                depth_filtered = cv2.GaussianBlur(
                                    depth_uint16.astype(np.float32),
                                    (kernel_size, kernel_size),
                                    0
                                ).astype(np.uint16)
                            else:
                                # Use median filter for smaller kernels (3, 5)
                                depth_filtered = cv2.medianBlur(depth_uint16, self.median_filter_size)
                            
                            # Convert back to meters
                            depth_meters = depth_filtered.astype(np.float32) / 1000.0
                            # Re-apply zeros where depth was invalid (0 in uint16 means invalid)
                            depth_meters[depth_filtered == 0] = 0
                        
                        # Convert depth to color visualization
                        # Normalize depth (already in meters)
                        depth_valid = depth_meters[depth_meters > 0]
                        if len(depth_valid) > 0:
                            min_val = depth_valid.min()
                            max_val = depth_valid.max()
                            if max_val > min_val:
                                depth_normalized = (depth_meters - min_val) / (max_val - min_val) * 255
                            else:
                                depth_normalized = np.zeros_like(depth_meters, dtype=np.uint8)
                        else:
                            depth_normalized = np.zeros_like(depth_meters, dtype=np.uint8)
                        
                        depth_normalized = depth_normalized.astype(np.uint8)
                        depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
                        
                        # Convert to PIL Image
                        depth_rgb = cv2.cvtColor(depth_colored, cv2.COLOR_BGR2RGB)
                        depth_pil = Image.fromarray(depth_rgb)
                        
                        # Resize to fit display - get actual label size
                        display_width = self.depth_label.winfo_width()
                        display_height = self.depth_label.winfo_height()
                        
                        # Wait for label to be properly sized if needed
                        if display_width <= 1 or display_height <= 1:
                            self.depth_window.update_idletasks()
                            display_width = self.depth_label.winfo_width()
                            display_height = self.depth_label.winfo_height()
                        
                        if display_width > 1 and display_height > 1:
                            img_width, img_height = depth_pil.size
                            scale = min(display_width / img_width, display_height / img_height)
                            new_width = int(img_width * scale)
                            new_height = int(img_height * scale)
                            
                            depth_pil = depth_pil.resize((new_width, new_height), Image.Resampling.LANCZOS)
                        
                        # Convert to PhotoImage
                        depth_tk = ImageTk.PhotoImage(image=depth_pil)
                        
                        # Update label
                        self.depth_label.config(image=depth_tk, text="")
                        self.depth_label.image = depth_tk  # Keep a reference
                    except Exception as e:
                        # Depth window might have been closed
                        if self.depth_window is not None:
                            pass
                
            except Exception as e:
                if self.running:
                    print(f"[ERROR] Frame update error: {e}")
        
        # Schedule next update
        # Adjust update rate based on median filter size to prevent freezing
        # Larger filters need more time, so we update less frequently
        if hasattr(self, 'median_filter_size') and self.median_filter_size >= 7:
            update_delay = 50  # ~20 FPS for large filters
        elif hasattr(self, 'median_filter_size') and self.median_filter_size >= 5:
            update_delay = 40  # ~25 FPS for medium filters
        else:
            update_delay = 33  # ~30 FPS for small/no filters
        
        self.root.after(update_delay, self.update_frame)
    
    def update_status(self, message, color="black"):
        """Update status label."""
        self.status_label.config(text=f"Status: {message}", foreground=color)
    
    def on_closing(self):
        """Handle window closing."""
        if self.camera:
            self.stop_camera()
        
        # Close depth window if open
        if self.depth_window is not None:
            self.depth_window.destroy()
        
        self.root.destroy()


def main():
    """Main function."""
    # Disable DepthAI logging
    os.environ['DEPTHAI_LEVEL'] = 'warn'
    
    root = tk.Tk()
    app = CameraGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == '__main__':
    main()

