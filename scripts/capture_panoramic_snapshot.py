#!/usr/bin/env python3
"""
Capture and save panoramic snapshots from all three OAK-D cameras.
Saves images to disk for analysis without requiring a display.
"""

import sys
from pathlib import Path
import cv2
import numpy as np
import depthai as dai
import time

# Add parent to path
sys.path.insert(0, str(Path(__file__).parent.parent))


class PanoramicCapture:
    """Capture panoramic views from all three OAK-D cameras."""

    def __init__(self):
        self.pipeline = None
        self.device = None

    def create_pipeline(self):
        """Create DepthAI pipeline with all three cameras."""
        pipeline = dai.Pipeline()

        # RGB Camera (center)
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setVideoSize(1280, 720)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(30)

        # Left stereo camera
        cam_left = pipeline.create(dai.node.MonoCamera)
        cam_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        cam_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        cam_left.setFps(30)

        # Right stereo camera
        cam_right = pipeline.create(dai.node.MonoCamera)
        cam_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        cam_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        cam_right.setFps(30)

        # Stereo depth
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(False)
        stereo.setExtendedDisparity(True)
        cam_left.out.link(stereo.left)
        cam_right.out.link(stereo.right)

        # Create outputs
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.video.link(xout_rgb.input)

        xout_left = pipeline.create(dai.node.XLinkOut)
        xout_left.setStreamName("left")
        cam_left.out.link(xout_left.input)

        xout_right = pipeline.create(dai.node.XLinkOut)
        xout_right.setStreamName("right")
        cam_right.out.link(xout_right.input)

        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        return pipeline

    def capture_panoramic_snapshot(self, output_dir="data/panoramic"):
        """Capture and save panoramic snapshot."""
        print("Initializing cameras...")

        self.pipeline = self.create_pipeline()
        self.device = dai.Device(self.pipeline)

        # Get output queues
        q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        q_left = self.device.getOutputQueue(name="left", maxSize=4, blocking=False)
        q_right = self.device.getOutputQueue(name="right", maxSize=4, blocking=False)
        q_depth = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        # Get calibration data
        calib = self.device.readCalibration()
        rgb_fov = calib.getFov(dai.CameraBoardSocket.CAM_A)
        left_fov = calib.getFov(dai.CameraBoardSocket.CAM_B)
        right_fov = calib.getFov(dai.CameraBoardSocket.CAM_C)
        baseline = calib.getBaselineDistance()

        print(f"\nCamera FOV Information:")
        print(f"  RGB (center): {rgb_fov:.1f}° horizontal")
        print(f"  Left stereo: {left_fov:.1f}° horizontal")
        print(f"  Right stereo: {right_fov:.1f}° horizontal")
        print(f"  Baseline: {baseline:.2f} cm")

        # Estimate total FOV
        total_fov = rgb_fov + (left_fov + right_fov) * 0.25  # Approximate
        print(f"  Approximate total FOV: {total_fov:.1f}°")

        print("\nWaiting for camera warmup...")
        time.sleep(2)  # Let cameras warm up

        print("Capturing frames...")

        # Capture frames
        rgb_frame = None
        left_frame = None
        right_frame = None
        depth_frame = None

        # Get latest frames
        attempts = 0
        while (rgb_frame is None or left_frame is None or right_frame is None) and attempts < 50:
            if rgb_frame is None:
                in_rgb = q_rgb.tryGet()
                if in_rgb is not None:
                    rgb_frame = in_rgb.getCvFrame()

            if left_frame is None:
                in_left = q_left.tryGet()
                if in_left is not None:
                    left_frame = in_left.getCvFrame()

            if right_frame is None:
                in_right = q_right.tryGet()
                if in_right is not None:
                    right_frame = in_right.getCvFrame()

            if depth_frame is None:
                in_depth = q_depth.tryGet()
                if in_depth is not None:
                    depth_frame = in_depth.getFrame()

            attempts += 1
            time.sleep(0.1)

        if rgb_frame is None:
            print("Failed to capture RGB frame")
            return False

        print("Frames captured successfully!")

        # Create output directory
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        # Save individual frames
        timestamp = time.strftime("%Y%m%d_%H%M%S")

        print(f"\nSaving individual camera feeds...")
        cv2.imwrite(str(output_path / f"rgb_{timestamp}.jpg"), rgb_frame)
        print(f"  Saved: rgb_{timestamp}.jpg")

        if left_frame is not None:
            cv2.imwrite(str(output_path / f"left_{timestamp}.jpg"), left_frame)
            print(f"  Saved: left_{timestamp}.jpg")

        if right_frame is not None:
            cv2.imwrite(str(output_path / f"right_{timestamp}.jpg"), right_frame)
            print(f"  Saved: right_{timestamp}.jpg")

        # Create panoramic view
        print(f"\nCreating panoramic view...")
        target_height = rgb_frame.shape[0]
        frames = []

        # Left camera
        if left_frame is not None:
            left_bgr = cv2.cvtColor(left_frame, cv2.COLOR_GRAY2BGR)
            aspect = left_bgr.shape[1] / left_bgr.shape[0]
            new_width = int(target_height * aspect)
            left_resized = cv2.resize(left_bgr, (new_width, target_height))
            frames.append(left_resized)

        # RGB center
        frames.append(rgb_frame)

        # Right camera
        if right_frame is not None:
            right_bgr = cv2.cvtColor(right_frame, cv2.COLOR_GRAY2BGR)
            aspect = right_bgr.shape[1] / right_bgr.shape[0]
            new_width = int(target_height * aspect)
            right_resized = cv2.resize(right_bgr, (new_width, target_height))
            frames.append(right_resized)

        # Stitch horizontally
        panorama = np.hstack(frames)

        # Add labels
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.8
        thickness = 2

        x_offset = 0
        if left_frame is not None:
            cv2.putText(panorama, "LEFT STEREO", (x_offset + 10, 40),
                       font, font_scale, (0, 255, 255), thickness)
            x_offset += left_resized.shape[1]

        cv2.putText(panorama, "RGB (CENTER)", (x_offset + 10, 40),
                   font, font_scale, (0, 255, 0), thickness)
        x_offset += rgb_frame.shape[1]

        if right_frame is not None:
            cv2.putText(panorama, "RIGHT STEREO", (x_offset + 10, 40),
                       font, font_scale, (255, 255, 0), thickness)

        # Add FOV info
        info_text = f"Total FOV: ~{total_fov:.1f}deg | Baseline: {baseline:.1f}cm"
        cv2.putText(panorama, info_text, (10, panorama.shape[0] - 20),
                   font, 0.7, (255, 255, 255), 2)

        # Save panoramic view
        panorama_filename = str(output_path / f"panoramic_{timestamp}.jpg")
        cv2.imwrite(panorama_filename, panorama)
        print(f"  Saved: panoramic_{timestamp}.jpg")

        # Save depth visualization if available
        if depth_frame is not None:
            # Normalize depth for visualization
            depth_normalized = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX)
            depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)

            depth_filename = str(output_path / f"depth_{timestamp}.jpg")
            cv2.imwrite(depth_filename, depth_colored)
            print(f"  Saved: depth_{timestamp}.jpg")

        # Print summary
        print(f"\n{'='*60}")
        print(f"Panoramic Snapshot Summary")
        print(f"{'='*60}")
        print(f"Panoramic size: {panorama.shape[1]} x {panorama.shape[0]} pixels")
        print(f"Total horizontal FOV: ~{total_fov:.1f}°")
        print(f"Output directory: {output_path.absolute()}")
        print(f"{'='*60}")

        # Analyze obstacle detection coverage
        if depth_frame is not None:
            h, w = depth_frame.shape

            # Analyze center region
            center_region = depth_frame[int(h*0.35):int(h*0.65), int(w*0.35):int(w*0.65)]
            depth_m = center_region.astype(np.float32) / 1000.0
            valid_depths = depth_m[(depth_m > 0.05) & (depth_m < 10.0)]

            if valid_depths.size > 0:
                min_depth = np.percentile(valid_depths, 10)
                median_depth = np.median(valid_depths)
                print(f"\nDepth Analysis (center region):")
                print(f"  Minimum depth (p10): {min_depth:.2f}m")
                print(f"  Median depth: {median_depth:.2f}m")
                print(f"  Valid pixels: {valid_depths.size}/{center_region.size}")

        # Close device
        self.device.close()

        return True


def main():
    """Main function."""
    print("=" * 60)
    print("OAK-D Panoramic Snapshot Capture")
    print("=" * 60)
    print()

    capture = PanoramicCapture()

    try:
        success = capture.capture_panoramic_snapshot()

        if success:
            print("\nCapture completed successfully!")
            print("Check the data/panoramic/ directory for output images.")
        else:
            print("\nCapture failed!")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
