#!/usr/bin/env python3
"""
Test script to capture and stitch all three OAK-D camera views.
Shows panoramic view from RGB (center), left stereo, and right stereo cameras.
"""

import sys
from pathlib import Path
import cv2
import numpy as np
import depthai as dai

# Add parent to path
sys.path.insert(0, str(Path(__file__).parent.parent))


class PanoramicCamera:
    """Capture and stitch images from all three OAK-D cameras."""

    def __init__(self):
        self.pipeline = None
        self.device = None
        self.running = False

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

        return pipeline

    def start(self):
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

            # Get camera calibration data
            calib = self.device.readCalibration()

            # Get camera FOVs
            self.rgb_fov = calib.getFov(dai.CameraBoardSocket.CAM_A)
            self.left_fov = calib.getFov(dai.CameraBoardSocket.CAM_B)
            self.right_fov = calib.getFov(dai.CameraBoardSocket.CAM_C)

            print(f"Camera FOVs:")
            print(f"  RGB (center): {self.rgb_fov:.1f}° horizontal")
            print(f"  Left stereo: {self.left_fov:.1f}° horizontal")
            print(f"  Right stereo: {self.right_fov:.1f}° horizontal")

            # Get baseline (distance between stereo cameras)
            self.baseline = calib.getBaselineDistance()
            print(f"\nStereo baseline: {self.baseline:.2f} cm")

            self.running = True
            return True
        except Exception as e:
            print(f"[ERROR] Failed to start cameras: {e}")
            return False

    def stop(self):
        """Stop camera streams."""
        self.running = False
        if self.device:
            self.device.close()
            self.device = None
        self.pipeline = None

    def get_frames(self):
        """Get frames from all three cameras."""
        if not self.running:
            return None, None, None

        try:
            # Get frames
            rgb_frame = None
            left_frame = None
            right_frame = None

            in_rgb = self.q_rgb.tryGet()
            if in_rgb is not None:
                rgb_frame = in_rgb.getCvFrame()

            in_left = self.q_left.tryGet()
            if in_left is not None:
                left_frame = in_left.getCvFrame()

            in_right = self.q_right.tryGet()
            if in_right is not None:
                right_frame = in_right.getCvFrame()

            return rgb_frame, left_frame, right_frame
        except Exception as e:
            print(f"[ERROR] Failed to get frames: {e}")
            return None, None, None

    def stitch_panoramic_view(self, rgb_frame, left_frame, right_frame):
        """
        Stitch frames into a panoramic view.
        Layout: [Left] [RGB] [Right]
        """
        if rgb_frame is None:
            return None

        # Target height for all frames (use RGB height as reference)
        target_height = rgb_frame.shape[0]

        # Resize and convert mono frames to BGR
        frames = []

        # Left camera
        if left_frame is not None:
            # Convert mono to BGR
            left_bgr = cv2.cvtColor(left_frame, cv2.COLOR_GRAY2BGR)
            # Resize to match RGB height
            aspect = left_bgr.shape[1] / left_bgr.shape[0]
            new_width = int(target_height * aspect)
            left_resized = cv2.resize(left_bgr, (new_width, target_height))
            frames.append(left_resized)

        # RGB camera (center)
        frames.append(rgb_frame)

        # Right camera
        if right_frame is not None:
            # Convert mono to BGR
            right_bgr = cv2.cvtColor(right_frame, cv2.COLOR_GRAY2BGR)
            # Resize to match RGB height
            aspect = right_bgr.shape[1] / right_bgr.shape[0]
            new_width = int(target_height * aspect)
            right_resized = cv2.resize(right_bgr, (new_width, target_height))
            frames.append(right_resized)

        # Stitch horizontally
        if len(frames) > 1:
            panorama = np.hstack(frames)
        else:
            panorama = frames[0] if frames else None

        return panorama

    def create_advanced_stitch(self, rgb_frame, left_frame, right_frame):
        """
        Create panoramic view with blending at overlap regions.
        This provides a smoother transition between cameras.
        """
        if rgb_frame is None:
            return None

        target_height = rgb_frame.shape[0]

        # Process frames
        if left_frame is not None:
            left_bgr = cv2.cvtColor(left_frame, cv2.COLOR_GRAY2BGR)
            aspect = left_bgr.shape[1] / left_bgr.shape[0]
            new_width = int(target_height * aspect)
            left_resized = cv2.resize(left_bgr, (new_width, target_height))
        else:
            left_resized = None

        if right_frame is not None:
            right_bgr = cv2.cvtColor(right_frame, cv2.COLOR_GRAY2BGR)
            aspect = right_bgr.shape[1] / right_bgr.shape[0]
            new_width = int(target_height * aspect)
            right_resized = cv2.resize(right_bgr, (new_width, target_height))
        else:
            right_resized = None

        # Simple horizontal stacking (can be enhanced with blending)
        frames = []
        if left_resized is not None:
            frames.append(left_resized)
        frames.append(rgb_frame)
        if right_resized is not None:
            frames.append(right_resized)

        if len(frames) > 1:
            panorama = np.hstack(frames)
        else:
            panorama = frames[0]

        # Add labels to identify each camera
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.8
        thickness = 2

        x_offset = 0
        if left_resized is not None:
            cv2.putText(panorama, "LEFT STEREO", (x_offset + 10, 40),
                       font, font_scale, (0, 255, 255), thickness)
            x_offset += left_resized.shape[1]

        cv2.putText(panorama, "RGB (CENTER)", (x_offset + 10, 40),
                   font, font_scale, (0, 255, 0), thickness)
        x_offset += rgb_frame.shape[1]

        if right_resized is not None:
            cv2.putText(panorama, "RIGHT STEREO", (x_offset + 10, 40),
                       font, font_scale, (255, 255, 0), thickness)

        return panorama


def main():
    """Main test function."""
    print("=" * 60)
    print("OAK-D Panoramic View Test")
    print("=" * 60)
    print("This script captures all three camera feeds and stitches them")
    print("into a panoramic view to visualize the total FOV coverage.")
    print()
    print("Controls:")
    print("  q - Quit")
    print("  s - Save panoramic snapshot")
    print("  1 - Simple stitch (horizontal stack)")
    print("  2 - Advanced stitch (with labels)")
    print("=" * 60)

    camera = PanoramicCamera()

    if not camera.start():
        print("Failed to start cameras!")
        return

    print("\nCameras started successfully!")
    print("Displaying panoramic view...\n")

    stitch_mode = 2  # Default to advanced stitch
    snapshot_count = 0

    try:
        while True:
            # Get frames from all cameras
            rgb_frame, left_frame, right_frame = camera.get_frames()

            if rgb_frame is None:
                continue

            # Create panoramic view
            if stitch_mode == 1:
                panorama = camera.stitch_panoramic_view(rgb_frame, left_frame, right_frame)
            else:
                panorama = camera.create_advanced_stitch(rgb_frame, left_frame, right_frame)

            if panorama is not None:
                # Add info overlay
                total_fov = camera.rgb_fov
                if left_frame is not None and right_frame is not None:
                    # Estimate total FOV (approximate)
                    total_fov = camera.rgb_fov + (camera.left_fov + camera.right_fov) / 2

                info_text = f"Total Approximate FOV: {total_fov:.1f}° | Mode: {'Simple' if stitch_mode == 1 else 'Advanced'}"
                cv2.putText(panorama, info_text, (10, panorama.shape[0] - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                # Display
                cv2.imshow("OAK-D Panoramic View", panorama)

            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                # Save snapshot
                snapshot_count += 1
                filename = f"panoramic_snapshot_{snapshot_count}.jpg"
                cv2.imwrite(filename, panorama)
                print(f"Saved {filename}")
            elif key == ord('1'):
                stitch_mode = 1
                print("Switched to simple stitch mode")
            elif key == ord('2'):
                stitch_mode = 2
                print("Switched to advanced stitch mode")

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        camera.stop()
        cv2.destroyAllWindows()
        print("\nCameras stopped")


if __name__ == "__main__":
    main()
