"""
SLAM Visualizer - Debug visualization for SLAM troubleshooting.
Provides visual aids for feature tracking, depth overlay, and comparison views.
"""

import cv2
import numpy as np
from typing import Optional, List, Tuple
from ..utils.data_structures import RobotPose


class SLAMVisualizer:
    """Visualizer for SLAM debugging and troubleshooting."""

    def __init__(self, config=None):
        """
        Initialize SLAM visualizer.

        Args:
            config: Configuration dictionary
        """
        self.config = config or {}
        viz_config = self.config.get('visualization', {})

        # Visualization settings
        self.show_feature_tracks = viz_config.get('show_feature_tracks', True)
        self.show_depth_overlay = viz_config.get('show_depth_overlay', True)
        self.feature_color = (0, 255, 0)  # Green for features
        self.match_color = (255, 255, 0)  # Yellow for matches
        self.track_color = (0, 255, 255)  # Cyan for tracks

        # Track history
        self.feature_tracks = []
        self.max_track_length = viz_config.get('max_track_length', 30)

    def visualize_features(self, image: np.ndarray,
                          keypoints: List[cv2.KeyPoint],
                          title: str = "Features",
                          max_features_to_show: int = 100) -> np.ndarray:
        """
        Visualize detected features on image.

        Args:
            image: Input image
            keypoints: Detected keypoints
            title: Window title
            max_features_to_show: Maximum number of features to visualize

        Returns:
            Image with features drawn
        """
        if image is None:
            return None

        # Create output image
        if len(image.shape) == 2:
            output = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        else:
            output = image.copy()

        # Draw keypoints
        if keypoints is not None and len(keypoints) > 0:
            # Sort keypoints by response (quality) and take top N
            sorted_kps = sorted(keypoints, key=lambda kp: kp.response, reverse=True)
            kps_to_draw = sorted_kps[:max_features_to_show]

            h, w = output.shape[:2]
            for kp in kps_to_draw:
                x, y = int(kp.pt[0]), int(kp.pt[1])

                # Skip if keypoint is out of bounds
                if x < 0 or x >= w or y < 0 or y >= h:
                    continue

                # Color based on response strength
                response = min(kp.response, 1.0) if kp.response > 0 else 0.5
                color_intensity = int(255 * response)
                color = (0, color_intensity, 255 - color_intensity)  # Green for high quality, red for low

                # Draw keypoint as small circle (reduced size)
                cv2.circle(output, (x, y), 3, color, 2)
                cv2.circle(output, (x, y), 1, (255, 255, 255), -1)  # White center

            # Add feature count
            total_features = len(keypoints)
            shown_features = len(kps_to_draw)
            cv2.putText(output, f"Features: {shown_features}/{total_features} (top quality)", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        else:
            cv2.putText(output, "Features: 0", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        return output

    def visualize_matches(self, img1: np.ndarray, img2: np.ndarray,
                         kp1: List[cv2.KeyPoint], kp2: List[cv2.KeyPoint],
                         matches: List[cv2.DMatch]) -> np.ndarray:
        """
        Visualize feature matches between two frames.

        Args:
            img1: First image
            img2: Second image
            kp1: Keypoints from first image
            kp2: Keypoints from second image
            matches: Matches between keypoints

        Returns:
            Side-by-side image with matches drawn
        """
        if img1 is None or img2 is None:
            return None

        # Convert to BGR if grayscale
        if len(img1.shape) == 2:
            img1 = cv2.cvtColor(img1, cv2.COLOR_GRAY2BGR)
        if len(img2.shape) == 2:
            img2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)

        # Draw matches
        match_img = cv2.drawMatches(
            img1, kp1, img2, kp2, matches,
            None,
            matchColor=self.match_color,
            singlePointColor=self.feature_color,
            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
        )

        # Add match count
        if matches is not None:
            cv2.putText(match_img, f"Matches: {len(matches)}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        return match_img

    def visualize_feature_tracks(self, image: np.ndarray,
                                keypoints: List[cv2.KeyPoint],
                                prev_keypoints: Optional[List[cv2.KeyPoint]] = None,
                                matches: Optional[List[cv2.DMatch]] = None,
                                max_tracks_to_show: int = 50) -> np.ndarray:
        """
        Visualize feature tracking over time.

        Args:
            image: Current image
            keypoints: Current keypoints
            prev_keypoints: Previous keypoints
            matches: Matches between prev and current keypoints
            max_tracks_to_show: Maximum number of tracks to display

        Returns:
            Image with feature tracks drawn
        """
        if image is None:
            return None

        # Convert to BGR if grayscale
        if len(image.shape) == 2:
            output = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        else:
            output = image.copy()

        # Draw feature tracks if we have matches
        if prev_keypoints is not None and matches is not None and len(matches) > 0:
            # Sort matches by distance (lower is better) and take top N
            sorted_matches = sorted(matches, key=lambda m: m.distance)
            matches_to_draw = sorted_matches[:max_tracks_to_show]

            h, w = output.shape[:2]
            for match in matches_to_draw:
                # Get matched points
                prev_pt = prev_keypoints[match.queryIdx].pt
                curr_pt = keypoints[match.trainIdx].pt

                prev_x, prev_y = int(prev_pt[0]), int(prev_pt[1])
                curr_x, curr_y = int(curr_pt[0]), int(curr_pt[1])

                # Skip if any point is out of bounds
                if (prev_x < 0 or prev_x >= w or prev_y < 0 or prev_y >= h or
                    curr_x < 0 or curr_x >= w or curr_y < 0 or curr_y >= h):
                    continue

                # Calculate motion magnitude
                motion = np.sqrt((curr_x - prev_x)**2 + (curr_y - prev_y)**2)

                # Color based on motion (blue=static, green=moving)
                if motion < 2:
                    color = (255, 0, 0)  # Blue for static
                elif motion < 10:
                    color = (0, 255, 255)  # Cyan for small motion
                else:
                    color = (0, 255, 0)  # Green for large motion

                # Draw track line
                cv2.line(output, (prev_x, prev_y), (curr_x, curr_y), color, 2)

                # Draw previous point
                cv2.circle(output, (prev_x, prev_y), 2, (255, 255, 0), -1)

                # Draw current point
                cv2.circle(output, (curr_x, curr_y), 4, (0, 255, 0), -1)

            total_matches = len(matches)
            shown_matches = len(matches_to_draw)

            # Count motion types for debug info
            static_count = sum(1 for match in matches_to_draw
                             if np.sqrt((int(keypoints[match.trainIdx].pt[0]) - int(prev_keypoints[match.queryIdx].pt[0]))**2 +
                                       (int(keypoints[match.trainIdx].pt[1]) - int(prev_keypoints[match.queryIdx].pt[1]))**2) < 2)

            cv2.putText(output, f"Tracks: {shown_matches}/{total_matches} ({static_count} static)", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        else:
            # Just draw current keypoints
            if keypoints is not None:
                # Show only top quality features
                sorted_kps = sorted(keypoints, key=lambda kp: kp.response, reverse=True)
                kps_to_draw = sorted_kps[:max_tracks_to_show]

                h, w = output.shape[:2]
                for kp in kps_to_draw:
                    x, y = int(kp.pt[0]), int(kp.pt[1])
                    # Bounds check
                    if x < 0 or x >= w or y < 0 or y >= h:
                        continue
                    cv2.circle(output, (x, y), 4, (0, 255, 0), -1)

                cv2.putText(output, f"Features: {len(kps_to_draw)}/{len(keypoints)} (top quality)", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        return output

    def visualize_depth_overlay(self, rgb_image: np.ndarray,
                               depth_image: np.ndarray,
                               alpha: float = 0.5,
                               colormap: int = cv2.COLORMAP_JET,
                               max_range: float = 1.5) -> np.ndarray:
        """
        Overlay depth information on RGB image.

        Args:
            rgb_image: RGB image
            depth_image: Depth map (in meters)
            alpha: Blending factor (0-1)
            colormap: OpenCV colormap to use for depth
            max_range: Maximum depth range to visualize (meters)

        Returns:
            RGB image with depth overlay
        """
        if rgb_image is None or depth_image is None:
            return rgb_image

        # Resize depth to match RGB dimensions first if needed
        if depth_image.shape[:2] != rgb_image.shape[:2]:
            depth_normalized = cv2.resize(depth_image,
                                         (rgb_image.shape[1], rgb_image.shape[0]),
                                         interpolation=cv2.INTER_NEAREST)
        else:
            depth_normalized = depth_image.copy()

        # Filter valid depths (focus on near-field: 0.1m to max_range)
        valid_mask = (depth_normalized > 0.1) & (depth_normalized < max_range)

        if not valid_mask.any():
            # No valid depth data in this range
            return rgb_image

        # Normalize to 0-255 range for visualization
        # Use fixed range 0.1m to max_range for consistent color mapping
        depth_viz = np.zeros_like(depth_normalized, dtype=np.uint8)

        # Clip depths to max_range and normalize
        depth_clipped = np.clip(depth_normalized, 0.1, max_range)
        depth_viz = ((depth_clipped - 0.1) / (max_range - 0.1) * 255).astype(np.uint8)

        # Set invalid depths to black
        depth_viz[~valid_mask] = 0

        # Apply colormap (JET: blue=close, red=far)
        depth_colored = cv2.applyColorMap(depth_viz, colormap)

        # Set invalid areas to black
        depth_colored[~valid_mask] = [0, 0, 0]

        # Blend images (only where valid depth exists)
        overlay = rgb_image.copy()
        overlay[valid_mask] = cv2.addWeighted(
            rgb_image[valid_mask], 1 - alpha,
            depth_colored[valid_mask], alpha, 0
        )

        # Add depth statistics
        if valid_mask.any():
            valid_depths = depth_normalized[valid_mask & (depth_normalized < max_range)]
            if len(valid_depths) > 0:
                min_depth = valid_depths.min()
                max_depth = valid_depths.max()
                mean_depth = valid_depths.mean()
                median_depth = np.median(valid_depths)

                cv2.putText(overlay, f"Depth (0.1-{max_range}m): min={min_depth:.2f}m median={median_depth:.2f}m mean={mean_depth:.2f}m",
                           (10, overlay.shape[0] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                # Add color scale legend
                cv2.putText(overlay, "Blue=Close | Red=Far", (10, overlay.shape[0] - 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        return overlay

    def visualize_point_cloud_projection(self, rgb_image: np.ndarray,
                                        depth_image: np.ndarray,
                                        camera_matrix: np.ndarray,
                                        sample_rate: int = 10,
                                        max_range: float = 1.5) -> np.ndarray:
        """
        Visualize 3D point cloud projected onto 2D image.

        Args:
            rgb_image: RGB image
            depth_image: Depth map (in meters)
            camera_matrix: Camera intrinsics
            sample_rate: Sample every Nth pixel
            max_range: Maximum depth range to visualize (meters)

        Returns:
            Image with point cloud visualization
        """
        if rgb_image is None or depth_image is None or camera_matrix is None:
            return rgb_image

        output = rgb_image.copy()

        # Ensure camera_matrix is numpy array
        if not isinstance(camera_matrix, np.ndarray):
            camera_matrix = np.array(camera_matrix)

        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]

        # Resize depth to match RGB if needed
        if depth_image.shape[:2] != rgb_image.shape[:2]:
            depth_resized = cv2.resize(depth_image,
                                      (rgb_image.shape[1], rgb_image.shape[0]),
                                      interpolation=cv2.INTER_NEAREST)
        else:
            depth_resized = depth_image

        h, w = depth_resized.shape
        valid_point_count = 0

        # Sample points
        for y in range(0, h, sample_rate):
            for x in range(0, w, sample_rate):
                # Bounds check
                if x >= w or y >= h:
                    continue

                depth = depth_resized[y, x]

                # Valid depth check (focus on near-field)
                if depth > 0.1 and depth < max_range:
                    # Compute 3D point
                    z = depth
                    px = (x - cx) * z / fx
                    py = (y - cy) * z / fy

                    # Color based on depth (blue=close, red=far)
                    # Normalize depth to 0-1 range
                    depth_norm = (depth - 0.1) / (max_range - 0.1)
                    depth_norm = np.clip(depth_norm, 0.0, 1.0)

                    # Create color gradient: blue (close) -> cyan -> green -> yellow -> red (far)
                    if depth_norm < 0.25:
                        # Blue to cyan
                        ratio = depth_norm / 0.25
                        color = (255, int(255 * ratio), 0)
                    elif depth_norm < 0.5:
                        # Cyan to green
                        ratio = (depth_norm - 0.25) / 0.25
                        color = (int(255 * (1 - ratio)), 255, 0)
                    elif depth_norm < 0.75:
                        # Green to yellow
                        ratio = (depth_norm - 0.5) / 0.25
                        color = (0, 255, int(255 * ratio))
                    else:
                        # Yellow to red
                        ratio = (depth_norm - 0.75) / 0.25
                        color = (0, int(255 * (1 - ratio)), 255)

                    # Draw point
                    cv2.circle(output, (x, y), 3, color, -1)
                    valid_point_count += 1

        cv2.putText(output, f"Point Cloud: {valid_point_count} points (0.1-{max_range}m)", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.putText(output, "Blue=Close | Red=Far", (10, 55),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        return output

    def create_slam_annotated_view(self, rgb_image: np.ndarray,
                                  depth_image: Optional[np.ndarray],
                                  keypoints: Optional[List[cv2.KeyPoint]],
                                  prev_keypoints: Optional[List[cv2.KeyPoint]],
                                  matches: Optional[List[cv2.DMatch]],
                                  pose: Optional[RobotPose],
                                  camera_matrix: Optional[np.ndarray],
                                  max_depth_range: float = 1.5) -> np.ndarray:
        """
        Create comprehensive SLAM annotated view showing all debug info.

        Args:
            rgb_image: RGB image
            depth_image: Depth map
            keypoints: Current keypoints
            prev_keypoints: Previous keypoints
            matches: Feature matches
            pose: Current robot pose
            camera_matrix: Camera intrinsics
            max_depth_range: Maximum depth range for visualization (meters)

        Returns:
            Annotated SLAM visualization
        """
        if rgb_image is None:
            return None

        output = rgb_image.copy()

        # 1. Draw feature tracks (show top 100 quality features)
        if keypoints is not None and prev_keypoints is not None and matches is not None:
            output = self.visualize_feature_tracks(output, keypoints, prev_keypoints, matches, max_tracks_to_show=100)
        elif keypoints is not None:
            output = self.visualize_features(output, keypoints, max_features_to_show=100)

        # 2. Overlay depth information (semi-transparent on right half)
        if depth_image is not None:
            h, w = output.shape[:2]
            # Create depth overlay for full image first
            full_depth_overlay = self.visualize_depth_overlay(
                output,
                depth_image,
                alpha=0.3,
                max_range=max_depth_range
            )
            if full_depth_overlay is not None:
                # Only apply depth overlay to right half
                output[:, w//2:] = full_depth_overlay[:, w//2:]

        # 3. Draw pose information
        if pose is not None:
            pose_text = f"Pose: ({pose.x:.2f}, {pose.y:.2f}) theta={np.degrees(pose.theta):.1f}deg"
            cv2.putText(output, pose_text, (10, output.shape[0] - 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # 4. Add grid overlay for depth perception
        self._draw_grid_overlay(output)

        # 5. Add title
        cv2.putText(output, "SLAM View (Features + Depth)", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        return output

    def _draw_grid_overlay(self, image: np.ndarray, grid_size: int = 50):
        """Draw a grid overlay on the image for reference."""
        h, w = image.shape[:2]
        color = (100, 100, 100)

        # Vertical lines
        for x in range(0, w, grid_size):
            cv2.line(image, (x, 0), (x, h), color, 1)

        # Horizontal lines
        for y in range(0, h, grid_size):
            cv2.line(image, (0, y), (w, y), color, 1)

    def create_side_by_side_view(self, panoramic_view: np.ndarray,
                                slam_view: np.ndarray,
                                title_left: str = "Panoramic View (Reality)",
                                title_right: str = "SLAM View (What SLAM Sees)") -> np.ndarray:
        """
        Create side-by-side comparison of panoramic view vs SLAM annotated view.

        Args:
            panoramic_view: Panoramic camera view
            slam_view: SLAM annotated view
            title_left: Title for left panel
            title_right: Title for right panel

        Returns:
            Side-by-side comparison image
        """
        if panoramic_view is None and slam_view is None:
            return None

        if panoramic_view is None:
            return slam_view
        if slam_view is None:
            return panoramic_view

        # Resize images to same height
        target_height = max(panoramic_view.shape[0], slam_view.shape[0])

        # Resize panoramic view
        pano_aspect = panoramic_view.shape[1] / panoramic_view.shape[0]
        pano_width = int(target_height * pano_aspect)
        pano_resized = cv2.resize(panoramic_view, (pano_width, target_height))

        # Resize SLAM view
        slam_aspect = slam_view.shape[1] / slam_view.shape[0]
        slam_width = int(target_height * slam_aspect)
        slam_resized = cv2.resize(slam_view, (slam_width, target_height))

        # Create side-by-side view
        side_by_side = np.hstack([pano_resized, slam_resized])

        # Add vertical separator
        separator_x = pano_width
        cv2.line(side_by_side, (separator_x, 0), (separator_x, target_height),
                (255, 255, 255), 3)

        # Add titles
        cv2.putText(side_by_side, title_left, (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        cv2.putText(side_by_side, title_right, (separator_x + 10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        return side_by_side

    def create_multi_panel_debug_view(self, rgb_image: np.ndarray,
                                     depth_image: Optional[np.ndarray],
                                     keypoints: Optional[List[cv2.KeyPoint]],
                                     prev_keypoints: Optional[List[cv2.KeyPoint]],
                                     matches: Optional[List[cv2.DMatch]],
                                     pose: Optional[RobotPose],
                                     camera_matrix: Optional[np.ndarray],
                                     max_depth_range: float = 1.5,
                                     odometry_diagnostics: Optional[dict] = None) -> np.ndarray:
        """
        Create comprehensive multi-panel debug view.

        Layout:
        [RGB with Features] [Depth Overlay]
        [Feature Tracks]    [Point Cloud]

        Args:
            rgb_image: RGB image
            depth_image: Depth map
            keypoints: Current keypoints
            prev_keypoints: Previous keypoints
            matches: Feature matches
            pose: Current robot pose
            camera_matrix: Camera intrinsics
            max_depth_range: Maximum depth range for visualization (meters)

        Returns:
            Multi-panel debug visualization
        """
        if rgb_image is None:
            return None

        h, w = rgb_image.shape[:2]
        panel_h, panel_w = h // 2, w // 2

        # Create panels
        # Panel 1: RGB with features (top 100 quality)
        panel1 = self.visualize_features(rgb_image, keypoints, max_features_to_show=100) if keypoints else rgb_image.copy()
        panel1 = cv2.resize(panel1, (panel_w, panel_h))
        cv2.putText(panel1, "1. Feature Detection", (5, 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Panel 2: Depth overlay (0.1-1.5m range)
        if depth_image is not None:
            panel2 = self.visualize_depth_overlay(rgb_image, depth_image, alpha=0.6, max_range=max_depth_range)
        else:
            panel2 = rgb_image.copy()
        panel2 = cv2.resize(panel2, (panel_w, panel_h))
        cv2.putText(panel2, "2. Depth Overlay", (5, 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Panel 3: Feature tracks (top 100 quality matches)
        panel3 = self.visualize_feature_tracks(rgb_image, keypoints, prev_keypoints, matches, max_tracks_to_show=100)
        panel3 = cv2.resize(panel3, (panel_w, panel_h))
        cv2.putText(panel3, "3. Feature Tracking", (5, 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Panel 4: Point cloud projection (0.1-1.5m range)
        if depth_image is not None and camera_matrix is not None:
            panel4 = self.visualize_point_cloud_projection(rgb_image, depth_image,
                                                          camera_matrix, sample_rate=15, max_range=max_depth_range)
        else:
            panel4 = rgb_image.copy()
        panel4 = cv2.resize(panel4, (panel_w, panel_h))
        cv2.putText(panel4, "4. Point Cloud", (5, 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Combine panels
        top_row = np.hstack([panel1, panel2])
        bottom_row = np.hstack([panel3, panel4])
        combined = np.vstack([top_row, bottom_row])

        # Add pose information
        if pose is not None:
            pose_text = f"Pose: ({pose.x:.2f}, {pose.y:.2f}) theta={np.degrees(pose.theta):.1f}deg"
            cv2.putText(combined, pose_text, (10, combined.shape[0] - 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Add odometry diagnostics
        if odometry_diagnostics is not None:
            odom_source = odometry_diagnostics.get('last_source', 'none')
            preferred_mode = odometry_diagnostics.get('preferred_mode', 'none')
            depth_failures = odometry_diagnostics.get('depth_failures', 0)
            visual_failures = odometry_diagnostics.get('visual_failures', 0)

            # Color code by odometry source
            if 'depth' in odom_source:
                odom_color = (0, 255, 255)  # Cyan for depth
            elif 'visual' in odom_source:
                odom_color = (255, 0, 255)  # Magenta for visual
            else:
                odom_color = (128, 128, 128)  # Gray for none

            # Display odometry status
            odom_text = f"Odometry: {odom_source.upper()} (Preferred: {preferred_mode}) | Failures: D={depth_failures} V={visual_failures}"
            cv2.putText(combined, odom_text, (10, combined.shape[0] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, odom_color, 2)

        return combined
