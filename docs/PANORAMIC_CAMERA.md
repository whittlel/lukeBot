# Panoramic Camera System

## Overview

The panoramic camera system stitches together images from all three OAK-D cameras to provide a wide field-of-view (~105°) for enhanced visualization and obstacle detection.

## Camera Specifications

### Hardware
- **OAK-D IOT-75** with three cameras:
  - CAM_A (RGB): Center camera, 68.8° horizontal FOV, 1280x720
  - CAM_B (Left Stereo): Left camera, 71.9° horizontal FOV, 720p mono
  - CAM_C (Right Stereo): Right camera, 71.9° horizontal FOV, 720p mono
- **Stereo Baseline**: 4.0 cm between left/right cameras
- **Total FOV**: ~104.7° horizontal (estimated)

### Panoramic Output
- **Resolution**: 3840 x 720 pixels (stitched)
- **Layout**: [Left Stereo] [RGB Center] [Right Stereo]
- **Depth**: Stereo depth available from left/right cameras
- **Frame Rate**: 30 FPS (synchronized)

## Architecture

### Components

1. **PanoramicCamera** (`src/camera/panoramic_camera.py`)
   - Multi-camera capture and synchronization
   - Horizontal stitching with optional labels
   - Depth processing and obstacle detection
   - FOV calculations and calibration data

2. **Integration in main.py**
   - Dual camera system: Main OakDCamera + PanoramicCamera
   - Toggle between panoramic and single camera views
   - Enhanced obstacle detection using wider FOV
   - SLAM uses stable center camera for pose tracking

3. **Utilities**
   - `capture_panoramic_snapshot.py`: Standalone snapshot tool
   - `test_panoramic_view.py`: Interactive viewer (requires display)

## Usage

### Running LukeBot with Panoramic View

```bash
python3 main.py
```

**Controls:**
- `v` - Toggle between panoramic view (105°) and single camera view (69°)
- `e` - Toggle autonomous exploration mode
- `w/a/s/d` - Manual movement controls
- `q` - Quit

### Capturing Panoramic Snapshots

```bash
python3 scripts/capture_panoramic_snapshot.py
```

Output saved to: `data/panoramic/`
- Individual camera feeds (RGB, left, right)
- Stitched panoramic view
- Depth visualization

### View Modes

**Panoramic View (Default)**
- Wide 105° horizontal FOV
- All three cameras stitched horizontally
- Enhanced peripheral obstacle detection
- Best for autonomous navigation

**Single Camera View**
- Standard 69° horizontal FOV
- RGB camera only
- Lower computational overhead
- Best for focused tracking

## Integration with SLAM

### Pose Estimation
- SLAM uses **main OakDCamera** (center RGB) for stable pose tracking
- Visual odometry requires consistent frame-to-frame features
- Center camera provides stable, undistorted view

### Obstacle Detection
- Autonomous exploration uses **panoramic depth** when available
- Wider FOV detects obstacles approaching from sides
- Falls back to main camera if panoramic unavailable

### Mapping
- Map builder uses depth from either camera system
- Panoramic depth provides wider coverage per frame
- Faster map building with enhanced coverage

## Performance

### Benefits
- **52% wider FOV** than single camera (105° vs 69°)
- Better peripheral obstacle awareness
- Enhanced situational awareness for navigation
- More complete visual coverage per frame

### Considerations
- Two simultaneous camera pipelines
- Additional GPU/VPU processing for stitching
- Graceful degradation if panoramic fails

## Technical Details

### Image Stitching
- Simple horizontal concatenation (no complex blending)
- Mono cameras converted to BGR for consistency
- Height normalization to RGB camera resolution
- Optional camera labels for debugging

### Depth Processing
- Stereo depth from left/right camera pair
- Extended disparity mode for close-range accuracy
- Depth aligned to center RGB camera
- Used for obstacle detection and mapping

### Calibration
- Intrinsic parameters loaded from device
- FOV calculations from factory calibration
- Baseline distance: 4.0 cm
- No manual calibration required

## Example Output

See `data/panoramic/` for example captures showing:
- Wide-angle panoramic view
- Individual camera feeds
- Depth map visualization
- FOV coverage demonstration

## Future Enhancements

Potential improvements:
1. **Advanced stitching**: Blend overlap regions for seamless panorama
2. **Dynamic FOV optimization**: Adjust based on environment
3. **Multi-camera SLAM**: Use all cameras for pose estimation
4. **360° coverage**: Add rear cameras for full surround view
5. **Semantic segmentation**: Apply to full panoramic view

## Troubleshooting

**Panoramic camera fails to start**
- System automatically falls back to single camera
- Check USB bandwidth (multiple cameras = high data rate)
- Verify OAK-D device has all three cameras functional

**Low frame rate in panoramic mode**
- Normal with dual camera pipelines
- Toggle to single camera mode with 'v' key if needed
- Reduce resolution in camera_config.yaml if necessary

**Stitching artifacts**
- Ensure cameras are time-synchronized
- Check lighting conditions (consistent across all cameras)
- Mono cameras will have different appearance than RGB

## References

- OAK-D DepthAI documentation
- Camera calibration data from device
- SLAM integration in `src/slam/slam_engine.py`
- Main controller in `main.py`
