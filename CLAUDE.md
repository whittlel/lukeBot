# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

LukeBot is a SLAM (Simultaneous Localization and Mapping) robot built on:
- **Camera**: OAK-D IOT-75 (RGB + stereo depth, on-device YOLO)
- **Processing**: Jetson Orin Nano (SLAM, planning)
- **Motor Control**: Arduino Mega (low-level motor driver)
- **Chassis**: Mecanum wheels (X-pattern, omnidirectional movement)

The robot performs autonomous exploration using depth-based odometry (ICP), visual odometry fallback, and YOLO-based obstacle detection.

## Development Commands

### Environment Setup

```bash
# Setup virtual environment (first time)
./setup_env.sh  # Linux/Jetson
# or
setup_env.bat   # Windows

# Activate environment
source .venv/bin/activate  # Linux/Jetson
# or
.venv\Scripts\activate     # Windows
```

### Running the Robot

```bash
# Main robot program (requires hardware)
python main.py

# Test individual components
python tests/test_camera_gui.py      # Camera with GUI (recommended for visualization)
python tests/test_camera_simple.py   # Camera test (OpenCV only)
python tests/test_yolo.py           # YOLO object detection
python tests/test_motors.py         # Motor control
python tests/test_slam.py           # SLAM components
python tests/test_wheels_functional.py  # Mecanum wheel patterns

# Utility scripts
python scripts/visualize_map.py              # View saved maps
python scripts/capture_panoramic_snapshot.py # Capture panoramic image
python scripts/calibrate_camera.py          # Camera calibration
```

### Arduino Firmware

Upload `arduino/lukeBot_motor_control/lukeBot_motor_control.ino` to Arduino Mega using Arduino IDE.

**Serial Protocol**: Commands sent via USB serial (115200 baud)
- Format: `COMMAND,param1,param2`
- Examples: `MOVE_FORWARD,50`, `TURN_LEFT,45`, `STOP`

## Architecture Overview

### Multi-Layer System Architecture

```
┌─────────────────────────────────────────────────────┐
│  main.py (LukeBot orchestrator)                     │
├─────────────────────────────────────────────────────┤
│  Camera Layer          │  SLAM Layer                │
│  - OakDCamera          │  - SLAMEngine              │
│  - PanoramicCamera     │  - DepthOdometry (ICP)     │
│  - YOLO on-device      │  - VisualOdometry (ORB)    │
│                        │  - MapBuilder              │
│                        │  - ExplorationPlanner      │
│                        │  - ObstacleAvoidance       │
├─────────────────────────────────────────────────────┤
│  Motor Control Layer                                │
│  - ArduinoInterface (Serial communication)          │
│  - MotionPlanner (High-level commands)              │
│  - PathPlanner (A* planning)                        │
├─────────────────────────────────────────────────────┤
│  Arduino Firmware (lukeBot_motor_control.ino)       │
│  - L298N driver control                             │
│  - Mecanum wheel kinematics                         │
└─────────────────────────────────────────────────────┘
```

### Camera System Architecture

The camera system uses **DepthAI v2 API** (not v3) with a single unified pipeline:

**OakDCamera** (`src/camera/oakd_camera.py`):
- Single `depthai.Pipeline` combining YOLO, RGB, depth, and IMU
- YOLO runs on-device (OAK-D VPU) for real-time object detection
- RGB camera (CAM_A): 1280x720 for SLAM pose estimation
- Stereo cameras (CAM_B/CAM_C): Depth computation and panoramic view
- IMU (BNO085): Provides gyroscope/accelerometer for sensor fusion

**PanoramicCamera** (optional, integrated in `OakDCamera.create_panoramic_view()`):
- Stitches all three cameras for ~105° horizontal FOV
- Used for visualization and enhanced obstacle detection
- SLAM still uses stable center camera (CAM_A) for tracking

**Key Decision**: SLAM uses center RGB camera for stable pose estimation, while panoramic view enhances situational awareness.

### SLAM System Architecture

**Hybrid Odometry Approach** (configurable in `config/slam_config.yaml`):

1. **Primary: Depth-Based ICP Odometry** (`src/slam/depth_odometry.py`)
   - Iterative Closest Point on 3D point clouds
   - Works in low-texture environments (white/tan walls)
   - Faster and more robust than feature-based methods
   - IMU fusion for improved rotation estimation

2. **Fallback: Visual Odometry** (`src/slam/visual_odometry.py`)
   - ORB feature detection and matching
   - Used when depth data insufficient
   - IMU-assisted pose estimation

3. **Odometry Modes**:
   - `hybrid` (default): Depth ICP + visual fallback
   - `depth`: Depth-only (fastest)
   - `visual`: Visual-only (legacy)

**Map Building** (`src/slam/map_builder.py`):
- Occupancy grid (5cm resolution)
- Probabilistic mapping (Bayesian updates)
- Point cloud representation for 3D understanding

**Key Insight**: The robot was tuned for small indoor spaces with poor texture. Depth-based odometry is critical for reliable tracking.

### Motor Control Flow

```
High-level command (Python)
    ↓
MotionPlanner.move_forward(speed) / turn_left(angle)
    ↓
ArduinoInterface.send_command("MOVE_FORWARD,50")
    ↓
Serial USB → Arduino Mega (115200 baud)
    ↓
lukeBot_motor_control.ino parseCommand()
    ↓
Mecanum wheel kinematics calculation
    ↓
L298N motor drivers (PWM control)
    ↓
4x DC motors (Mecanum wheels)
```

**Mecanum Wheel Pattern**: X-pattern configuration
- Front-Left/Rear-Right: Rollers at 45° (↘)
- Front-Right/Rear-Left: Rollers at -45° (↙)
- Enables omnidirectional movement (forward, strafe, rotate, diagonal)

**Motor Wiring** (defined in `config/robot_config.yaml`):
- L298N #1 (Left): FL and RL motors (pins 2-7)
- L298N #2 (Right): FR and RR motors (pins 8-12, 44)
- **Note**: Pin 13 avoided due to built-in LED PWM interference

### Autonomous Exploration

**Current Implementation** (`main.py:_autonomous_exploration()`):
- **Simple reactive behavior** (not frontier-based)
- Move forward until obstacle detected (<25cm threshold)
- Turn randomly (5-10° increments) when blocked
- Dual obstacle detection:
  1. YOLO detections from camera (dynamic obstacles)
  2. Depth sensor center region (static obstacles)

**Why Simple?**: Optimized for small spaces (like a bedroom). The robot was tuned down from complex frontier-based exploration because:
- Reduced speeds (20 PWM) for stable feature tracking
- Slow turns (2-10°) to avoid motion blur
- Small detection thresholds (25cm vs original 70cm)

**Key Tuning** (from git commit in `<env>`):
- Depth unit bug fix: sensor returns meters, not millimeters
- Depth ROI: 35-65% vertical to avoid detecting floor
- Uses 10th percentile depth instead of minimum (noise robustness)

## Configuration Files

All configs in `config/` directory:

- **camera_config.yaml**: OAK-D camera settings, YOLO model path, depth parameters
- **robot_config.yaml**: Physical dimensions, wheel specs, motor pins, speeds
- **slam_config.yaml**: Odometry modes, ICP settings, mapping parameters, exploration thresholds

**Critical Settings**:
- `slam.odometry_mode`: Choose `"hybrid"` (default), `"depth"`, or `"visual"`
- `motion.default_speed`: 20 (tuned for tracking stability, not speed)
- `motion.default_turn_speed`: 3 (extremely slow for feature stability)
- `slam.depth_odometry.sample_rate`: 30 (samples every 30th pixel for Jetson Nano performance)

## Key Runtime Controls

When running `main.py`:

- `q` - Quit
- `p` - Pause/Resume
- `w/a/s/d` - Manual control (forward/left/backward/right)
- `Space` - Stop motors
- `e` - Toggle autonomous exploration
- `v` - Toggle panoramic view (105° FOV)
- `m` - Cycle visualization modes (Normal/SLAM Debug/Side-by-Side/Multi-Panel)
- `o` - Toggle odometry mode (Hybrid/Visual/Depth)

**Visualization Modes**:
0. Normal: Panoramic camera with YOLO detections
1. SLAM Debug: Features, tracking, depth overlay
2. Side-by-Side: Panoramic (left) vs SLAM view (right)
3. Multi-Panel: 4-panel debug view

## Important Implementation Details

### DepthAI Version
- **MUST use DepthAI v2** (not v3): `depthai>=2.27.0,<3.0.0`
- v3 API is incompatible with existing pipeline code
- YOLO models are in `.blob` format (OpenVINO compiled for OAK-D VPU)

### Depth Data Units
- **Depth values are in METERS**, not millimeters
- Bug was fixed where code expected mm (see git commit message)
- Valid range: 0.05m to 10m (5cm to 10m)

### Performance Tuning for Jetson Nano
- Keyframe interval: 5 frames (process every 5th frame)
- ICP max iterations: 5 (reduced from typical 20-30)
- Point cloud sampling: every 30th pixel
- Main loop delay: 0.05s (20 Hz, reduced from 0.01s to lighten Arduino load)

### Serial Communication
- **Non-blocking reads** from Arduino
- Commands are fire-and-forget (no acknowledgment required)
- Auto-detect Arduino port or specify in `robot_config.yaml`

### Map Storage
- Auto-save every 60 seconds
- Saved to `data/maps/` with timestamp
- Format: YAML occupancy grid + metadata

## Common Development Patterns

### Adding New Motor Commands

1. Define command in `src/motor_control/arduino_interface.py`
2. Add parsing logic in Arduino firmware (`parseCommand()`)
3. Implement motor control logic in Arduino
4. Add high-level wrapper in `src/motor_control/motion_planner.py`

### Adding New YOLO Models

1. Convert model to `.blob` format (OpenVINO for Myriad X VPU)
2. Place in `data/models/<model_name>/`
3. Update `config/camera_config.yaml` with model path and input size
4. Ensure confidence threshold is appropriate

### Modifying SLAM Parameters

1. Edit `config/slam_config.yaml`
2. Restart `main.py` (configs loaded at startup)
3. Use `o` key to toggle odometry modes for live testing
4. Use `m` key to cycle visualization modes for debugging

### Testing Without Hardware

- Camera tests will fail without OAK-D connected
- Motor tests will fail without Arduino connected
- SLAM components can be tested with recorded data (see `tests/test_slam.py`)

## Known Issues & Limitations

1. **No formal unit tests**: Tests are integration tests requiring hardware
2. **Single-threaded main loop**: Camera and SLAM run in main thread
3. **No loop closure**: SLAM accumulates drift over time (loop closure is configured but not fully implemented)
4. **Manual calibration**: Camera intrinsics are read from device, but no automatic extrinsic calibration between cameras
5. **Limited error recovery**: Hardware disconnects require restart

## Documentation References

- **DEPTH_ODOMETRY_GUIDE.md**: Detailed guide on depth-based ICP odometry
- **docs/PANORAMIC_CAMERA.md**: Panoramic camera system architecture
- **README.md**: Hardware setup and general usage
- **ODOMETRY_STABILIZATION_TODO.md**: Pending odometry improvements
