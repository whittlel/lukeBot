# LukeBot - SLAM Robot Project

A modular SLAM robot system using OAK-D IOT-75 camera, Jetson Orin Nano, Arduino Mega, and Mecanum wheel chassis.

## Hardware Components

- **Camera**: OAK-D IOT-75 (RGB + Stereo depth cameras)
- **Processing**: Jetson Orin Nano (on-board SLAM processing)
- **Motor Control**: Arduino Mega (low-level motor control)
- **Chassis**: Mecanum wheel chassis (X-pattern, 4 wheels)

## Project Overview

This project implements a SLAM (Simultaneous Localization and Mapping) robot that can:
- Run YOLO object detection on the OAK-D camera (on-device processing)
- Stream RGB + depth data to Jetson for SLAM
- Map indoor environments using visual SLAM
- Control Mecanum wheels via high-level commands to Arduino

## Architecture

### OAK-D Camera
- Runs YOLO detection on-device (using DepthAI v2)
- Streams RGB camera (CAM_A) for YOLO and visualization
- Streams stereo cameras (CAM_B, CAM_C) for depth and SLAM
- Provides synchronized RGB + depth + YOLO detections

### Jetson Orin Nano
- Receives camera streams from OAK-D
- Runs visual SLAM algorithm (stereo visual odometry)
- Builds and stores maps
- Sends high-level motion commands to Arduino

### Arduino Mega
- Receives high-level commands via USB Serial
- Handles low-level motor control (PWM, L298N drivers)
- Controls 4 Mecanum wheels in X-pattern

## Project Structure

```
lukeBot/
├── README.md                    # This file
├── requirements.txt            # Python dependencies
├── config/                      # Configuration files
│   ├── camera_config.yaml      # OAK-D camera settings
│   ├── robot_config.yaml       # Robot physical parameters
│   └── slam_config.yaml        # SLAM algorithm parameters
├── src/                         # Main source code
│   ├── camera/                  # OAK-D camera integration
│   ├── slam/                    # SLAM implementation
│   ├── motor_control/           # Arduino communication
│   └── utils/                   # Utility functions
├── arduino/                     # Arduino firmware
│   └── lukeBot_motor_control/
├── scripts/                     # Utility scripts
├── data/                        # Runtime data storage
└── tests/                       # Unit tests
```

## Setup Instructions

### 1. Prerequisites

- Jetson Orin Nano with JetPack installed
- OAK-D IOT-75 camera connected via USB
- Arduino Mega connected via USB
- Python 3.8+ on Jetson

### 2. Setup Virtual Environment

**Windows:**
```batch
# Run setup script (creates .venv and installs dependencies)
setup_env.bat

# Or manually:
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

**Linux/Mac (or Jetson):**
```bash
# Run setup script (creates .venv and installs dependencies)
chmod +x setup_env.sh
./setup_env.sh

# Or manually:
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

**Note:** Always activate the virtual environment before running scripts:
- Windows: `activate_env.bat` or `.venv\Scripts\activate`
- Linux/Mac: `source activate_env.sh` or `source .venv/bin/activate`

### 3. Configure Hardware

#### OAK-D Camera
- Ensure camera is connected via USB
- Verify camera is detected: `python scripts/test_camera.py`

#### Arduino Mega
- Upload firmware from `arduino/lukeBot_motor_control/lukeBot_motor_control.ino`
- Verify connection: `python scripts/test_motors.py`

#### Motor Wiring
- L298N #1 (Left side: Front-left, Rear-left):
  - IN1→2, IN2→3, IN3→4, IN4→5, ENA→6, ENB→7
- L298N #2 (Right side: Front-right, Rear-right):
  - IN1→8, IN2→9, IN3→10, IN4→11, ENA→12, ENB→13

### 4. Configuration

Edit configuration files in `config/`:
- `camera_config.yaml`: Camera settings, YOLO model path
- `robot_config.yaml`: Robot dimensions, wheel parameters
- `slam_config.yaml`: SLAM algorithm parameters

### 5. Activate Environment and Run

**Windows:**
```batch
# Activate virtual environment
activate_env.bat

# Main SLAM robot program
python main.py

# Or test individual components
python tests/test_camera_gui.py  # Test camera with GUI (recommended)
python tests/test_camera_simple.py  # Test camera (simple OpenCV)
python tests/test_yolo.py        # Test YOLO detection
python tests/test_motors.py      # Test motor control
```

**Linux/Mac (or Jetson):**
```bash
# Activate virtual environment
source activate_env.sh

# Main SLAM robot program
python main.py

# Or test individual components
python tests/test_camera_gui.py  # Test camera with GUI (recommended)
python tests/test_camera_simple.py  # Test camera (simple OpenCV)
python tests/test_yolo.py        # Test YOLO detection
python tests/test_motors.py      # Test motor control
```

## Usage

### Basic SLAM Mapping

1. Start the robot: `python main.py`
2. The robot will begin mapping the environment
3. Maps are saved to `data/maps/`
4. View maps: `python scripts/visualize_map.py`

### Motor Control

The robot accepts high-level commands:
- `MOVE_FORWARD,<speed>` - Move forward (speed: 0-100)
- `MOVE_BACKWARD,<speed>` - Move backward
- `TURN_LEFT,<angle>` - Turn left (angle in degrees)
- `TURN_RIGHT,<angle>` - Turn right
- `STOP` - Stop all motors

## Development

### Running Tests

```bash
# Run all tests
python -m pytest tests/

# Run specific test
python -m pytest tests/test_camera.py
```

### Code Structure

- `src/camera/`: OAK-D camera integration, YOLO detection
- `src/slam/`: Visual SLAM implementation
- `src/motor_control/`: Arduino communication, wheel control
- `src/utils/`: Logging, data structures

## Troubleshooting

### Camera Issues
- Verify USB connection: `lsusb | grep OAK`
- Check DepthAI installation: `python -c "import depthai; print(depthai.__version__)"`
- Ensure using DepthAI v2 (not v3)

### Arduino Issues
- Verify serial port: `ls /dev/tty*` (Linux) or check Device Manager (Windows)
- Update port in `config/robot_config.yaml`
- Check baud rate matches (default: 115200)

### Motor Control Issues
- Verify Arduino firmware is uploaded
- Check wiring connections
- Test motors individually using `scripts/test_motors.py`

## License

[Add your license here]

## Acknowledgments

- OAK-D camera examples from `oakd/` folder
- DepthAI library for camera integration
- OpenCV for SLAM implementation

