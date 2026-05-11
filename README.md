# Vision-Guided Robot Pick System

A ROS 2 vision-guided pick system for the **Fairino FR5** robot arm using an **Orbbec** depth camera, **YOLOv8** object detection, and an AI agent for natural language commands. Includes a **Unity** digital twin for real-time visualization.

Research project at **Telos**.

## Architecture

The system is organized as a layered ROS 2 pipeline:

| Layer | Node | Description |
|-------|------|-------------|
| 1 | `camera_check` | Verify Orbbec color/depth streams and intrinsics |
| 2 | `detector_2d` | YOLOv8 object detection on color image |
| 3 | `position_3d` | Fuse 2D detections with point cloud for 3D coordinates |
| 4 | `calibration` | Hand-eye calibration (eye-to-hand) via point correspondence + SVD |
| 5 | `commander` | Move robot TCP to detected object positions |
| 6 | `ai_agent` | Natural language commands via Gemini function calling |

### Unity Digital Twin

C# scripts bridge ROS 2 topics to a Unity simulation of the FR5:

- **FR5JointSubscriber** — subscribes to joint states and mirrors robot pose
- **DetectionUI / DetectionSubscriber** — visualizes YOLO detections
- **AICommandUI** — sends natural language commands to the AI agent
- **FR5KeyboardController** — manual jog control

## ROS 2 Packages

```
ros2/
├── vision_robot_pick/    # Vision pipeline + robot commander + AI agent
└── ros2_unity/           # Fairino SDK bridge, joint publisher, jog/cmd servers
```

## Prerequisites

- ROS 2 (Humble)
- Python 3.10+
- Orbbec camera ROS 2 driver
- Fairino robot SDK (`fairino` Python package, included)
- Unity with [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)

### Python Dependencies

```bash
pip install ultralytics opencv-python numpy google-generativeai
```

## Setup

```bash
# Clone into your ROS 2 workspace
cd ~/ros2_ws/src
git clone https://github.com/prof-lijar/vision-guided-robot-pick-system.git

# Build
cd ~/ros2_ws
colcon build --packages-select vision_robot_pick ros2_unity
source install/setup.bash
```

## Usage

Run each layer in order to verify the pipeline:

```bash
# 1. Verify camera streams
ros2 run vision_robot_pick camera_check

# 2. Run 2D object detection
ros2 run vision_robot_pick detector_2d

# 3. Get 3D positions (optionally filter by class)
ros2 run vision_robot_pick position_3d
ros2 run vision_robot_pick position_3d --ros-args -p target_class:='cell phone'

# 4. Run hand-eye calibration (collect 10+ point pairs)
ros2 run vision_robot_pick calibration

# 5. Command robot to pick detected objects
ros2 run vision_robot_pick robot_commander

# 6. Natural language control
ros2 run vision_robot_pick ai_agent
ros2 topic pub /ai_command std_msgs/String "data: 'go to phone with speed 20%'" --once
```

### Unity Bridge

```bash
# Start joint state publisher for digital twin
ros2 run ros2_unity joint_bridge
ros2 run ros2_unity state_publisher
```

## Calibration

The system uses eye-to-hand calibration with SVD-based point correspondence. Move the robot TCP to touch detected objects, collecting at minimum 10 point pairs. Results are saved to `config/calibration.yaml`.

## Tech Stack

- **Robot**: Fairino FR5
- **Camera**: Orbbec RGB-D
- **Detection**: YOLOv8 (ultralytics)
- **3D Estimation**: Point cloud depth fusion
- **Calibration**: SVD least squares
- **AI Agent**: Gemini 2.5 Flash with function calling
- **Digital Twin**: Unity + ROS-TCP-Connector
- **Middleware**: ROS 2
