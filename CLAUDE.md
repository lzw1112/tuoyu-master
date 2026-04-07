# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Tuoyu Intelligent Connected Sensor Teaching Software — an educational platform for autonomous vehicle sensor systems (LiDAR, camera, GNSS/IMU, radar) built on ROS Melodic.

## Build Commands

This project uses Catkin (ROS build system). The workspace must be set up outside this repo:

```bash
# Standard build
cd ~/tuoyu_ws
catkin build

# Build specific packages
catkin build cyrobot_monitor_camera
catkin build cyrobot_monitor_lidar
catkin build cyrobot_monitor_gnss_imu

# Build IMU calibration dependencies first
catkin build code_utils
catkin build imu_utils
```

**Ceres Solver** (required for IMU calibration, build once):
```bash
cd driver/ceres-solver
mkdir build && cd build
cmake .. && make -j4 && sudo make install
```

**Cangaroo CAN bus tool** (optional):
```bash
cd driver/cangaroo
qmake -qt=qt5 && make && make install
```

## Running

```bash
# Main LiDAR system
roslaunch lidar.launch

# Process monitor
roslaunch process_monitor run.launch

# Source workspace before running
source ~/tuoyu_ws/devel/setup.bash
```

## Architecture

### Component Flow

```
Hardware Drivers (driver/)
    ↓ ROS topics (sensor_msgs, custom autoware_msgs)
Function Modules (function_achivement/)
    ↓ processed data
UI Applications (ui_camera, ui_lidar, ui_gnss_imu)
    ↓ visualization
Process Monitor (process_monitor/)
```

### Key Directories

- `common/` — Shared ROS message definitions (autoware_msgs, autoware_config_msgs). These are Autoware-derived message types used across all packages.
- `driver/` — Hardware drivers: rslidar_sdk (RoboSense LiDAR), lslidar_c16 (Leishen LiDAR), nmea_navsat_driver (GNSS), usb_cam (camera), ars_40X-master (Continental radar), socket_can-master (CAN bus).
- `function_achivement/` — Algorithm implementations: LiDAR detection pipeline (points_preprocessor → points_downsampler → lidar_euclidean_cluster_detect → lidar_shape_estimation), YOLOv4-Tiny camera detection (PyTorch), IMU calibration (Ceres-based Allan variance).
- `ui_camera/`, `ui_lidar/`, `ui_gnss_imu/` — Three Qt5 desktop applications, one per sensor type. Each follows the same structure: `main_window.cpp` (GUI logic), `qnode.cpp` (ROS communication layer), `.ui` files (Qt Designer layouts).
- `process_monitor/` — ROS node that monitors system processes and resource usage.

### UI Package Pattern

All three UI packages share the same architecture:
- `src/src/main_window.cpp` — Large monolithic GUI file (~55–75KB each), handles all UI interactions
- `src/src/qnode.cpp` — ROS node wrapper; subscribes/publishes topics, bridges ROS ↔ Qt signals
- `src/include/` — Headers for main_window and qnode
- `src/ui/` — Qt Designer `.ui` files
- `src/resources/` — Images, icons, QSS stylesheets

### LiDAR Detection Pipeline

Point cloud data flows through these packages in order:
1. `points_preprocessor` — Ground removal, coordinate transform
2. `points_downsampler` — Voxel grid filtering
3. `lidar_euclidean_cluster_detect` — Euclidean clustering, object detection
4. `lidar_shape_estimation` — Bounding box fitting, shape classification

### Topic Bridge

`driver/topic_transmit.py` converts rslidar point cloud topics to Autoware-compatible format when using RoboSense hardware.

## Tech Stack

- **ROS Melodic** + **Catkin** — middleware and build system
- **C++11/14** — UI applications and sensor processing
- **Python** — GNSS driver (nmea_navsat_driver), YOLOv4 detection node
- **Qt5** — Desktop GUI (Qt5Core, Qt5Widgets, Qt5Multimedia)
- **PCL** — Point cloud processing
- **OpenCV** — Image processing
- **PyTorch** — YOLOv4-Tiny object detection
- **Ceres Solver** — IMU calibration optimization
- **RViz** — 3D visualization (embedded in UI apps)
