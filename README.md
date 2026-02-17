# pycuvslam_ros2

A ROS 2 wrapper for NVIDIA **cuVSLAM** (CUDA Visual SLAM), providing robust visual-inertial odometry / SLAM using RGBD cameras (e.g., Orbbec Gemini 2, RealSense).

## Overview

This package subscribes to synchronized RGB and Depth images from a camera node and uses the `cuvslam` Python API to compute odometry. It publishes the robot's pose as `nav_msgs/Odometry` and broadcasts the corresponding TF transform.

## Features

- **RGBD Odometry**: Uses depth for scale-accurate tracking.
- **GPU Acceleration**: Leverage CUDA for high-performance tracking.
- **ROS 2 Integration**: Standard topics and TF support.
- **Rerun Visualization**: Optional integration with [Rerun](https://rerun.io/) for 3D visualization of the camera trajectory and sparse point cloud.

## Requirements

- **OS**: Linux (Ubuntu 20.04/22.04 recommended)
- **ROS 2**: Humble / Iron / Jazzy
- **Hardware**: NVIDIA GPU with CUDA support
- **Python Dependencies**:
  - `cuvslam` (NVIDIA Isaac / cuVSLAM SDK)
  - `numpy`
  - `opencv-python`
  - `rerun-sdk` (optional, for visualization)

## Installation

1.  **Clone the repository** into your ROS 2 workspace `src` folder:
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/your-username/pycuvslam_ros2.git
    ```

2.  **Install dependencies**:
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```
    *Note: `cuvslam` must be installed separately as it is not a standard rosdep key.*

3.  **Build**:
    ```bash
    colcon build --symlink-install --packages-select pycuvslam_ros2
    ```

4.  **Source**:
    ```bash
    source install/setup.bash
    ```

## Usage

### 1. Start your Camera
Ensure your camera driver is running and publishing RGB, Depth, and CameraInfo.
Example (Orbbec):
```bash
ros2 launch orbbec_camera_driver gemini_330_series.launch.py
```

### 2. Launch pycuvslam
```bash
ros2 launch pycuvslam_ros2 pycuvslam.launch.py
```

### Launch Arguments

| Argument | Default | Description |
| t --- | --- | --- |
| `rgb_topic` | `/camera/color/image_raw` | Topic for RGB image |
| `depth_topic` | `/camera/depth/image_raw` | Topic for Depth image (aligned to RGB) |
| `camera_info_topic` | `/camera/color/camera_info` | Topic for Camera Info (run once at startup) |
| `odom_frame` | `odom` | Name of the odometry frame |
| `base_frame` | `camera_link` | Name of the robot/camera base frame |
| `publish_tf` | `true` | Whether to broadcast odom->base_frame transform |
| `use_gpu` | `true` | Enable GPU acceleration in cuVSLAM |
| `use_rerun` | `false` | Enable Rerun visualization |

## Node Details

### Subscribed Topics

*   **`rgb_topic`** (`sensor_msgs/Image`): Color image stream (RGB8/BGR8/Mono8).
*   **`depth_topic`** (`sensor_msgs/Image`): Depth image stream (16UC1).
*   **`camera_info_topic`** (`sensor_msgs/CameraInfo`): Camera intrinsics. Used only once at startup to initialize the tracker.

### Published Topics

*   **`~/odom`** (`nav_msgs/Odometry`): The computed odometry of the camera in the `odom` frame.
*   **`/tf`** (`tf2_msgs/TFMessage`): Transform from `odom_frame` to `base_frame`.

### Parameters

The node `pycuvslam_node` accepts the following parameters (mostly mapped from launch args):

*   `depth_scale_factor` (double, default: 1000.0): Scale to convert depth image values to meters.
*   `enable_depth_denoising` (bool, default: true): Apply median/gaussian blur to depth map.
*   `async_sba` (bool, default: true): Run Sparse Bundle Adjustment asynchronously.
*   `use_motion_model` (bool, default: true): Use constant velocity motion model for prediction.
*   `warmup_frames` (int, default: 30): Number of frames to process before publishing odom (allows auto-exposure to settle).

## Troubleshooting

- **"Tracking lost"**: Ensure your depth camera has valid data and isn't pointing at a featureless wall.
- **"Size mismatch"**: Ensure RGB and Depth images have the same resolution.
- **"Waiting for CameraInfo"**: Verify the camera driver is publishing camera info and the topic name is correct.
