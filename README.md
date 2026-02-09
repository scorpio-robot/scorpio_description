# scorpio_description

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test](https://github.com/scorpio-robot/scorpio_description/actions/workflows/build_and_test.yml/badge.svg)](https://github.com/scorpio-robot/scorpio_description/actions/workflows/build_and_test.yml)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

## 1. Overview

`scorpio_description` is a ROS2 package that provides the robot description for the Scorpio robot, including URDF (Unified Robot Description Format) and SDF (Simulation Description Format) files. This package supports both simulation environments (e.g., Gazebo) and real-world robot deployment.

Features:

- URDF models for robot kinematics and visualization
- SDF models for Gazebo simulation
- Integration with ROS2 control systems
- Support for various sensors and actuators

## 2. Quick Start

### 2.1 Setup Environment

Ubuntu 22.04: [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### 2.2 Create Workspace

### 2.2.1 Clone

```sh
pip3 install vcs2l
pip3 install xmacro
```

```sh
mkdir -p ~/scorpio_ws
cd ~/scorpio_ws
```

```sh
git clone https://github.com/scorpio-robot/scorpio_description src/scorpio_description
```

```sh
vcs import src < src/scorpio_description/dependencies.repos
```

#### 2.2.3 Build

```sh
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
```

### 2.3 Running

To launch the robot description in RViz:

```sh
ros2 launch scorpio_description robot_description_launch.py
```
