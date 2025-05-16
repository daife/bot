# Sensor Fusion Package

This ROS 2 package uses the `robot_localization` package to fuse wheel odometry and IMU data for accurate robot localization. The package provides an Extended Kalman Filter (EKF) implementation that combines these sensor sources to generate a more accurate odometry estimate and publish the appropriate TF transforms.

## Overview

The package fuses the following sensor data:
- Wheel odometry (position and velocity from encoders)
- IMU data (orientation, angular velocity, and linear acceleration)

The fusion result provides improved odometry that accounts for wheel slip and other inaccuracies in pure wheel odometry.

## Usage

To launch the sensor fusion node:

```bash
ros2 launch sensor_fusion_pkg sensor_fusion.launch.py
```

## Configuration

The EKF configuration is stored in `config/ekf_config.yaml`. You may need to modify this file to:
- Adjust the process noise parameters based on your robot's characteristics
- Change the topic names in the remappings section of the launch file if your topics use different names
- Modify which sensor data dimensions are used in the fusion process

## Topic Subscriptions

- `/odom` - Wheel odometry input
- `/imu/data` - IMU data input

## Topic Publications

- `/odometry/filtered` - Fused odometry output
- `/tf` - TF transforms (odom → base_link)

## Requirements

- ROS 2
- robot_localization package