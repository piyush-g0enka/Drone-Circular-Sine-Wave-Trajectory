# Drone Circular Sine-wave Trajectory

This repository contains modified offboard_control C++ code from px4_ros_com ROS2 package to control a MODAL AI VOXL2 m500 drone, making it perform a circular-helical trajectory while maintaining its heading towards the center of the circle. 

## Demonstration
![Drone Circular Trajectory](https://github.com/piyush-g0enka/Drone_Circular_Trajectory/blob/main/trajectory.png)

## Features

- **Circular-Helical Trajectory:** The drone follows a 3D path in the shape of a helix, maintaining a fixed radius in the horizontal plane while ascending or descending.
- **Heading Control:** The drone's heading (yaw) is adjusted continuously to point towards the center of the circle during flight.
- **Parameter Customization:** Users can easily configure parameters such as radius, altitude change rate, and speed of the trajectory.
  
## Prerequisites

1. **MODAL AI Drone** with compatible flight controllers.
2. **ROS2** ROS2 installed on the drone's computer
3. **PX4**: PX4 ROS2 packages 

## YouTube Video
[![Drone Circular Trajectory Video](https://github.com/piyush-g0enka/Drone_Circular_Trajectory/blob/main/yt.png)](https://www.youtube.com/watch?v=jx8FvLK_zhY)


## Usage

Put the offboard_control.cpp file in a ROS2 package in the drone and run it in OFFBOARD mode



