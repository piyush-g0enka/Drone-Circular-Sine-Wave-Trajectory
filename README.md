# Drone_Circular_Trajectory

This repository contains C++ code to control a MODAL AI drone, making it perform a circular-helical trajectory while maintaining its heading towards the center of the circle. The code is designed for autonomous flight, enabling smooth and precise circular motion with real-time heading adjustments.

## Features

- **Circular-Helical Trajectory:** The drone follows a 3D path in the shape of a helix, maintaining a fixed radius in the horizontal plane while ascending or descending.
- **Heading Control:** The drone's heading (yaw) is adjusted continuously to point towards the center of the circle during flight.
- **Parameter Customization:** Users can easily configure parameters such as radius, altitude change rate, and speed of the trajectory.
  
## Prerequisites

1. **MODAL AI Drone** with compatible flight controllers.
2. **C++ Compiler** supporting C++11 or later.
3. **Drone SDK**: Ensure the appropriate SDK for controlling the drone is installed and integrated.

## Installation

Clone this repository:

```bash
git clone https://github.com/yourusername/Drone_Circular_Trajectory.git
cd Drone_Circular_Trajectory
