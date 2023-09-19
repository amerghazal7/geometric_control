# Geometric Control for Car-like Robots

This repository provides a geometric control solution for car-like robots using ROS Noetic. The control methodology is based on geometric properties and employs Model Predictive Control (MPC) to guide the robot along a desired trajectory.

## Features

- **Geometric Control**: Utilizes geometric properties of the system for control.
- **Odometry Feedback**: Takes odometry data to understand the robot's current state.
- **Waypoint Following**: Takes a file of waypoints that the robot should follow.
- **Longitudinal and Lateral Control**: Uses both longitudinal and lateral control methods to keep the robot on the track.


## Overview

The main scripts in the repository are:

1. **control_using_odom.py**: This script is the primary control script that uses odometry data. It subscribes to odometry data and publishes control commands. The script uses a 2D controller for the control logic.

2. **controller2d.py**: This script defines a 2D controller class. It takes in the current state and reference state and computes the control commands. The controller uses a Stanley control approach to compute the control commands (Also MPC version inside PythonClient/MPC).

3. **cutils.py**: This script contains utility functions for the controller. It has functions for transforming coordinates, computing errors, and other utility functions that aid in the control logic.

4. **live_plotter.py**: As the name suggests, this script is used for live plotting. It can plot the trajectory of the robot, reference trajectory, and other relevant data in real-time.

5. **mpc_utils.py**: This script contains utility functions for the model predictive control. It has functions for setting up and solving the optimization problem.


## Installation and Running

### Installing Required Librariesyou 
#### Install ROS Noetic and make sure it's working.
#### Install python libs:

```bash
# Install numpy
pip install numpy

# Install matplotlib
pip install matplotlib==2.2.5

# Install cvxpy
pip install cvxpy

# Install scipy
pip install scipy
```

#### Clone the repo into catkin_ws/src then:
```bash
cd catkin_ws
catkin_make
source devel/setup.sh
roslaunch geometric_control odom_geometric_control.launch
```
#### Checkout the launch file to specify waypoints file.

### Waypoints Generation Helper
**wayPointsGenration.ipynb** Inside **../data** folder, is a jupyter notebook that explains the process.
