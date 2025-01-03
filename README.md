# Motion Planning for RoboCup MSL Robots

This repository includes the Webots simulation for motion planning (local and global) of a RoboCup MSL Robot using a 4-wheel omni-drive. This repository is maintained by team ERA, IITK.

## Requirements

1. OpenCV v4.8.1 (C++ Version)
2. OMPL v1.6 (C++ Version, python bindings are not required)
2. Webots v2023b
3. cmake
4. boost

__Note__: Use following instructions for Ubuntu -

```
sudo apt update && sudo apt upgrade
sudo apt install libopencv-dev
sudo apt install libboost-all-dev
sudo apt install libompl-dev ompl-demos
```

For MacOS, you need specific homebrew packages.

## Description

### Input:
1. o1_data to o5_data and b1_data to b5_data = {x, y, theta, enc0, enc1, enc2, enc3} : state of each bot
2. /o1/decision_target_data = {x, y, theta} : final state of o1 given by decision module

### Output:
1. /o1/target_pos = {x, y, theta, vx, vy, omega, speed, idx, path_length} : next target state of o1

## How to Run

- Install Webots and OpenCV
- Clone the repository and go to ros_ws directory `cd ros_ws`
- Build the directory `colcon build`
- Source the ros2 package `source install/local_setup.bash`
- Run the package by `ros2 run subscriber_package main`

- You can test the final target point (to be given by decision module) by `ros2 topic pub decision_target_data std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 0.0]}"`, where the array has data in format [x, y, theta]

## Code Organisation
- src
  - `Coordinate.hpp` => It has definitions of `wheel_angular_vel` struct and `Point2D` struct
  - `Motion.hpp` => Has complete description of class `PID` and the class which implements motion control of the robot.
  - `Robot.hpp` => Provides code needed for interfacing the desired motion from `Motion.hpp` to actual wheel velocities via forward and inverse kinematics with odometry information.
  - `Visualise.hpp` => All the code necessary for the visualization of the current state of the robot.
  - `PathPlanner.hpp` => Code that needs to be imported in main for generating a set of points. Needs to be split into a corresponding PathPlanner.cpp file.

### Wheel Control and Local Motion Planning

- A simple PID controller was integrated with the system to minimize the error.
- Maintaining Orientation:
- Inverse Kinematic Equations:
- Forward Kinematic Equations:
- Control Diagram:

### Global Path Planning

## References

- <https://github.com/MasdikaAliman/Kinematic-4-Omniwheels-in-Webots>
