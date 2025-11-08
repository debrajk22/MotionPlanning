# Motion Planning for RoboCup MSL Robots

This repository includes the Motion planning codebase of a RoboCup MSL Robot using a 4-wheel omni-drive. This repository is maintained by team ERA, IITK.
There are 4 workspaces:
1. path\_planner
2. controller
3. webots\_sim
4. python\_sim
5. stm\_comm

## Requirements and Installation Guide

1. OpenCV v4.8.1 (C++ Version)
2. OMPL v1.6 (C++ Version, python bindings are not required)
2. Webots
3. cmake
4. Boost
5. Eigen
6. ROS 2 Jazzy Jalisco
7. Webots-ROS2 package
8. Pygame
9. Pyserial
10. PyYAML
11. numpy

```
sudo apt update && sudo apt upgrade
sudo apt install libopencv-dev
sudo apt install libboost-all-dev
sudo apt install libompl-dev ompl-demos
sudo apt-get install libeigen3-dev
pip install pygame
pip install pyserial
```

Install OMPL:
Download the OMPL installation script from https://ompl.kavrakilab.org/install-ompl-ubuntu.sh and run:
```
chmod u+x install-ompl-ubuntu.sh
./install-ompl-ubuntu.sh
```

Install Acado:
```
git clone https://github.com/acado/acado.git
cd acado
mkdir build
cd build
cmake ..
make
sudo make install
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

## Path Planner Description

### Input:
1. o1\_data to o5\_data and b5\_data to b5\_data = {x, y, theta, enc0, enc1, enc2, enc3} : state of each bot
2. /o1/decision\_target\_data = {x, y, theta} : final state of o1 given by decision module

### Output:
1. /o1/target\_pos = {x, y, theta, vx, vy, omega, speed, idx, path\_length} : next target state of o1

### How to Run
Build the package:
```
cd path_planner
colcon build
source install/local_setup.bash
```

Run the module:
```
ros2 run path_planner_pkg main
```

- You can test the final target point (to be given by decision module) by `ros2 topic pub /decision_target_data std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 0.0]}"`, where the array has data in format [x, y, theta]

### Code Organisation
- src
  - `Coordinate.hpp` => It has definitions of `wheel_angular_vel` struct and `Point2D` struct
  - `Motion.hpp` => Has complete description of class `PID` and the class which implements motion control of the robot.
  - `Robot.hpp` => Provides code needed for interfacing the desired motion from `Motion.hpp` to actual wheel velocities via forward and inverse kinematics with odometry information.
  - `Visualise.hpp` => All the code necessary for the visualization of the current state of the robot.
  - `PathPlanner.hpp` => Code that needs to be imported in main for generating a set of points. Needs to be split into a corresponding PathPlanner.cpp file.

## MPC Controller Description

### Input:
1. o1\_data = {x, y, theta, enc0, enc1, enc2, enc3} : current state of o1
2. /o1/target\_pos = {x, y, theta, vx, vy, omega, speed, nearest\_point\_idx, path\_size} : next target state of o1

### Output:
1. /o1/cmd\_vel = {vx, vy, omega} : next predicted velocities of o1

### How to Run
Build the package:
```
cd controller
colcon build
source install/local_setup.bash
```

Run the module:
```
ros2 run controller_pkg controller
```

## Webots Simulation Description

### The module publishes:
- `/robot_data`: {x-coordiante, y-coordinate, z-orientation, encoder1, encoder2, encoder3, encoder4} in the same mentioned order

### The module subscribes to:
- `/cmd_vel`: The `linear.x`, `linear.y`, and `angular.z` values are used to control the motion of the robot.

### How to Run
Build the package:
```
cd webots_pkg
colcon build
source install/local_setup.bash
```

Run the module:
```
ros2 launch webots_pkg robot_launch.py
```

## Python Simulation Description
It's working is similar to the webot simulation.

### How to Run
```
cd python_sim/src/python_sim_pkg/python_sim_pkg
python3 robot_sim.py
```

## STM Microcontroller Communicator Description
It subscribes to the necessary topics and generates velocities of the individual wheels and sends them to the STM microcontroller by serial communication. Make sure that the STM is connected to the correct port in the system.

### How to Run
```
cd stm_comm
python3 serial_comm.py
```
## References
- <https://github.com/MasdikaAliman/Kinematic-4-Omniwheels-in-Webots>
