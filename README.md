# Experimental Robotics laboratory Assignment 2

ROS 2 companion package providing the Gazebo environment for an experimental-robotics mission based on ArUco marker detection and planned inspection.

The mission logic is maintained in [exp_rob_ass2](https://github.com/MattiaTinfena/exp_rob_ass2). This repository provides the robot model, simulation world, sensor bridges, visualisation and localisation configuration.

## Contents

- Robot description and simulation resources.
- Gazebo world and ArUco marker model.
- RViz configuration.
- Extended Kalman Filter configuration through `robot_localization`.
- ROS–Gazebo bridges for velocity, odometry, joint states, camera, IMU and laser data.

## Requirements

ROS 2 Jazzy, Gazebo Sim and the ROS packages used by the launch file: `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`, `robot_state_publisher`, `robot_localization`, `rviz2`, `xacro` and [aruco_opencv](https://github.com/fictionlab/ros_aruco_opencv).

## Build and launch

Clone this repository and `aruco_opencv` into the `src/` directory of a ROS 2 workspace. From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}"
ros2 launch erl2 erl2_simulation.launch.py
```

The environment variable is initialised because the launch file appends to it directly. Make the supplied ArUco model discoverable by Gazebo, using the model folder and resource-path instructions from the [companion mission repository](https://github.com/MattiaTinfena/exp_rob_ass2).

## Configuration

The launch file accepts `world`, `model`, `x`, `y`, `yaw`, `rviz`, `rviz_config` and `use_sim_time` arguments. Configure the ArUco tracker to use the expected camera topic and marker dictionary.

For the complete Nav2/PlanSys2 mission, follow the build and launch instructions in `exp_rob_ass2`. Dependencies used by the launch file are not fully listed in the current package manifest.

## Authors

Elisa Martinenghi, Mattia Tinfena and Roberto Bertelli.
