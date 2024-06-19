# arob_mpc

`arob_mpc` is a ROS package designed for autonomous drone racing using Model Predictive Control (MPC). The package generates and tracks trajectories through a series of gates, optimizing for smooth and efficient flight.

## Features

- Trajectory generation with position and velocity constraints
- Model Predictive Control (MPC) for trajectory tracking
- Visualization of trajectories and gates in RViz
- Configurable through ROS parameters and launch files

## Required packages

## Installation

1. **Clone the repository:**
    ```sh
    cd ~/catkin_ws/src
    git clone https://github.com/albertozafra7/AR.git
    ```
    **You can also clone the different dependencies:**
    ```sh
    cd ~/catkin_ws/src
    git clone https://github.com/ros-geographic-info/unique_identifier.git
    git clone https://github.com/ros-geographic-info/geographic_info.git
    git clone https://github.com/RAFALAMAO/hector_quadrotor_noetic.git
    mkdir traj_gen
    cd traj_gen
    git clone https://github.com/ethz-asl/mav_trajectory_generation.git
    git clone https://github.com/catkin/catkin_simple.git
    git clone https://github.com/ethz-asl/eigen_catkin.git
    git clone https://github.com/ethz-asl/eigen_checks.git
    git clone https://github.com/ethz-asl/glog_catkin.git
    git clone https://github.com/ethz-asl/mav_comm.git
    git clone https://github.com/ethz-asl/nlopt.git
    cd ~/catkin_ws
    catkin build
    ```

2. **Install Casadi:**

    ```sh
    sudo apt update
    sudo apt install build-essential
    sudo apt install coinor-libipopt-dev
    pip3 install casadi
    sudo apt install manpages-dev software-properties-common
    sudo add-apt-repository ppa:ubuntu-toolchain-r/test
    sudo apt update && sudo apt install gcc-11 g++-11
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 110
    sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 110
    sudo apt install gfortran liblapack-dev pkg-config --install-recommends
    sudo apt install swig
    cd
    git clone https://github.com/casadi/casadi.git -b master casadi
    cd casadi
    mkdir build
    cd build
    cmake -DWITH_PYTHON=ON -DWITH_OPENMP=ON -DWITH_THREAD=ON ..
    make
    sudo make install
    ```

3. **Build the package:**
    ```sh
    cd ~/catkin_ws
    catkin buld
    ```

4. **Source your workspace:**
    ```sh
    source devel/setup.bash
    ```

## Usage

### Running the Package

To run the package it would be necessary to first launch the roscore system and execute the rviz node with the "arob6.rviz" configuration found in "/arob_mpc/src"

```sh
roscore
rosrun rviz rviz
```

Load the "/arob_mpc/src/arob6.rviz" configuration file

Run the package locating the quadrotor within the easy scenario with the provided launch file:
```sh
roslaunch arob_mpc mpc.launch
```

If you want to run the package with the hard scenario or a specific gates configuration file, you can use the provided launch file:

```sh


roslaunch arob_mpc mpc_hard.launch
```
This command launches the drone_race node with the gates_hard.txt configuration.

### Configuring Parameters

You can configure the different simulation parameters by modifying them within the "params.yaml" file

```yaml
drone_params:
  lookahead: 50
  sampling_interval: 0.1 # our dt

  max_vel: 2  # max vel of the drone
  max_accel: 2 #8.65 # max accel of the drone

  # Weights of the cost function
  pos_error_w: 0.75 

  # Saving file path
  saving_file: "/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/arob_mpc/src/mpc_positions.txt"
```

## Nodes
### drone_race

This node handles the trajectory generation and tracking for the drone.

#### Subscribed Topics:

- **/uav_pose** (geometry_msgs/Pose): Receives the current pose of the UAV.

#### Published Topics:

- **/cmd_vel** (geometry_msgs/Twist): Publishes velocity commands to the drone.
- **/trajectory_markers** (visualization_msgs/MarkerArray): Publishes trajectory visualization markers.

#### Parameters:
- **~max_velocity** (double, default: 2.0): Maximum allowed velocity for the drone.
- **~max_acceleration** (double, default: 2.0): Maximum allowed acceleration for the drone.
- **~control_horizon** (int, default: 10): Number of steps in the control horizon for MPC.
- **~sampling_time** (double, default: 0.1): Sampling time interval for the MPC loop.
- **~gates_file** (string, default: "gates.txt"): Path to the gates configuration file.

Example Usage

You can specify a different gates file by modifying the mpc_hard.launch file or by passing an argument when launching:

```sh

roslaunch arob_mpc mpc_hard.launch gates_file:=gates_easy.txt
```

### mpc

The `mpc` node is responsible for executing Model Predictive Control to follow the generated trajectory.

#### Subscribed Topics

- **/desired_trajectory** (mav_msgs/DroneStateArray): Receives the desired trajectory states to follow.
- **/current_pose** (geometry_msgs/PoseStamped): Receives the current pose of the drone.
- **/current_velocity** (geometry_msgs/TwistStamped): Receives the current velocity of the drone.

#### Published Topics

- **/cmd_vel** (geometry_msgs/Twist): Publishes the velocity commands to control the drone.
- **/mpc_debug** (visualization_msgs/MarkerArray): Publishes debug markers for visualization in RViz.

#### Parameters

- **~control_horizon** (int, default: 10): Number of steps in the control horizon for MPC.
- **~sampling_time** (double, default: 0.1): Sampling time interval for the MPC loop.
- **~weights_position** (double, default: 1.0): Weight for the position error in the cost function.
- **~weights_velocity** (double, default: 0.1): Weight for the velocity error in the cost function.
- **~max_velocity** (double, default: 5.0): Maximum allowed velocity for the drone.
- **~max_acceleration** (double, default: 2.0): Maximum allowed acceleration for the drone.