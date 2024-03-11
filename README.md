# Tracking Controller

Dirt simple quadrotor controller that is initially port of [mavros_controllers](https://github.com/jaeyoung-lim/mavros_controllers) to [jax](https://github.com/google/jax.git)-accelerated, functional-styled python, then backported to C++.

## Dependencies

``` bash
sudo apt-get install libeigen3-dev libfmt-dev ros-$ROS1_DISTRO_NAME-tf2-eigen
```

where `$ROS1_DISTRO_NAME` is to be replaced with your ROS distribution name, e.g. `noetic`

## Building

``` bash
catkin build
```

## Running

This controller is tested to be able to control the stock 3DR iris during PX4 SITL simulation.
Launch this simulation via

``` bash
roslaunch px4 mavros_posix_sitl.launch
```

Once the simulator is up and running, run

``` bash
roslaunch tracking_control tracking_control.launch
```

Send position setpoints to the controller on the command line

``` bash
rostopic pub /tracking_controller/target trajectory_msgs/JointTrajectoryPoint "positions: [10, 100, 10]
velocities: [0]
accelerations: [0]
effort: [0]
time_from_start: {secs: 0, nsecs: 0}"
```

Feel free to leave the fields: `velocities` and `accelerations` uninitialized.
The controller will complain but will silently set those fields to zero.
