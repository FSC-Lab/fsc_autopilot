# FSC Autopilot

The FSC Lab's autopilot package for controlling multirotor in OFFBOARD mode via mavros.

Compared to [previous controller packages](https://github.com/jaeyoung-lim/mavros_controllers), our controller is:

- **Robust**: Our multirotor controller ensures *safety* and *reasonable* performance in spite of

  1. Motor thrust mismatch
  2. Vehicle weight mismatch
  3. Strong winds and aerodynamic effects
  
  thanks to a UDE (uncertainty and disturbance estimator)-based design

- **Cross-Platform**: Our framework works on drones running either Ardupilot or PX4

- **Tested**: Our controller has been deployed on both a lightweight DJI F450 quadrotor (1.7kg) AND a heavier T15 octorotor

## Dependencies

We depend on `Eigen>=3.7` and a few standard ROS packages

``` bash
sudo apt-get install libeigen3-dev \
  ros-$ROS1_DISTRO_name-mavros \
  ros-$ROS1_DISTRO_name-mavros-msgs \
  ros-$ROS1_DISTRO_name-geometry-msgs \
  ros-$ROS1_DISTRO_name-nav-msgs \
  ros-$ROS1_DISTRO_name-sensor-msgs \
  ros-$ROS1_DISTRO_NAME-tf2-eigen
```

where ``$ROS1_DISTRO_NAME`` is to be replaced with your ROS distribution name, e.g. ``noetic``

## Building

Simply run

``` bash
catkin build
```

## Simulation

### PX4 SITL

PX4's [gazebo classic with ROS1](https://docs.px4.io/main/en/simulation/ros_interface.html) simulation is by far the easiest for getting started.
Once you have set up the simulation environment per PX4 documentation, you can launch the simulation (alongside mavros) with

``` bash
roslaunch px4 mavros_posix_sitl.launch
```

Once the simulator is up and running, you can start our controller with

``` bash
roslaunch fsc_autopilot_ros autopilot.launch builtin_param_file:=gazebo_iris_params.json
```

> :mag_right: You can inspect these default parameters in ``params/gazebo_iris_params.json``.

---

#### Alternative Positioning

Gazebo lets you directly access ground truth pose and velocity of the quadrotor via the [P3D plugin](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins#P3D(3DPositionInterfaceforGroundTruth)).
To let our autopilot receive ground truth feedback (instead of from `/mavros/local_position/odom` by default), you should remap the ground truth topic to `/fsc_autopilot/position_controller/feedback`.

> :bulb: If you pass the `feedback_topic:=<your topic name>` argument to `autopilot.launch`, the launch file handles the remapping for you

---

Engage the controller by sending setpoints to the controller on the topic ``/position_controller/target``
In the simplest case, you can send plain position/yaw setpoints on the command line

``` bash
rostopic pub /fsc_autopilot/position_controller/reference fsc_autopilot_msgs/PositionControllerReference "
position: {x: 0.0, y: 0.0, z: 0.0}
yaw: 0.0
"
```

> :mag_right: You can inspect the message definition at ``fsc_autopilot_ros/fsc_autopilot_msgs/msg/PositionControllerReference.msg`` in order to write more involved (e.g. path planning) programs sending out these setpoints

### Ardupilot SITL

We use the baseline [Ardupilot SITL](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html) without Gazebo.

> :question: Ardupilot currently supports Gazebo Garden onwards, which **cannot** be installed alongside ROS1 on Ubuntu 20.04; Its support for Gazebo classic depends on a [community plugin](https://github.com/khancyr/ardupilot_gazebo).

Once you have set up the simulation environment per Ardupilot documentation, you can launch the simulation with

``` bash
sim_vehicle.py -v copter -w  --mavproxy-args="--streamrate=100" --map --console
```

> :warning: The `--mavproxy-args="--streamrate=100"` argument is **absolutely necessary** to ensure a healthy IMU publishing rate

Then launch mavros and let it connect with the simulation with

``` bash
roslaunch mavros apm.launch fcu_url:="udp://127.0.0.1:14550@"
```

Once the simulator and mavros are up and running, you can start our controller with

``` bash
roslaunch fsc_autopilot_ros autopilot.launch builtin_param_file:=apm_sitl_params.json
```
