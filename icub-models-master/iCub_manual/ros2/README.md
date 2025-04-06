:warning: The ROS2 support is in beta :warning:

This directory contains the files related for using `iCubGazeboV*` with ROS2. The contents of this directory are installed inside `iCub`.
ros2 package. The location of `iCub` ros pacakge is discovered by running `roscd iCub`.
The `launch` directory contains `launch.py` file `robot_state_publisher.launch.py`. The robot model
that will be used depends `YARP_ROBOT_NAME` set in `.bashrc`.

Right now are published on ROS2 topics:

- FT sensors measurements
- IMU sensors measurements
- Joint states

The steps to run rviz visualization correctly are:

- Edit `iCubGazeboV*/model.urdf` putting  `icub_ros2.xml` `robotinterface` xml file used at startup.
- Ensure that the robot is started correctly with ros configuration files for motor control boards.
- On starting the robot, one should see `/<prefix>/joint_states` ros topic by running `ros2 topic list`.
  Also, ensure that the `/<prefix>/joint_states` streams all the robot joint angles by running
  `ros2 topic echo /<prefix>/joint_states`.
- Start `transform server` by running `yarprobotinterface --config transform-server.xml`.
  The transform server is a central location for transforms `tfs` and it streams the transforms
  to `/tf` ros topic.
- Launch [robot_state_publisher](http://wiki.ros.org/robot_state_publisher)

```
ros2 launch robot_state_publisher.launch.py
```
 - Ensure that the transforms are available correctly by running `ros2 topic echo /tf`.

- At this point you can launch `rviz2` and visualize iCub. There is an issue for visualizing the meshes(https://github.com/robotology/icub-models-generator/issues/229) but the tfs or IMU/FT sensor measurements can be visualized without problems.

The following two parameters are important to ensure correct visualization:

- Under `Global Options`, `Fixed Frame` field is the frame name with respect to
  which all the other frames transforms are give. The default value set inside the `map`, you have to set it to `root_link`.

![immagine](https://user-images.githubusercontent.com/19152494/206218846-faf4375f-f1d2-4e24-a05d-234ca2e848a5.png)

The contents of this directory are tested on:

##### Linux

```
Ubuntu 22.04 LTS
Rosdistro: Humble
```
