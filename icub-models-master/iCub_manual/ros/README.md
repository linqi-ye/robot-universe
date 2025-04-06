This directory contains the files related to [Rviz](http://wiki.ros.org/rviz)
visualization of icub. The contents of this directory are installed inside `iCub`
ros package. The location of `iCub` ros pacakge is discovered by running `roscd iCub`.
The `launch` directory contains `roslaunch` file `iCubRviz.launch`. The robot model
that will be used for rviz visualization depends `YARP_ROBOT_NAME` set in `.bashrc`.
The default rviz configuration file is `iCubRviz.rviz` located inside `rviz` directory.
The steps to run rviz visualization correctly are:

- Ensure that the robot is started correctly with ros configuration files for motor control boards.
  On the real robot, the [icub_ros_wrapper.xml](https://github.com/robotology/robots-configuration/blob/devel/iCubGenova04/wrappers/motorControl/icub_ros_wrapper.xml) configuration file must be included inside the startup
  `xml` [configuration file](https://github.com/robotology/robots-configuration/blob/devel/iCubGenova04/icub_wbd.xml#L107).
- On starting the robot, one should see `/<prefix>/joint_states` ros topic by running `rostopic list`.
  Also, ensure that the `/<prefix>/joint_states` streams all the robot joint angles by running
  `rostopic echo /<prefix>/joint_states`. The name of topic is set through [ROS_topicName](https://github.com/robotology/robots-configuration/blob/devel/iCubGenova04/wrappers/motorControl/icub_ros_wrapper.xml#L38)
  parameter.
- Start `transform server` by running `yarprobotinterface --config transform-server.xml`.
  The transform server is a central location for transforms `tfs` and it streams the transforms
  to `/tf` ros topic.
- Launch [yarprobotstatepublisher](https://github.com/robotology/idyntree/tree/master/src/tools/yarprobotstatepublisher) that takes a tf-prefix, model, base frame name, joint states topic name as input and
gives all the model transforms to the transform server. An example of running `yarprobotstatepublisher` is

```
yarprobotstatepublisher --name-prefix icub --tf-prefix /icub/ --model model.urdf --base-frame root_link --jointstates-topic "/icub04/joint_states"
```
 - Ensure that the transforms are available correctly by running `rostopic echo /tf`.
   The transforms of the links are streamed with the prefix give to `--tf-prefix` of `yarprobotstatepublisher`.

- Rviz visualization of icub is launched by running

```
  roslaunch iCub iCubRviz.launch
```

The following two parameters are important to ensure correct visualization:

- Under `Global Options`, `Fixed Frame` field is the frame name with respect to
  which all the other frames transforms are give. The default value set inside the `iCubRviz.rviz` configuration value is `icub/root_link`.

  ![Selection_001](https://user-images.githubusercontent.com/6505998/62306298-e064e400-b481-11e9-9b5e-c665b5aaa5ff.png)


- The next parameter is under `Robot Model`, `TF Prefix`. The default value set inside the `iCubRviz.rviz` configuration value is `icub`.
  The value of the parameter should match the `--tf-prefix` of `yarprobotstatepublisher` for rviz to get the correct transforms to visualize the model.

  ![Screenshot from 2019-08-01 11-37-26](https://user-images.githubusercontent.com/6505998/62283354-ab409d80-b451-11e9-9811-bd9158548e3e.png)

The contents of this directory are tested on:

##### Linux

```
Ubuntu 18.04 LTS
Rosdistro: Melodic
Rosversion: 1.14.3
```

##### MacOS

```
MacOs High Sierra 10.13.6
Rosdistro: Lunar
Rosversion: 1.13.7
```
