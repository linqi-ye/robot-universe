# Changelog

This file documents notable changes to this project done before June 2024. For changes after that date, plase refers to the release notes of each release at https://github.com/robotology/icub-models/releases .

## [Unreleased]

# [2.6.0] - 2024-06-03

### Added
* icub2_5: Add alljoints-inertials_wrapper/remapper (https://github.com/robotology/icub-models-generator/pull/264)
* Add left/right_arm FT IMU in iCubGazeboV2_* models (https://github.com/robotology/icub-models-generator/pull/259)

### Fixed 
* iCubV2_*: Fix measure of arm ft sensors (https://github.com/robotology/icub-models-generator/pull/265).

### Removed
* Remove iCubParis01 and iCubParis02 (https://github.com/robotology/icub-models-generator/pull/273).

# [2.5.0] - 2024-02-06

### Added

* Added `iCubWaterloo01` model (https://github.com/robotology/icub-models-generator/pull/257).
* Added `iCubGenova11` model (https://github.com/robotology/icub-models-generator/pull/252).

### Fixed

* Fixed wrong orientation for head IMU frame in iCub 2.6 and 2.7 models (https://github.com/robotology/icub-models/issues/231, https://github.com/robotology/icub-models-generator/pull/258).


# [2.4.1] - 2023-10-30

### Fixed
* Fixed version number in CMakeLists.txt and package.xml (https://github.com/robotology/icub-models/pull/222).

# [2.4.0] - 2023-10-30

### Changed
* Enable publishing of robot joints in ROS 2 via `controlBoard_nws_ros2` for `iCubGazeboV2_5_visuomanip` (https://github.com/robotology/icub-models/pull/211).
* CMake: Permit to explictly specify Python installation directory by setting the `ICUB_MODELS_PYTHON_INSTALL_DIR` CMake variable (https://github.com/robotology/icub-models/pull/218).
* Delete unused files (https://github.com/robotology/icub-models-generator/pull/236).
* Remove iCubGenova04_plus model  and rename iCubGazeboV2_5_plus to iCubGazeboV2_5_KIT_007 (https://github.com/robotology/icub-models-generator/pull/250).

### Fixed
* Fixed wrong simulated finger hall effect sensors port names prefix for `iCubGazeboV2_5_visuomanip` (https://github.com/robotology/icub-models/pull/215).
* Fixed compatibility with Python 3.12 and various issues with recent Python versions (https://github.com/robotology/icub-models/pull/217, https://github.com/robotology/icub-models/pull/218, https://github.com/robotology/icub-models/pull/219).
* Change `*_hand_fingers` to `*_hand_finger` in `iCubGazeboV2_5_visuomanip` for consistency with the real robot (https://github.com/robotology/icub-models/pull/213).
* Fix location of collision element in iCubGazeboV3 model (https://github.com/robotology/icub-models-generator/pull/251).


# [2.3.0] - 2023-09-11

### Fixed

* Fix location of `*_foot_*` links in iCub3 models (https://github.com/robotology/icub-models-generator/pull/243).
* Fixed publication of xsens imu on iCubGazeboV2_7 (https://github.com/robotology/icub-models-generator/pull/246).

# [2.2.2] - 2023-07-10

### Fixed

* Fix version number in CMake and package.xml files.

# [2.2.1] - 2023-07-10

### Fixed

* Fix Gazebo Classic simulation of iCub 2.* and 3 models, that stopped working since the 2.1.0 release (https://github.com/robotology/icub-models-generator/pull/240). Note that this fix highlighted a bug in icub-models that was introduced in 1.26.0, namely that the names of the FT sensors in the iCub models is not consistent with the real robot. This issue will be fixed in a future release, see https://github.com/robotology/icub-models-generator/issues/242 for more details.

### Changed

* Tune position PID gains for torso for iCub 2.* (https://github.com/robotology/icub-models-generator/pull/241).

# [2.2.0] - 2023-06-27

**Due to an error, this release did not contained improvements.**

# [2.1.0] - 2023-06-05

### Changed

* For consistency with the robot, now iCub simulated models publish their Force-Torque Sensors measurements via the [`multipleanalogsensorsserver`](https://www.yarp.it/latest//classMultipleAnalogSensorsServer.html) YARP device (https://github.com/robotology/icub-models/issues/198, https://github.com/robotology/icub-models-generator/pull/239, https://github.com/robotology/robots-configuration/pull/517).  The `name`  parameter passed to this device is: `/<robotPortPrefix>/<partName>/FT`.  This means that for each part there will be a `multipleanalogsensorsserver` device that will open the following YARP ports (these YARP ports are not meant to be accessed directly, but should be accessed instead via the [`multipleanalogsensorsclient`](https://www.yarp.it/git-master/classMultipleAnalogSensorsClient.html) device):
  * `/<robotPortPrefix>/<partName>/FT/measures:o` : that publishes sensors information for the part, using the structure defined in https://github.com/robotology/yarp/blob/master/src/devices/multipleAnalogSensorsMsgs/multipleAnalogSensorsSerializations.thrift
  * `/<robotPortPrefix>/<partName>/FT/rpc:o` : that expose several information related to the part via a YARP RPC port

# [2.0.1] - 2023-03-31

### Fixed

* Harmonize iCubGazeboV2_5_visuomanip RGB cameras port names (https://github.com/robotology/icub-models/pull/196).

# [2.0.0] - 2023-03-08

### Fixed

* Make models compatible with the YARP 3.8 (https://github.com/robotology/icub-models/issues/171, https://github.com/robotology/icub-models/pull/188, https://github.com/robotology/icub-models-generator/pull/231).
* Fix incompatibility between RViz2 and iCubGazeboV3 model ( https://github.com/robotology/icub-models-generator/issues/229 ).
* Uniform IMUs sensor names (https://github.com/robotology/icub-models-generator/pull/235).
* Add partial support to expose sensor data of iCubGazeboV3 on ROS2 (https://github.com/robotology/icub-models-generator/pull/228).


# [1.26.0] - 2022-12-09

### Changed
* Export frames in urdf for the sensors (imu and Ft sensors) (https://github.com/robotology/icub-models-generator/pull/227).
* Migrate iCub3 models to use gazebo_yarp_robotinterface plugin (https://github.com/robotology/icub-models-generator/pull/222).

### Fixed
* Fix finding the `iCub` package in ROS2 (https://github.com/robotology/icub-models/pull/178).

## [1.25.0] - 2022-08-23

### Changed

* Update iCub3 model with new weight, add FT on left leg, increase shoulder pitch limits (https://github.com/robotology/icub-models-generator/pull/223).
* iCub3: Decrease limit of hip roll after installation of hard stop on robot (https://github.com/robotology/icub-models-generator/pull/224).

## [1.24.1] - 2022-06-08

### Fixed

* Fix r_wrist_yaw joint direction in several iCub 2.* models (https://github.com/robotology/icub-models/issues/156, https://github.com/robotology/icub-models-generator/pull/220).

## [1.24.0] - 2022-05-31

### Fixed
* New tuned pidParams for icub3 gazebo model for walking simulations (https://github.com/robotology/icub-models-generator/pull/219).

## [1.23.4] - 2022-05-06

### Fixed
* Make the C++ helper library more robust to system that automatically make the library relocatable such as conda (https://github.com/robotology/icub-models/pull/154).

## [1.23.3] - 2022-04-08

### Fixed
* In Python helper library, fix the `get_models_path()` when the `icub_models` package is installed with the robotology-superbuild (https://github.com/robotology/icub-models/pull/149, https://github.com/robotology/icub-models/pull/150).
* Fix the `PYTHON_ABSOLUTE_INSTDIR` when a relative `PYTHON_INSTDIR` is detected (https://github.com/robotology/icub-models/pull/150)

## [1.23.2] - 2022-03-31

### Fixed

* Fix CMake configuration if Python library installation is enabled (https://github.com/robotology/icub-models/pull/146).

## [1.23.1] - 2022-03-30

### Changed

* The python module `icub_models` is now implemented in pure python. This removes the pybind11 dependency (https://github.com/robotology/icub-models/pull/143)

## [1.23.0] - 2022-02-28

### Added

* Implement a machinery to easily access model locations on C++ and Python (https://github.com/robotology/icub-models/pull/130)

### Changed

* All Gazebo models installed by icub-models are in SDF format version 1.7. This means that Gazebo >= 11 is required to load them (https://github.com/robotology/icub-models/pull/130).
* iCub3 models (`iCubGenova09` and `iCubGazeboV3`) now have an FT sensor installed in the right upper leg (https://github.com/robotology/icub-models-generator/pull/218)
* The URDF colors of models is not random anymore (https://github.com/robotology/icub-models-generator/pull/217).

### Fixed
* Fix missing velocityControlImplementationType parameter in left_wrist_mk2 (https://github.com/robotology/icub-models-generator/pull/213).
* Fix r_wrist_yaw of icub3 generated as continous joint (https://github.com/robotology/icub-models-generator/pull/214)

## [1.22.1] - 2021-12-04

### Fixed
* Fix bug in iCub3 models that made `r_wrist_yaw` a continuous joint (https://github.com/robotology/icub-models-generator/pull/214).
* Fix compatibility of `left_wrist_mk2` with gazebo-yarp-plugins 4 (https://github.com/robotology/icub-models-generator/pull/213).

## [1.22.0] - 2021-11-12

### Changed
* Update iCub3 with feet covers (https://github.com/robotology/icub-models-generator/pull/210).
* Increase metacarpus joint limit of MK3 left hand's thumb (https://github.com/robotology/icub-models-generator/pull/211).

### Fixed
* Fixed compatibility of all iCub3 models with gazebo-yarp-plugins 4 (https://github.com/robotology/icub-models-generator/pull/21).

## [1.21.1] - 2021-09-03

### Fixed
* Fixed document on the use of `AMENT_PREFIX_PATH` environment variable (https://github.com/robotology/icub-models/pull/101).
* Fixed wrong default collision on the arms that slowed down Gazebo Classic simulation for iCub3 models (https://github.com/robotology/icub-models/issues/100, https://github.com/robotology/icub-models-generator/pull/208).

## [1.21.0] - 2021-08-27

### Added
* The `hand_mk3` and `wrist_mk2` containing the new version of iCub's hand and wrist, codenamed iCub hand Mk3 and iCub wrist Mk2, have been added to the repo. As this models are still in development, they could change in the future (https://github.com/robotology/icub-models-generator/pull/203, https://github.com/robotology/icub-models/pull/95).

### Changed
* The iCub3 models (`iCubGenova09` and `iCubGazeboV3`) have been updated with the new arm covers (https://github.com/robotology/icub-models-generator/pull/202).

### Fixed 
* The masses of iCub3 models (`iCubGenova09` and `iCubGazeboV3`) have been fixed to be more similar to the one of the real robot (https://github.com/robotology/icub-models-generator/pull/207).


## [1.20.0] - 2021-05-30

### Changed
* The gains and damping parameters of `iCubGazeboV3` have been changed (https://github.com/robotology/icub-models-generator/pull/199).

## [1.19.0] - 2021-02-24

### Added 
- Added documentation to use the models in ROS2 (https://github.com/robotology/icub-models/pull/82).
- Added model for the iCubErzelli02 robot (https://github.com/robotology/icub-models-generator/pull/195).

### Fixed 
- Fixed the license tag in the package.xml for use the models in ROS/ROS2 (https://github.com/robotology/icub-models/pull/78).
- Fixed several issues and problems with the iCubGenova09/iCubGazeboV3 model (https://github.com/robotology/icub-models-generator/pull/194, https://github.com/robotology/icub-models-generator/pull/193, https://github.com/robotology/icub-models-generator/pull/189, https://github.com/robotology/icub-models-generator/pull/187). 

## [1.18.1] - 2020-11-30

### Fixed 
- Fix the direction of some axis for the iCub3 models (https://github.com/robotology/icub-models-generator/pull/184). 

## [1.18.0] - 2020-11-30

### Added
- Added first URDF model for the iCub3, namely the `iCubGazeboV3`  with inertia modified to run in Gazebo Classic, and iCubGenova09 for control of the real robot. 
  As the development of this model is an ongoing activity, interested users are suggested to keep in sync with the updates that will happen in the `devel` branch. 

### Changed
- The `base_link` massless link is now a child of the `root_link` link instead of being a parent. This has the effect that now when the URDF model of iCub is imported
  in Gazebo, after fixed joint lumping the link corresponding to the robot waist will be `root_link`, instead of `base_link`. This means that when the iCub model is included 
  in a nested model, the joint to connect it to the rest of the model should refer to `iCub::root_link` instead of `iCub::base_link`
  (https://github.com/robotology/icub-models-generator/pull/142, https://github.com/robotology/icub-models-generator/issues/140#issuecomment-700653608).
