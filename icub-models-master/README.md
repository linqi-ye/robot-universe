# icub-models

Repository containing URDF and SDF models of iCub humanoid robots.

The model contained in this repo are licensed under the [Creative Commons Attribution-ShareAlike 4.0 International License (CC BY-SA 4.0) ](https://creativecommons.org/licenses/by-sa/4.0/).

## Installation

### conda (recommended) 

You can easily install the `icub-models` and the C++ and Python helper libraries via [`conda-forge`](https://conda-forge.org) using the following command
~~~
conda install -c conda-forge icub-models
~~~

If you are not familiar with conda or conda-forge, you can read an introduction document in [conda-forge overview](https://github.com/robotology/robotology-superbuild/blob/master/doc/conda-forge.md#conda-forge-overview).

### robotology-superbuild (advanced)

If you use apt to install your dependencies or if you are install `icub-models` for use as part of [iCub humanoid robot software installation](https://icub-tech-iit.github.io/documentation/sw_installation/), you may want to install icub-models through the [robotology-superbuild](https://github.com/robotology/robotology-superbuild), an easy way to download, compile and install the robotology software on multiple operating systems, using the [CMake](https://www.cmake.org) build system and its extension [YCM](http://robotology.github.io/ycm). `icub-models` is always installed by `robotology-superbuild`.

### Build from source (advanced)

If you want to build icub-models directly from source of from the repo, you can check the documentation in [`build-from-source.md`](doc/build-from-source.md).

## Model Details

The models contained in `icub-models` are listed in the following table. Most models are identified by a name like `iCub<Something><Number>`, that we refer as `YARP_ROBOT_NAME`. Historically, on iCub robot setups, the `YARP_ROBOT_NAME` env variable identified the specific robot to use.

|  `YARP_ROBOT_NAME`   | `package:/`-URI     | Notes                           |
|:--------------------:|:------------:|:-------------------------------:|
| `iCubErzelli02`      | `package://iCub/robots/iCubErzelli02/model.urdf` | v2.5.5           |
| `iCubGazeboV2_5`     | `package://iCub/robots/iCubGazeboV2_5/model.urdf` | v2.5.5, joint damping, and inertias of some links increased in a non realistic way to run smoothly in Gazebo Classic (ODE). |
| `iCubGazeboV2_5_visuomanip` | `package://iCub/robots/iCubGazeboV2_5/model.urdf`  | v2.5.5 with hands and eyes, base_link fixed to the ground and legs disabled. |
| `iCubGazeboV2_5_KIT_007`| `package://iCub/robots/iCubGazeboV2_5_KIT_007/model.urdf` | v2.5 + [KIT_007](https://icub-tech-iit.github.io/documentation/upgrade_kits/ankle_for_stairs/support/) with backpack, joint damping, and inertias of some links increased in a non realistic way to run smoothly in Gazebo Classic (ODE). |
| `iCubGazeboV2_6`     | `package://iCub/robots/iCubGazeboV2_6/model.urdf` | v2.6 with  joint damping, and inertias of some links increased in a non realistic way to run smoothly in Gazebo Classic (ODE). |
| `iCubGazeboV2_7`     | `package://iCub/robots/iCubGazeboV2_7/model.urdf` | v2.7 with  joint damping, and inertias of some links increased in a non realistic way to run smoothly in Gazebo Classic (ODE). |
| `iCubGenova02`       | `package://iCub/robots/iCubGenova02/model.urdf` | v2.5.5 + [KIT_007](https://icub-tech-iit.github.io/documentation/upgrade_kits/ankle_for_stairs/support/) with backpack           |
| `iCubGenova03`       | `package://iCub/robots/iCubGenova03/model.urdf`           | v2 with legs v1 and feet v2.5   |
| `iCubGenova11`       | `package://iCub/robots/iCubGenova11/model.urdf`           | v2.7   |
| `iCubLisboa01`       | `package://iCub/robots/iCubLisboa01/model.urdf`           | v1 with head v2                 |
| `iCubNancy01`        | `package://iCub/robots/iCubNancy01/model.urdf`           | v2.5 with arms v1 and head v2   |
| `iCubWaterloo11`       | `package://iCub/robots/iCubWaterloo11/model.urdf`           | v2.7   |


## Usage

### Use the models with ROS

Models contained in `icub-models` can be used using the `package:/`-URI listed in previous table. This URI always follow the structure `package://iCub/robots/<name>/model.urdf`, where `<name>` is `iCubErzelli02`, `iCubGazeboV2_5` or a similar identifier.

### Use the models with Classic or Modern Gazebo

Note that only the models that are known to work fine with the default physics engine settings of Classic Gazebo (the one that start with `iCubGazebo`)
are installed. If you want to make available in Gazebo all the models, enable the `ICUB_MODELS_INSTALL_ALL_GAZEBO_MODELS` CMake option.

To include the model in a Classic or Modern Gazebo, use the following SDF:

~~~xml
<include>
    <pose>0 0 0.5 0 -0.1 3.14</pose>
    <uri>model://iCubGazeboV2_5</uri>
</include>
~~~

The `model:/` string is used to include the model, following the structure `model://<name>`, where `<name>` is `iCubGazeboV2_5`, `iCubGazeboV2_7` or a similar identifier. Note that you can also use the structure `model://iCub/robots/<name>`.

### Use the models from C++ helper library

In order to use these models in `c++` application you can exploit the `icub-models` library.
`icub-models` provides native `CMake` support which allows the library to be easily used in `CMake` projects.
**icub-models** exports a CMake target called `icub-models::icub-models` which can be imported using the `find_package` CMake command and used by calling `target_link_libraries` as in the following example:
```cmake
cmake_minimum_required(VERSION 3.5)
project(myproject)
find_package(icub-models REQUIRED)
add_executable(example example.cpp)
target_link_libraries(example icub-models::icub-models)
```

The `example.cpp` will contains
```cpp
#include <iCubModels/iCubModels.h>
#include <iostream>

int main()
{
    std::cout << "Models have been installed in: " << iCubModels::getModelsPath() << std::endl;

    std::cout << "Available robots: " << std::endl;
    for (const auto& robot : iCubModels::getRobotNames())
    {
        std::cout << " - " << robot << ": " <<  iCubModels::getModelFile(robot) << std::endl;
    }

    return EXIT_SUCCESS;
}
```

***Note: The ABI and the API of the C++ helper library has breaking changes only at major releases of the `icub-models` package.***

### Use the models from C++ using YARP

To find the model in C++ using YARP, you just need to make sure that `YARP_ROBOT_NAME` environment variable is set, and search for the `model.urdf` file:

~~~
std::string modelAbsolutePath =yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName("model.urdf");
~~~

### Use the models from Python icub-models library

The following script can be used to locate the models
```python
import icub_models

print(f"Models have been installed in: {icub_models.get_models_path()}")

print(f"Available robots: {icub_models.get_robot_names()}")

for robot_name in icub_models.get_robot_names():
    print(f"{robot_name}: {icub_models.get_model_file(robot_name)}")
```

### Use the models from Python using resolve-robotics-uri-py

First of all, make sure that you installed [`resolve-robotics-uri-py`](https://github.com/ami-iit/resolve-robotics-uri-py) python library. Then, you can find the `icub-models` models using `package:/` or `model:/` URIs, as you would use with Gazebo or ROS:

~~~
absolute_path = resolve_robotics_uri_py.resolve_robotics_uri("package://iCub/robots/iCubGazeboV2_7/model.urdf")
~~~

or

~~~
absolute_path = resolve_robotics_uri_py.resolve_robotics_uri("model://iCub/robots/iCubGazeboV2_7/model.urdf")
~~~
 

## FAQs

### Change the orientation of the root frame
The iCub robot `root frame` is defined as [`x-backward`][1], meaning that the x-axis points behind the robot. Nevertheless, in the robotics community, sometimes the root frame of a robot is defined as [`x-forward`][2]. As a consequence, to use the iCub models with software developed for the `x-forward` configuration (e.g. [IHMC-ORS][3]), might be necessary to quickly update the root frame orientation.  
For this purpose, locate the joint `<joint name="base_fixed_joint" type="fixed">` in the `URDF` model and perform the following substitution in the `origin` section:

```
-  <origin xyz="0 0 0" rpy="0 -0 0"/>
+  <origin xyz="0 0 0" rpy="0 -0 3.14159265358979323846"/>
```

[1]:http://wiki.icub.org/wiki/ICubForwardKinematics
[2]:http://www.ros.org/reps/rep-0103.html#axis-orientation
[3]:https://github.com/ihmcrobotics/ihmc-open-robotics-software
