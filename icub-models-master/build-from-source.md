# Build icub-models from source

To install the icub-models, execute the following commands from within the ``icub-models`` folder:


```sh
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<prefix> -DCMAKE_BUILD_TYPE=Release -DICUB_MODELS_USES_PYTHON:BOOL=ON ..
cmake --build . --config Release --target install
```

Once the models are installed into a given prefix, edit the env variables as follows to ensure that they can be found.

```sh
export YARP_DATA_DIRS=${YARP_DATA_DIRS}:<prefix>/share/iCub
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:<prefix>/share
export AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH}:<prefix>
```