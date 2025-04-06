cmake_minimum_required(VERSION 3.5)
project(jvrc_description VERSION 1.0.4)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes rsdf urdf
    DESTINATION share/${PROJECT_NAME}
)

ament_package()