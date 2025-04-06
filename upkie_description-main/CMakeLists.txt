# CMakeLists.txt -- Upkie description package
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

cmake_minimum_required(VERSION 3.1)
project(upkie_description VERSION 1.1.1)

find_package(catkin)

catkin_package()

install(DIRECTORY meshes urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
