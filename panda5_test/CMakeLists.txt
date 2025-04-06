cmake_minimum_required(VERSION 2.8.3)

project(panda5_test)

find_package(catkin REQUIRED)

catkin_package()

find_package(catkin REQUIRED COMPONENTS
joint_state_publisher
robot_state_publisher
roscpp
rospy
rviz
xacro
kdl_parser
tf
sensor_msgs
std_msgs
trac_ik_lib
)
find_package(Boost REQUIRED COMPONENTS date_time)
 
find_package(orocos_kdl REQUIRED)
 
find_package(roslaunch)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo
CATKIN_DEPENDS controller_manager gazebo_ros gazebo_ros_control joint_state_publisher robot_state_publisher roscpp rospy rviz xacro trac_ik_lib
DEPENDS Boost orocos_kdl
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)
add_executable(panda5_view src/panda5.cpp)
target_link_libraries(panda5_view
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
