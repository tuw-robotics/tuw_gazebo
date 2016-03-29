cmake_minimum_required(VERSION 2.8.3)
project(tuw_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS 
  std_msgs
  roscpp 
  rospy 
  nodelet 
  urdf 
  tf 
  tf2_ros 
  dynamic_reconfigure 
  gazebo_msgs 
)


# Depend on system install of Gazebo and SDFormat
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

generate_dynamic_reconfigure_options(
# cfg/*.cfg
)

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS}
  ${OGRE-Paging_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${OGRE_LIBRARY_DIRS}
  ${OGRE-Terrain_LIBRARY_DIRS}
  ${OGRE-Paging_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  tuw_gazebo_ros_utils 
  tuw_gazebo_ros_diff_drive
  tuw_gazebo_ros_laser
  tuw_gazebo_ros_gpu_laser
  tuw_gazebo_ros_imu9dof
  
  CATKIN_DEPENDS 
  std_msgs
  roscpp 
  rospy 
  nodelet 
  urdf 
  tf 
  tf2_ros 
  dynamic_reconfigure 
  gazebo_msgs 
  DEPENDS 
    gazebo 
    SDF
)

add_dependencies(${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

## Utils
add_library(tuw_gazebo_ros_utils src/gazebo_ros_utils.cpp src/noise_sim_model.cpp)
target_link_libraries(tuw_gazebo_ros_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_definitions(-fPIC) # what is this for?

## Plugins
add_library(tuw_gazebo_ros_diff_drive src/gazebo_ros_diff_drive.cpp)
target_link_libraries(tuw_gazebo_ros_diff_drive tuw_gazebo_ros_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(tuw_gazebo_ros_laser src/gazebo_ros_laser.cpp)
target_link_libraries(tuw_gazebo_ros_laser tuw_gazebo_ros_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(tuw_gazebo_ros_gpu_laser src/gazebo_ros_gpu_laser.cpp)
target_link_libraries(tuw_gazebo_ros_gpu_laser tuw_gazebo_ros_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(tuw_gazebo_ros_imu9dof src/gazebo_ros_imu9dof.cpp)
target_link_libraries(tuw_gazebo_ros_imu9dof tuw_gazebo_ros_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
