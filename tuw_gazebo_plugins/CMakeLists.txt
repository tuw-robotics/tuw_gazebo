cmake_minimum_required(VERSION 2.8.3)
project(tuw_gazebo_plugins)

add_compile_options(-std=c++14  -fext-numeric-literals -lmlg) ## for c++ 2014

SET(CMAKE_BUILD_TYPE Release)

MESSAGE( STATUS "CMAKE_BUILD_TYPE:              " ${CMAKE_BUILD_TYPE} )
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -fno-strength-reduce -pipe ")# -Og
set(CMAKE_CSS_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -pipe -Ofast")

if(CMAKE_BUILD_TYPE MATCHES Release)
MESSAGE( STATUS "CMAKE_CSS_FLAGS_RELEASE:       " ${CMAKE_CSS_FLAGS_RELEASE} )
endif(CMAKE_BUILD_TYPE MATCHES Release)

if(CMAKE_BUILD_TYPE MATCHES Debug)
MESSAGE( STATUS "CMAKE_CXX_FLAGS_DEBUG:         " ${CMAKE_CXX_FLAGS_DEBUG} )
endif(CMAKE_BUILD_TYPE MATCHES Debug)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nodelet
  urdf
  tf
  tf2_ros
  dynamic_reconfigure
  std_msgs
  nav_msgs
  gazebo_msgs
  tuw_nav_msgs
  tuw_vehicle_msgs
)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(XML libxml-2.0)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  pkg_check_modules(OGRE-Paging OGRE-Paging)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()



# Depend on system install of Gazebo and SDFormat
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

execute_process(COMMAND
  pkg-config --variable=plugindir OGRE
  OUTPUT_VARIABLE OGRE_PLUGIN_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

catkin_python_setup()

generate_dynamic_reconfigure_options( cfg/GazeboRosBridgeModelPlugin.cfg
                                      cfg/GazeboRosHumanReceiverPlugin.cfg
                                      cfg/ConeDetectionSim.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    tuw_gazebo_ros_utils
    tuw_gazebo_ros_diff_drive
    tuw_gazebo_ros_laser
    tuw_gazebo_ros_gpu_laser
    tuw_gazebo_ros_imu9dof
    tuw_gazebo_ros_bridge_model
    tuw_gazebo_ros_encoder
    tuw_gazebo_ros_rwd
    tuw_gazebo_tire_model
    map_common
    cone_detection_sim
    tuw_gazebo_ros_map
    tuw_gazebo_ros_odom
  CATKIN_DEPENDS
  DEPENDS
    gazebo
    SDF
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

add_library(tuw_gazebo_ros_bridge_model src/gazebo_ros_bridge_model.cpp)
target_link_libraries(tuw_gazebo_ros_bridge_model tuw_gazebo_ros_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(tuw_gazebo_ros_human_receiver src/gazebo_ros_human_receiver.cpp)
target_link_libraries(tuw_gazebo_ros_human_receiver ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(tuw_gazebo_tire_model src/tire_model.cpp)
target_link_libraries(tuw_gazebo_tire_model tuw_gazebo_ros_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(tuw_gazebo_ros_rwd src/gazebo_ros_rwd.cpp src/HV_battery.cpp src/aero_forces.cpp)
target_link_libraries(tuw_gazebo_ros_rwd tuw_gazebo_ros_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(tuw_gazebo_ros_encoder src/gazebo_ros_encoder.cpp)
target_link_libraries(tuw_gazebo_ros_encoder tuw_gazebo_ros_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(map_common src/map_common.cpp)
target_link_libraries(map_common tuw_gazebo_ros_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(cone_detection_sim src/cone_detection_sim.cpp)
target_link_libraries(cone_detection_sim tuw_gazebo_ros_utils map_common ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(tuw_gazebo_ros_map src/gazebo_ros_map.cpp)
target_link_libraries(tuw_gazebo_ros_map tuw_gazebo_ros_utils map_common ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(tuw_gazebo_ros_odom src/gazebo_ros_odom.cpp)
target_link_libraries(tuw_gazebo_ros_odom tuw_gazebo_ros_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
  tuw_gazebo_ros_utils
  tuw_gazebo_ros_diff_drive
  tuw_gazebo_ros_laser
  tuw_gazebo_ros_gpu_laser
  tuw_gazebo_ros_imu9dof
  tuw_gazebo_ros_bridge_model
  tuw_gazebo_tire_model
  tuw_gazebo_ros_rwd
  tuw_gazebo_ros_encoder
  map_common
  cone_detection_sim
  tuw_gazebo_ros_map
  tuw_gazebo_ros_odom
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
