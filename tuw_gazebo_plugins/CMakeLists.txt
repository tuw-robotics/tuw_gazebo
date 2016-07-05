cmake_minimum_required(VERSION 2.8.3)
project(tuw_gazebo_plugins)

set(IwsControllersSource False)
if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/gazebo_ros_iws_drive/)
  set(IwsControllersSource True)
  MESSAGE(STATUS "Found IWS Controller source files. Afferent libraries will be build!")
endif()


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
  tuw_gazebo_msgs 
)

IF(IwsControllersSource)
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
    tuw_gazebo_msgs
    tuw_control 
    tuw_geometry
    )
endif()

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

IF(IwsControllersSource)
  generate_dynamic_reconfigure_options( cfg/gazebo_ros_iws_drive/GazeboRosIwsControllerPlugin.cfg cfg/gazebo_ros_iws_drive/GazeboRosI4wsBridgePlugin.cfg )
endif()

IF(IwsControllersSource)
  catkin_package(LIBRARIES 
  tuw_gazebo_ros_iws_controller  tuw_gazebo_ros_iws_controller_plugin  tuw_gazebo_ros_i4ws_bridge_plugin
  tuw_gazebo_ros_i2ws_controller tuw_gazebo_ros_i2ws_controller_plugin tuw_gazebo_ros_i2ws_bridge_plugin)
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  tuw_gazebo_ros_utils 
  tuw_gazebo_ros_diff_drive
  tuw_gazebo_ros_laser
  tuw_gazebo_ros_gpu_laser
  tuw_gazebo_ros_imu9dof
  
  CATKIN_DEPENDS 
  roscpp 
  rospy 
  nodelet 
  urdf 
  tf 
  tf2_ros 
  dynamic_reconfigure 
  std_msgs
  gazebo_msgs 
  tuw_gazebo_msgs 
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

IF(IwsControllersSource)
  add_subdirectory(src/gazebo_ros_iws_drive)
endif()


install(TARGETS 
  tuw_gazebo_ros_utils 
  tuw_gazebo_ros_diff_drive
  tuw_gazebo_ros_laser
  tuw_gazebo_ros_gpu_laser
  tuw_gazebo_ros_imu9dof
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
