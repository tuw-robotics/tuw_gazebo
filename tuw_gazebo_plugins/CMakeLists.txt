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
  gazebo_ros_utils 
  gazebo_ros_diff_drive
  
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

## Executables
add_library(gazebo_ros_utils src/gazebo_ros_utils.cpp)
target_link_libraries(gazebo_ros_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_definitions(-fPIC) # what is this for?

## Plugins
add_library(gazebo_ros_diff_drive src/gazebo_ros_diff_drive.cpp)
target_link_libraries(gazebo_ros_diff_drive gazebo_ros_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS 
  gazebo_ros_utils
  gazebo_ros_diff_drive
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
