# tuw_gazebo
GazeboSim Models such as a pioneer differential drive
## ROS2
A ROS2 version can be found under [tuw2_gazebo](https://github.com/tuw-robotics/tuw2_gazebo).
## Installation Requirements
* tuw_gazebo depends on tuw_nav_msgs which are located in the pkg tuw_msgs (https://github.com/tuw-robotics/tuw_msgs)
``` git clone https://github.com/tuw-robotics/tuw_msgs.git YOUR_CATKIN_SRC_NEXT_TO_TUW_GAZEBO/ ```
* ros general
``` sudo apt-get install ros-kinetic-robot-state-publisher ros-kinetic-joint-state-publisher ros-kinetic-xacro```
* the following ros pkgs depending on your gazebo version
  * Gazebo7
``` sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-xacro```
  * Gazebo8
``` sudo apt-get install ros-kinetic-gazebo8-ros-pkgs ros-kinetic-xacro```
