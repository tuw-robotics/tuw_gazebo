# tuw_gazebo
GazeboSim Models such as a pioneer differential drive
## ROS2
A ROS2 version can be found under [tuw2_gazebo](https://github.com/tuw-robotics/tuw2_gazebo).
## Installation Requirements
* tuw_gazebo depends on tuw_nav_msgs which are located in the pkg tuw_msgs (https://github.com/tuw-robotics/tuw_msgs)
``` 
export ROS_DISTRO=noetic
git clone https://github.com/tuw-robotics/tuw_msgs.git YOUR_CATKIN_SRC_NEXT_TO_TUW_GAZEBO/ 
# git clone -b ${ROS_DISTRO}-devel git@github.com:ros-simulation/gazebo_ros_pkgs.git
sudo apt-get install ros-${ROS_DISTRO}-gazebo-ros-pkgs 
sudo apt-get install ros-${ROS_DISTRO}-robot-state-publisher ros-${ROS_DISTRO}-joint-state-publisher ros-${ROS_DISTRO}-xacro
```
