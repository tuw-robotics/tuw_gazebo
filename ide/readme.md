# ROS2 Gazebo
## VSCode
If you like to use VSCode link the 
* ln -s ./ws01/src/tuw2_gazebo/ide/vscode .vscode 
* ln -s ./ws01/src/tuw2_gazebo/ide/readme.md readme.md
## Commands
```
tmux new -s gz \; split-window -v \; select-pane -t 1 \; split-window -v \; select-pane -t 0 \; attach
ros2 launch tuw_gazebo world.launch.py room:=cave
ros2 launch tuw_gazebo_models spawn_robot.launch.py model_name:=robot0 robot:=pioneer3dx  X:=-6.0 Y:=-6.0 Theta:=0.7
```

## GIT
```
git clone -b ros2  git@github.com:ros-simulation/gazebo_ros_pkgs.git ./ws00/src/gazebo_ros_pkgs
git clone -b ros2  git@github.com:tuw-robotics/tuw2_gazebo.git       ./ws01/src/tuw2_gazebo
git clone -b ros2  git@github.com:tuw-robotics/tuw2_launches.git     ./ws01/src/tuw2_launches
```
