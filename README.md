# tuw2_gazebo
gazebo models for ros2

<img src="tuw_gazebo/res/pioneer_in_cave.png" alt="pioneer robot in cave" height="200px" /> <img src="tuw_gazebo/res/pioneer.png" alt="pioneer robot with laser ranger" height="200px" />  <img src="tuw_gazebo/res/pioneer_in_roblab.png" alt="pioneer robot in roblab" height="200px" />

## Demo
Lunch World
```
ros2 launch tuw_gazebo world.launch.py room:=cave
```
Spawn a robot
```
ros2 launch tuw_gazebo_models spawn_robot.launch.py robot:=pioneer3dx
```
Wanderer
```
ros2 run tuw_gazebo_wanderer wanderer_v0.py cmd:=cmd_vel scan:=scan_raw __ns:=r0
```

## IDE
IDE configuartion are avliable in [ide](ide)

## Install missing dependencies
`rosdep install -i --from-path src --rosdistro galactic -y`
