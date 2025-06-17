## Robot Package Template

This is a GitHub template. You can make your own copy by clicking the green "Use this template" button.

It is recommended that you keep the repo/package name the same, but if you do change it, ensure you do a "Find all" using your IDE (or the built-in GitHub IDE by hitting the `.` key) and rename all instances of `my_platform` to whatever your project's name is.

Note that each directory currently has at least one file in it to ensure that git tracks the files (and, consequently, that a fresh clone has direcctories present for CMake to find). These example files can be removed if required (and the directories can be removed if `CMakeLists.txt` is adjusted accordingly).


### On main branch
With new launch file, just run:
```
ros2 launch my_platform launch_sim.launch.py # launches all applications required for SLAM and Localization
```
Outdated:
```
cd dev_ws
source install/local_setup.bash

ros2 launch my_platform launch_sim.launch.py # launches turtlebot4, default world, and rviz

ros2 run teleop_twist_keyboard teleop_twist_keyboard # to control turtlebot4

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true slam_params_file:=/home/bmanara/dev_ws/src/my_platform/config/mapper_params_online_async.yaml # launch slam

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true # launch nav2
```
