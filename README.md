## Robot Package Template

This is a GitHub template. You can make your own copy by clicking the green "Use this template" button.

It is recommended that you keep the repo/package name the same, but if you do change it, ensure you do a "Find all" using your IDE (or the built-in GitHub IDE by hitting the `.` key) and rename all instances of `my_platform` to whatever your project's name is.

Note that each directory currently has at least one file in it to ensure that git tracks the files (and, consequently, that a fresh clone has direcctories present for CMake to find). These example files can be removed if required (and the directories can be removed if `CMakeLists.txt` is adjusted accordingly).


### On main branch
With new launch file, just run:
```
ros2 launch my_platform launch_sim.launch.py # launches all applications required for SLAM and Localization
```

Once the above is launched (give it 10 seconds for everything to fully launch, including controllers and slam_toolbox),
in another terminal, run:
```
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
  points: [
    {                                           
      positions: [1.0, 0.5, 1.0, 1.0, 1.0, 1.0],
      time_from_start: {sec: 2, nanosec: 0}
    }
  ]
}"

```
