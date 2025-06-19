## Mobile Manipulator Package
Required Packages for this to work: 
- `Universal_Robots_ROS2_Description` 
- `Universal_Robots_ROS2_Driver` (changes were made to the `ur_moveit_config`, look at `my_ur` README for more info)
- `my_ur` (package in my Github repos, mobile_manipulator branch)
- `moveit2` 
- `gz_ros2_control` (changes made, look at `my_ur` README for more info)

### On main branch
With the new launch file, just run:
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

To launch together with MoveIt (to control the UR10 arm), run:
```
ros2 launch my_platform launch_sim_mnoveit.launch.py
```
