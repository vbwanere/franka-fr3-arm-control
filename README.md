# Franka panda arm control using ROS2 for pick and place operations (ROS2 Humble).

## Getting Started:

```
git clone -b humble https://github.com/frankaemika/franka_description.git
git clone -b humble https://github.com/frankaemika/franka_ros2.git
```

### Building PackagesFrom inside the repo ```franka-panda-arm-control``` run each of the following lines individually:
```
cd src
ros2 pkg create --build-type ament_cmake panda_pick_bringup
ros2 pkg create --build-type ament_cmake franka_ros2_control
ros2 pkg create --build-type ament_cmake franka_task_planning
````


```
pkill -9 -f "ros2 launch"
pkill -9 -f "ign gazebo"
pkill -9 -f "ruby /usr/bin/ign"
pkill -9 -f "controller_manager"
pkill -9 -f "robot_state_publisher"
pkill -9 -f "parameter_bridge"
pkill -9 -f rviz2
sleep 3
ros2 daemon stop
ros2 daemon start

colcon build
source install/setup.bash
ros2 launch panda_pick_bringup pick_and_place.launch.py
```

## TODO:
MoveIt2 integration — wire up franka_fr3_moveit_config so you can plan via RViz GUI (drag the end-effector, hit Plan & Execute). This is the natural next step.

AprilTag pipeline — bring back your detection code from the old project. The camera topics are ready.

Load your final.world — convert your old Gazebo Classic .world files to Ignition SDF and load instead of sensor_demo_world.sdf.

Gripper control — franka_finger_joint1 is in the URDF but no controller for it. Add a position_controller for the gripper, then you can pinch/release.

