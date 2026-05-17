# franka-panda-arm-control
Franka panda arm control using ROS2 for pick and place operations.


From inside the repo ```franka-panda-arm-control``` run each of the following lines individually:
```
cd src

ros2 pkg create --build-type ament_cmake franka_description

ros2 pkg create --build-type ament_cmake franka_bringup

ros2 pkg create --build-type ament_cmake franka_moveit_config

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

colcon buid
source install/setup.bash
ros2 launch panda_pick_bringup pick_and_place.launch.py
```


