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
pkill -9 -f rviz2
pkill -9 -f robot_state_publisher
pkill -9 -f joint_state_publisher_gui
sleep 1
pgrep -af "rviz2\|robot_state_publisher\|joint_state_publisher" || echo "all dead"

cd ~/Vaibhav-GitHub/franka-panda-arm-control
colcon build --packages-select fanuc_description
source install/setup.bash
ros2 launch fanuc_description view_robot.launch.py
```