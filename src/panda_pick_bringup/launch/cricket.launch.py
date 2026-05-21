import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import re

def generate_launch_description():
    os.environ['GZ_SIM_RESOURCE_PATH'] = ':'.join([
        os.path.dirname(get_package_share_directory('franka_description')),
        os.path.dirname(get_package_share_directory('realsense2_description')),
        get_package_share_directory('panda_pick_bringup') + '/models',
    ])

    robot_type = 'fr3'
    load_gripper = 'true'
    franka_hand = 'franka_hand'

    pkg_share = get_package_share_directory('panda_pick_bringup')

    xacro_file = os.path.join(pkg_share, 'urdf', 'fr3_with_camera.urdf.xacro')
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={
            'robot_type': robot_type,
            'hand': load_gripper,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': franka_hand,
        },
    )

    robot_description = {'robot_description': robot_description_config.toxml()}

    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    world_path = os.path.join(
        get_package_share_directory('panda_pick_bringup'),
        'worlds',
        'cricket_world.sdf',
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'fr3',
            '-allow_renaming', 'true',
            '-x', '1.0',
            '-y', '-0.75',
            '-z', '0.0',
            '-Y', '0.0',     # yaw in radians; 0 = facing +X (down the pitch)
        ],
        output='screen',
    )

    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fr3_arm_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-type', 'joint_trajectory_controller/JointTrajectoryController',
                   '--param-file', controllers_yaml],
        output='screen',
    )

    load_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fr3_gripper_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-type', 'position_controllers/JointGroupPositionController',
                   '--param-file', controllers_yaml],
        output='screen',
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth@sensor_msgs/msg/Image[gz.msgs.Image',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    rviz_config = os.path.join(pkg_share, 'rviz', 'pick_and_place.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        rsp,
        gz_sim,
        spawn,
        gz_bridge,
        rviz,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[load_jsb],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_jsb,
                on_exit=[load_arm_controller, load_gripper_controller],
            )
        ),
    ])
