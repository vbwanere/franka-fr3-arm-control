import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(
    get_package_share_directory('franka_description'))
    robot_type = 'fr3'
    load_gripper = 'true'
    franka_hand = 'franka_hand'

    xacro_file = os.path.join(
        get_package_share_directory('panda_pick_bringup'),
        'urdf',
        'fr3_with_camera.urdf.xacro',
    )

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

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    world_path = os.path.join(
        get_package_share_directory('franka_gazebo_bringup'),
        'worlds',
        'empty_no_gravity.sdf',
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
        ],
        output='screen',
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'joint_state_broadcaster'],
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

    return LaunchDescription([
        rsp,
        gz_sim,
        spawn,
        gz_bridge,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
    ])
