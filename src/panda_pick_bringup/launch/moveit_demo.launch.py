import os
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    # Build robot_description from our combined xacro (includes the D435 camera)
    xacro_file = os.path.join(
        get_package_share_directory('panda_pick_bringup'),
        'urdf',
        'fr3_with_camera.urdf.xacro',
    )
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={
            'robot_type': 'fr3',
            'hand': 'true',
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': 'franka_hand',
        },
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # SRDF from Franka — defines planning groups (arm, gripper, etc.)
    srdf_xacro = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.srdf.xacro',
    )
    robot_description_semantic_config = xacro.process_file(
        srdf_xacro,
        mappings={'hand': 'true', 'ee_id': 'franka_hand'},
    )
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config.toxml()
    }

    # Kinematics, planning, controllers — all from Franka's MoveIt config
    kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters':
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/ResolveConstraintFrames '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_yaml = load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml')
    if ompl_yaml:
        ompl_planning_pipeline_config['move_group'].update(ompl_yaml)

    moveit_controllers_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/fr3_controllers.yaml'
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_controllers_yaml,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': True},
        ],
    )

    # MoveIt-aware RViz
    rviz_config = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'),
        'rviz', 'moveit.rviz',
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([move_group_node, rviz_node])
