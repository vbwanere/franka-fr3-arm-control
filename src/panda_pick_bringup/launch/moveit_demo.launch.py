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
    srdf_xml = robot_description_semantic_config.toxml()

    # Patch SRDF to match our combined URDF (fr3_with_camera) and ignore
    # collisions between the wrist-mounted D435 camera and gripper/hand links.
    srdf_xml = srdf_xml.replace(
        '<robot name="fr3">', '<robot name="fr3_with_camera">'
    )
    # Remove Franka's virtual_joint (it references 'base' which we don't have);
    # our URDF has a real <world>→fr3_link0 fixed joint that MoveIt sees directly.
    import re
    srdf_xml = re.sub(
        r'<virtual_joint[^/]*/>', '', srdf_xml
    )
    # Add camera-link collision-ignore pairs before </robot>
    camera_ignores = ''.join(
        f'<disable_collisions link1="camera_link" link2="{link}" reason="Never"/>\n'
        for link in [
            'fr3_hand', 'fr3_hand_tcp',
            'fr3_leftfinger', 'fr3_rightfinger',
            'fr3_link5', 'fr3_link6', 'fr3_link7', 'fr3_link8',
        ]
    )
    srdf_xml = srdf_xml.replace('</robot>', camera_ignores + '</robot>')

    robot_description_semantic = {'robot_description_semantic': srdf_xml}


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
