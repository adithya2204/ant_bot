import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler,
    TimerAction,
    ExecuteProcess
)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():

    # --------------------------------------------------
    # 1. PATH SETUP (Dynamic, not hardcoded)
    # --------------------------------------------------
    pkg_name = 'ant_bot'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Calculate workspace path dynamically
    workspace_dir = os.path.dirname(pkg_share) 
    
    xacro_path = os.path.join(pkg_share, 'urdf', 'hexapod.xacro')
    controllers_path = os.path.join(pkg_share, 'config', 'controllers.yaml')
    world_path = os.path.join(pkg_share, 'worlds', 'empty.world')

    print(f"[DEBUG] World: {world_path}")
    print(f"[DEBUG] Xacro: {xacro_path}")
    print(f"[DEBUG] Controllers: {controllers_path}")

    # --------------------------------------------------
    # 2. STRICT ENV SETUP
    # --------------------------------------------------
    gazebo_env = {
        'GAZEBO_MODEL_PATH': f"{workspace_dir}:/usr/share/gazebo-11/models",
        'GAZEBO_RESOURCE_PATH': "/usr/share/gazebo-11"
    }

    # --------------------------------------------------
    # 3. LOAD XACRO → URDF
    # --------------------------------------------------
    # MODIFIED: Pass the 'controllers_yaml' argument dynamically
    robot_description = subprocess.check_output(
        ['xacro', xacro_path, 'controllers_yaml:=' + controllers_path],
        text=True
    )

    # XML safety fix
    robot_description = robot_description.replace(
        '<?xml version="1.0" encoding="utf-8"?>', ''
    ).replace(
        "<?xml version='1.0' encoding='utf-8'?>", ''
    )

    # --------------------------------------------------
    # 4. NODES
    # --------------------------------------------------
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True,
            }
        ],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'hexapod_ant',
            '-x', '0.0', '-y', '0.0', '-z', '0.15',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0',
        ],
        output='screen',
    )

    # NEW: Gait Controller Node
    # Added to main list so it launches immediately (before delayed controllers)
    gait_controller_node = Node(
        package='ant_bot',
        executable='gait_controller',
        output='screen',
    )

    # --------------------------------------------------
    # 5. GAZEBO SERVER & CLIENT
    # --------------------------------------------------
    
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             '--verbose',
             world_path
            ],
        output='screen',
        additional_env=gazebo_env
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        additional_env=gazebo_env
    )

    # --------------------------------------------------
    # 6. CONTROLLERS (Delayed)
    # --------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'forward_position_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # Delay controllers until AFTER robot spawn
    spawn_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[
                        joint_state_broadcaster_spawner,
                        forward_position_controller_spawner,
                    ],
                )
            ],
        )
    )

    # --------------------------------------------------
    # 7. LAUNCH DESCRIPTION
    # --------------------------------------------------
    return LaunchDescription([
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        gait_controller_node, # Starts here (early)
        spawn_controllers,    # Starts here (but logic delays execution by 2s)
    ])