#!/usr/bin/env python3
"""
Main Gazebo Simulation Launch for Industrial AMR in Warehouse
Launches:
  1. Gazebo Classic with warehouse.world and custom models path
  2. Map Server to publish the warehouse 2D occupancy grid
  3. RViz2 pre-configured with warehouse displays
  4. Spawns 3 AMRs (synapse_amr_1, synapse_amr_2, synapse_amr_3)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def spawn_amr(robot_name, x, y, z, yaw, use_sim_time, xacro_file):
    """Helper function to spawn an AMR and its state publisher"""
    robot_description = Command(['xacro ', xacro_file, ' robot_name:=', robot_name])
    
    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'robot_state_publisher_{robot_name}',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'frame_prefix': robot_name + '/'
        }]
    )

    # Spawn AMR into Gazebo
    spawn_amr_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=f'spawn_{robot_name}',
        output='screen',
        arguments=[
            '-topic', f'/{robot_name}/robot_description',
            '-entity', robot_name,
            '-x', str(x),
            '-y', str(y),
            '-z', str(z),
            '-Y', str(yaw)
        ]
    )
    
    return [robot_state_publisher_cmd, spawn_amr_cmd]

def generate_launch_description():
    pkg_synapse_amr = get_package_share_directory('synapse_amr_warehouse')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Path definitions
    world_file = os.path.join(pkg_synapse_amr, 'worlds', 'warehouse.world')
    models_dir = os.path.join(pkg_synapse_amr, 'models')
    xacro_file = os.path.join(pkg_synapse_amr, 'urdf', 'synapse_amr.urdf.xacro')
    rviz_config = os.path.join(pkg_synapse_amr, 'rviz', 'warehouse_view.rviz')
    map_yaml = os.path.join(pkg_synapse_amr, 'maps', 'warehouse_map.yaml')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    rviz = LaunchConfiguration('rviz', default='true')
    publish_map = LaunchConfiguration('publish_map', default='true')

    # Set Gazebo Model Path to include our custom warehouse models
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    new_model_path = f"{models_dir}:{existing_model_path}" if existing_model_path else models_dir
    set_model_path_cmd = SetEnvironmentVariable('GAZEBO_MODEL_PATH', new_model_path)

    # Gazebo Server and Client
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'gui': gui,
            'verbose': 'false'
        }.items()
    )

    # Map Server: Publish the pre-computed warehouse occupancy grid
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(publish_map)
    )

    # Lifecycle manager to activate map_server automatically
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server']
        }],
        condition=IfCondition(publish_map)
    )

    # RViz2 Visualization
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz)
    )

    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock'))
    ld.add_action(DeclareLaunchArgument('gui', default_value='true', description='Set to "false" to run Gazebo headless'))
    ld.add_action(DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz2 for visualization'))
    ld.add_action(DeclareLaunchArgument('publish_map', default_value='true', description='Publish preloaded warehouse occupancy map'))

    # Environment
    ld.add_action(set_model_path_cmd)

    # Core Nodes
    ld.add_action(gazebo_cmd)
    ld.add_action(map_server_cmd)
    ld.add_action(lifecycle_manager_cmd)
    ld.add_action(rviz_cmd)

    # Spawn 3 AMRs in different corners of the warehouse
    # AMR 1 (Southeast - Charging Dock)
    for cmd in spawn_amr('synapse_amr_1', 8.0, -6.5, 0.05, 3.14159, use_sim_time, xacro_file):
        ld.add_action(cmd)
        
    # AMR 2 (Northwest - Outbound)
    for cmd in spawn_amr('synapse_amr_2', -8.0, 5.9, 0.05, 0.0, use_sim_time, xacro_file):
        ld.add_action(cmd)
        
    # AMR 3 (Southwest - Inbound Staging)
    for cmd in spawn_amr('synapse_amr_3', -7.5, -5.9, 0.05, 0.0, use_sim_time, xacro_file):
        ld.add_action(cmd)

    return ld
