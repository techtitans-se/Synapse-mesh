import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_synapse_mesh_bringup = get_package_share_directory('synapse_mesh_bringup')
    
    world_file = os.path.join(pkg_synapse_mesh_bringup, 'worlds', 'warehouse.world')
    
    from launch.substitutions import LaunchConfiguration
    from launch.actions import DeclareLaunchArgument

    gui_arg = DeclareLaunchArgument('gui', default_value='true', description='Set to "false" to run headless')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'gui': LaunchConfiguration('gui')
        }.items()
    )

    from launch.actions import TimerAction
    tb3_model_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models', 'turtlebot3_burger', 'model.sdf')

    spawn_amr_1 = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-entity', 'amr_1',
             '-robot_namespace', 'amr_1',
             '-x', '-3.2', '-y', '0.0', '-z', '0.05', '-Y', '0.0',
             '-file', tb3_model_path],
        output='screen'
    )

    spawn_amr_2 = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-entity', 'amr_2',
             '-robot_namespace', 'amr_2',
             '-x', '3.2', '-y', '0.0', '-z', '0.05', '-Y', '3.1415',
             '-file', tb3_model_path],
        output='screen'
    )

    spawn_amr_3 = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-entity', 'amr_3',
             '-robot_namespace', 'amr_3',
             '-x', '0.0', '-y', '-3.2', '-z', '0.05', '-Y', '1.5708',
             '-file', tb3_model_path],
        output='screen'
    )

    from launch_ros.actions import Node

    # Synapse-Mesh Nodes for amr_1
    amr_1_lma = Node(package='lma_node', executable='lma_node', name='lma', namespace='amr_1', parameters=[{'robot_id': 'amr_1'}])
    amr_1_vos = Node(package='vos_node', executable='vos_node', name='vos', namespace='amr_1', parameters=[{'robot_id': 'amr_1'}])
    amr_1_st_lease = Node(package='st_lease_node', executable='st_lease_node', name='st_lease', namespace='amr_1', parameters=[{'robot_id': 'amr_1', 'priority': 2.0}])

    # Synapse-Mesh Nodes for amr_2
    amr_2_lma = Node(package='lma_node', executable='lma_node', name='lma', namespace='amr_2', parameters=[{'robot_id': 'amr_2'}])
    amr_2_vos = Node(package='vos_node', executable='vos_node', name='vos', namespace='amr_2', parameters=[{'robot_id': 'amr_2'}])
    amr_2_st_lease = Node(package='st_lease_node', executable='st_lease_node', name='st_lease', namespace='amr_2', parameters=[{'robot_id': 'amr_2', 'priority': 1.0}])

    # Synapse-Mesh Nodes for amr_3
    amr_3_lma = Node(package='lma_node', executable='lma_node', name='lma', namespace='amr_3', parameters=[{'robot_id': 'amr_3'}])
    amr_3_vos = Node(package='vos_node', executable='vos_node', name='vos', namespace='amr_3', parameters=[{'robot_id': 'amr_3'}])
    amr_3_st_lease = Node(package='st_lease_node', executable='st_lease_node', name='st_lease', namespace='amr_3', parameters=[{'robot_id': 'amr_3', 'priority': 3.0}])

    # Web Dashboard Bridge & FMS Server
    from launch.launch_description_sources import AnyLaunchDescriptionSource

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')
        )
    )
    fms_server = Node(package='synapse_mesh_bringup', executable='fms_server', name='fms_server')

    delayed_spawn = TimerAction(
        period=5.0, # wait 5 seconds for Gazebo to fully load
        actions=[spawn_amr_1, spawn_amr_2, spawn_amr_3, 
                 amr_1_lma, amr_1_vos, amr_1_st_lease, 
                 amr_2_lma, amr_2_vos, amr_2_st_lease,
                 amr_3_lma, amr_3_vos, amr_3_st_lease,
                 rosbridge, fms_server]
    )

    return LaunchDescription([
        gui_arg,
        gazebo,
        delayed_spawn
    ])
