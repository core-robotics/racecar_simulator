from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import TimerAction, RegisterEventHandler
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    
    ld = LaunchDescription()
    
    # Get the share directory for the racecar_simulator package
    pkg_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

    # Define the path to the map server parameters file
    map_server_params = os.path.join(pkg_dir, 'maps', 'c_track.yaml')
    
    # Define the path to the rviz configuration file
    rviz_config_file = os.path.join(pkg_dir, 'params', 'simulator.rviz')
    
    # # Define the path to the robot description file
    # car0_xacro_file = os.path.join(pkg_dir, 'params', 'racecar0.xacro')
    # car1_xacro_file = os.path.join(pkg_dir, 'params', 'racecar1.xacro')
    
    # car0_desc=Command([
    #     'xacro',
    #     car0_xacro_file,
    #     'prefix:=0',
    # ])
    
    # car1_desc=Command([
    #     'xacro',
    #     car1_xacro_file,
    #     'prefix:=1',
    # ])
    
    
    # robot1_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     namespace='racecar0',
    #     output='screen',
    #     parameters=[{'robot_description': car0_desc}]
    # )
    
    # robot2_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     namespace='racecar1',
    #     output='screen',
    #     parameters=[{'robot_description': car1_desc}]
    # )
    

    
    racecar_node =Node(
        package='racecar_simulator',
        executable='simulator',
        name='racecar_simulator',
        output='screen',
    )
    
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            # 'use_sim_time': True, # Set to True if using simulated time
            'autostart': True, # Automatically startup the managed nodes
            'node_names': ['map_server'], # Specify the lifecycle nodes to be managed
        }],
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_server_params}],
    )
    
    delay_node = TimerAction(
        period=1.0,
        actions=[
            map_server_node
            ,racecar_node
            ,lifecycle_manager_node
            # ,robot1_state_publisher_node
            # ,robot2_state_publisher_node
            ]
    )
    
    node_start=RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=rviz_node,
            on_start=[delay_node]
        )
    )
    ld.add_action(rviz_node)
    ld.add_action(node_start)

    return ld