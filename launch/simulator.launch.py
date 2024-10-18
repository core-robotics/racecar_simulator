from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():

    ld = LaunchDescription()

    # Get the share directory for the racecar_simulator package
    pkg_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

    # Define the path to the rviz configuration file
    rviz_config_file = os.path.join(pkg_dir, "params", "simulator.rviz")
    simulation_config_file = os.path.join(pkg_dir, "params", "simulation.yaml")

    # # Define the path to the robot description file
    car0_xacro_file = os.path.join(pkg_dir, "params", "racecar0.xacro")
    car1_xacro_file = os.path.join(pkg_dir, "params", "racecar1.xacro")
     
    robot0_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace="racecar0",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", str(car0_xacro_file), " prefix:=0"]), value_type=str
                ),
                # "use_sim_time": True
            }
        ],
    )

    robot1_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace="racecar1",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", str(car1_xacro_file), " prefix:=1"]), value_type=str
                ),
                # "use_sim_time": True
            }
        ],
    )

    racecar_node = Node(
        package="racecar_simulator",
        executable="simulator",
        name="racecar_simulator",
        output="screen",
        parameters=[simulation_config_file,
                    # {"use_sim_time": True}
                    ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
        # parameters=[{"use_sim_time": True}],
    )

    ld.add_action(rviz_node)
    ld.add_action(racecar_node)
    ld.add_action(robot0_state_publisher_node)
    ld.add_action(robot1_state_publisher_node)
    

    return ld
