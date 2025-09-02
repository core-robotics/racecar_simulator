from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():

    ld = LaunchDescription()

    pkg_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

    # map_name = "Austin"
    # map_name = "Melbourne"
    # map_name = "Oschersleben"
    map_name = "Shanghai"
    # map_name = "Zandvoort"
    # map_name = "BrandsHatch"
    # map_name = "MexicoCity"
    # map_name = "Silverstone"
    # map_name = "Budapest"
    # map_name = "IMS"
    # map_name = "Montreal"
    # map_name = "Sochi"
    # map_name = "Catalunya"
    # map_name = "Monza"
    # map_name = "Sakhir"
    # map_name = "Spa"
    # map_name = "MoscowRaceway"
    # map_name = "SaoPaulo"
    # map_name = "Spielberg"
    # map_name = "Hockenheim"
    # map_name = "LICENSE"
    # map_name = "Nuerburgring"
    # map_name = "Sepang"
    # map_name = "YasMarina"

    rviz_config_file = os.path.join(pkg_dir, "params", "simulator.rviz")
    simulation_config_file = os.path.join(pkg_dir, "params", "simulation.yaml")
    env_config_file = os.path.join(pkg_dir, "params", "environment.yaml")
    map_folder = os.path.join(pkg_dir, "maps/f1tenth_racetracks")
    map_img = os.path.join(map_folder, map_name, map_name + "_map.png")
    map_yaml = os.path.join(map_folder, map_name, map_name + "_map.yaml")
    map_center = os.path.join(map_folder, map_name, map_name + "_centerline.csv")


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
        parameters=[
            simulation_config_file,
            # {"use_sim_time": True},   
        ],
    )

    map_publisher_node = Node(
        package="racecar_simulator",
        executable="map_publisher",
        name="map_publisher",
        output="screen",
        parameters=[
            {"map_img_file_path": map_img},
            {"map_yaml_file_path": map_yaml},
            {"race_line_file_path": map_center},
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
    ld.add_action(map_publisher_node)
    ld.add_action(robot0_state_publisher_node)
    ld.add_action(robot1_state_publisher_node)

    return ld
