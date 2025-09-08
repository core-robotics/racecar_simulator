from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ld = LaunchDescription()

    pkg_dir = '/home/a/racecar_simulator/src/racecar_simulator/'


    map_name = "Austin"
    # 1. Austin
    # 2. Melbourne
    # 3. Oschersleben
    # 4. Shanghai
    # 5. Zandvoort
    # 6. BrandsHatch
    # 7. MexicoCity
    # 8. Silverstone
    # 9. Budapest
    # 10. IMS
    # 11. Montreal
    # 12. Sochi
    # 13. Catalunya
    # 14. Monza
    # 15. Sakhir
    # 16. Spa
    # 17. MoscowRaceway
    # 18. SaoPaulo
    # 19. Spielberg
    # 20. Hockenheim
    # 21. Nuerburgring
    # 22. Sepang
    # 23. YasMarina

    rviz_config_file = os.path.join(pkg_dir, "params", "simulator.rviz")
    simulation_config_file = os.path.join(pkg_dir, "params", "simulation.yaml")
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
