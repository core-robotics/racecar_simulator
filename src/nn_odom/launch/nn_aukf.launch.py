# launch/nn_aukf.launch.py
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = 'nn_odom'
    share_dir = get_package_share_directory(pkg_name)

    # config yaml 경로
    config_file = os.path.join(
        share_dir,
        'config',
        'nn_aukf.yaml'
    )

    # TorchScript 모델 경로 (설치 후 기준)
    default_model_path = os.path.join(
        share_dir,
        'models',
        '500_loss_0.020788_scripted.pt'
    )

    nn_node = Node(
        package=pkg_name,
        executable='nn_aukf',
        name='nn_aukf',
        output='screen',
        parameters=[
            config_file,
        ]
    )
    
    ld = LaunchDescription()

    ld.add_action(nn_node)

    return ld