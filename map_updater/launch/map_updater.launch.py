import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("map_updater")
    config_path = os.path.join(pkg_dir, "config", "map_updater.yaml")

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="map_updater",
                namespace="map_updater",
                executable="map_updater_node",
                name="map_updater_node",
                output="screen",
                parameters=[{"config_path": config_path}],
            ),
        ]
    )
