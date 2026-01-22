import launch
import launch_ros.actions
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    remap_mode = LaunchConfiguration("remap_mode")
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("pgo"), "rviz", "pgo.rviz"]
    )
    pgo_config_path = PathJoinSubstitution(
        [FindPackageShare("pgo"), "config", "pgo.yaml"]
    )

    lio_config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio.yaml"]
    )


    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                "remap_mode",
                default_value="false",
                description="true: do not start lio_node (use existing LIO for remap/PGO)",
            ),
            launch_ros.actions.Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="lio_node",
                name="lio_node",
                output="screen",
                parameters=[{"config_path": lio_config_path.perform(launch.LaunchContext())}],
                condition=UnlessCondition(remap_mode),
            ),
            launch_ros.actions.ComposableNodeContainer(
                name="pgo_container",
                namespace="pgo",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="pgo",
                        plugin="PGONode",
                        name="pgo_node",
                        parameters=[{"config_path": pgo_config_path.perform(launch.LaunchContext())}],
                    )
                ],
            ),
            launch_ros.actions.Node(
                package="rviz2",
                namespace="pgo",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
            )
        ]
    )
