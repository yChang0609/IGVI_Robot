"""Nav2 planning-only stack.

Runs map_server, amcl, planner_server, and bt_navigator with a custom BT that
calls ComputePathToPose but skips FollowPath. The controller_server,
behavior_server, smoother_server, velocity_smoother, and waypoint_follower are
intentionally not launched — motion_arbiter is the sole velocity authority.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = "/configs/nav2_params.yaml"

    lifecycle_nodes = [
        "map_server",
        "amcl",
        "planner_server",
        "bt_navigator",
    ]

    nodes = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[params],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[params],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {
                    "autostart": True,
                    "node_names": lifecycle_nodes,
                    "bond_timeout": 0.0,
                }
            ],
        ),
    ]

    return LaunchDescription(nodes)
