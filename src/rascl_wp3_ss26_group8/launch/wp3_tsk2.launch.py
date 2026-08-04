"""Launch the WP3 Task 2 online pick-and-place node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    parameter_names = (
        "goal_topic",
        "goal_x",
        "goal_y",
        "min_feasible_radius",
        "max_feasible_radius",
        "travel_z",
        "pick_z",
        "place_z",
        "motion_duration",
        "gripper_duration",
        "rate_hz",
        "apply_board_xy_compensation",
        "require_torque_service",
        "execute",
        "save_csv",
        "output_directory",
    )
    configurations = {
        name: LaunchConfiguration(name) for name in parameter_names
    }

    task2_node = Node(
        package="rascl_wp3_ss26_group8",
        executable="wp3_tsk2",
        name="wp3_tsk2",
        output="screen",
        parameters=[configurations],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "goal_topic",
                default_value="/goal_poses",
                description="Runtime cube-centre topic (geometry_msgs/msg/Point).",
            ),
            DeclareLaunchArgument("goal_x", default_value="0.18"),
            DeclareLaunchArgument("goal_y", default_value="-0.04"),
            DeclareLaunchArgument(
                "min_feasible_radius",
                default_value="0.10",
                description="Documented minimum collision-free reachable radius [m].",
            ),
            DeclareLaunchArgument(
                "max_feasible_radius",
                default_value="0.18439088914585774",
                description="Documented maximum radius; must equal the fixed goal radius [m].",
            ),
            DeclareLaunchArgument("travel_z", default_value="0.10"),
            DeclareLaunchArgument("pick_z", default_value="0.045"),
            DeclareLaunchArgument("place_z", default_value="0.045"),
            DeclareLaunchArgument("motion_duration", default_value="5.0"),
            DeclareLaunchArgument("gripper_duration", default_value="5.0"),
            DeclareLaunchArgument("rate_hz", default_value="50.0"),
            DeclareLaunchArgument(
                "apply_board_xy_compensation", default_value="true"
            ),
            DeclareLaunchArgument(
                "require_torque_service",
                default_value="false",
                description="Use true for physical hardware and false for fake hardware.",
            ),
            DeclareLaunchArgument(
                "execute",
                default_value="true",
                description="The node moves only after a valid /goal_poses message arrives.",
            ),
            DeclareLaunchArgument("save_csv", default_value="true"),
            DeclareLaunchArgument(
                "output_directory", default_value="/tmp/rascl_wp3_tsk2"
            ),
            task2_node,
        ]
    )
