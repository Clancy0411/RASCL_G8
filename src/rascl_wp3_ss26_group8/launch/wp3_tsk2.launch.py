"""Launch the WP3 Task 2 online pick-and-place node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    parameter_names = (
        "goal_topic",
        "goal_x",
        "goal_y",
        "inner_route_max_radius",
        "middle_route_max_radius",
        "inner_route_x",
        "inner_route_y",
        "outer_route_x",
        "outer_route_y",
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
            DeclareLaunchArgument("goal_x", default_value="0.1812"),
            DeclareLaunchArgument("goal_y", default_value="-0.0336"),
            DeclareLaunchArgument(
                "inner_route_max_radius",
                default_value="0.17",
                description="Use the inner push-out route below this radius [m].",
            ),
            DeclareLaunchArgument(
                "middle_route_max_radius",
                default_value="0.20",
                description=(
                    "Use the direct route through this radius; larger inputs "
                    "use outer pull-in [m]."
                ),
            ),
            DeclareLaunchArgument(
                "inner_route_x",
                default_value="0.1517",
                description="Legacy group-29 inner placement-route X [m].",
            ),
            DeclareLaunchArgument(
                "inner_route_y",
                default_value="-0.0282",
                description="Legacy group-29 inner placement-route Y [m].",
            ),
            DeclareLaunchArgument(
                "outer_route_x",
                default_value="0.2107",
                description="Legacy group-29 outer placement-route X [m].",
            ),
            DeclareLaunchArgument(
                "outer_route_y",
                default_value="-0.0391",
                description="Legacy group-29 outer placement-route Y [m].",
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
                "output_directory",
                default_value=PathJoinSubstitution([
                    FindPackageShare("rascl_wp3_ss26_group8"),
                    "trajectories",
                ]),
                description="Package directory for Task 2 input and trajectory CSV files.",
            ),
            task2_node,
        ]
    )
