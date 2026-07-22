"""Launch RViz with the RASCL robot description."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # This launch file is intended for URDF/RViz inspection without real hardware.
    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        description="Start joint_state_publisher_gui for manual URDF inspection.",
    )

    pkg_share = FindPackageShare("rascl_description")
    # robot_state_publisher consumes this xacro-generated robot_description.
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([pkg_share, "urdf", "rascl.urdf"]),
            " use_fake_hardware:=true",
        ]
    )

    # URDF is XML text.  Force the parameter type so launch_ros does not try to
    # interpret xacro output (including XML comments containing colons) as YAML.
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"use_gui": LaunchConfiguration("use_gui")}],
    )

    # RViz reads /robot_description and /tf to display the articulated model.
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([pkg_share, "rviz", "urdf.rviz"])],
    )

    return LaunchDescription(
        [
            use_gui_arg,
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
