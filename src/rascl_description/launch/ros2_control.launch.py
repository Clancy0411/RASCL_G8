"""Start the RASCL ros2_control stack and the EtherCAT bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments keep the same package usable with fake hardware, real
    # EtherCAT hardware, and different lab workstation network-interface names.
    interface_arg = DeclareLaunchArgument(
        "interface",
        default_value="robot_interface",
        description="EtherCAT network interface, e.g. robot_interface or enx...",
    )
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false",
        description="Use internal fake hardware for URDF/controller tests without EtherCAT.",
    )
    host_arg = DeclareLaunchArgument(
        "host",
        default_value="127.0.0.1",
        description="Local TCP host used between the C++ hardware plugin and the pysoem bridge.",
    )
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="15001",
        description="Local TCP port used between the C++ hardware plugin and the pysoem bridge.",
    )
    control_mode_arg = DeclareLaunchArgument(
        "control_mode",
        default_value="profile",
        description="Lower-level drive command mode: profile or csp.",
    )
    controller_config_arg = DeclareLaunchArgument(
        "controller_config",
        default_value="controllers.yaml",
        description="Controller YAML file, e.g. controllers.yaml or controllers_csp.yaml.",
    )

    axis_counts_arg = DeclareLaunchArgument(
        "axis_counts_per_revolution",
        default_value="3211264",
        description="Joint output counts per radian basis for axes 1-3: 4096 * 196 by default.",
    )
    gripper_counts_arg = DeclareLaunchArgument(
        "gripper_counts_per_revolution",
        default_value="1323008",
        description="Joint output counts per revolution for the end-effector drive: 4096 * 323 by default.",
    )

    pkg_share = FindPackageShare("rascl_description")

    # xacro expands both the visual model and the ros2_control hardware parameters.
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([pkg_share, "urdf", "rascl.urdf"]),
            " use_fake_hardware:=", LaunchConfiguration("use_fake_hardware"),
            " host:=", LaunchConfiguration("host"),
            " port:=", LaunchConfiguration("port"),
            " interface:=", LaunchConfiguration("interface"),
            " control_mode:=", LaunchConfiguration("control_mode"),
            " axis_counts_per_revolution:=", LaunchConfiguration("axis_counts_per_revolution"),
            " gripper_counts_per_revolution:=", LaunchConfiguration("gripper_counts_per_revolution"),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # The bridge owns pysoem/EtherCAT and provides a local TCP endpoint to C++.
    bridge_node = Node(
        package="rascl_hardware_interface",
        executable="rascl_faulhaber_bridge.py",
        name="rascl_faulhaber_bridge",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_fake_hardware")),
        parameters=[
            {
                "interface": LaunchConfiguration("interface"),
                "host": LaunchConfiguration("host"),
                "port": LaunchConfiguration("port"),
                "slave_indices": [0, 1, 2, 3],
                "sdo_delay_s": 0.05,
                "motion_timeout_s": 8.0,
                "verbose": True,
                "profile_velocity": 0,
                "profile_acceleration": 0,
                "profile_deceleration": 0,
                "configure_pdo_mapping": True,
                "enable_dc_sync": False,
                "dc_cycle_ns": 20000000,
                "pdo_timeout_us": 20000,
            }
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, PathJoinSubstitution([pkg_share, "config", LaunchConfiguration("controller_config")])],
        output="screen",
    )

    # joint_state_broadcaster publishes /joint_states for RViz and robot_state_publisher.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # The forward-command controller accepts absolute joint position commands.
    rascl_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rascl_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription(
        [
            interface_arg,
            use_fake_hardware_arg,
            host_arg,
            port_arg,
            control_mode_arg,
            controller_config_arg,
            axis_counts_arg,
            gripper_counts_arg,
            bridge_node,
            robot_state_publisher_node,
            TimerAction(period=2.0, actions=[ros2_control_node]),
            TimerAction(period=4.0, actions=[joint_state_broadcaster_spawner]),
            TimerAction(period=5.0, actions=[rascl_position_controller_spawner]),
        ]
    )
