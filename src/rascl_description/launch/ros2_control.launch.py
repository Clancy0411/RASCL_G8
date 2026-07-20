"""Start the RASCL ros2_control stack and the EtherCAT bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
        default_value="csp",
        description="Lower-level drive command mode: profile or csp.",
    )
    controller_config_arg = DeclareLaunchArgument(
        "controller_config",
        default_value="controllers_csp.yaml",
        description="Controller YAML file, e.g. controllers.yaml or controllers_csp.yaml.",
    )
    pdo_cycle_ns_arg = DeclareLaunchArgument(
        "pdo_cycle_ns",
        default_value="20000000",
        description="CSP PDO cycle in nanoseconds (20 ms = 50 Hz).",
    )
    pdo_timeout_us_arg = DeclareLaunchArgument(
        "pdo_timeout_us",
        default_value="5000",
        description="Timeout for one EtherCAT process-data receive call in microseconds.",
    )
    enable_dc_sync_arg = DeclareLaunchArgument(
        "enable_dc_sync",
        default_value="false",
        description="Use EtherCAT DC-Sync. The conservative default is SM-Sync.",
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
    lowerarm_direction_arg = DeclareLaunchArgument(
        "lowerarm_direction",
        default_value="-1",
        description=(
            "Drive 2 encoder-to-URDF sign. -1 is the physical lowerarm direction "
            "correction and must be paired with lowerarm_home_offset_counts:=802816."
        ),
    )
    start_bridge_arg = DeclareLaunchArgument(
        "start_bridge",
        default_value="false",
        description="Reuse homing.launch.py by default; true starts a standalone bridge.",
    )
    shoulder_home_offset_arg = DeclareLaunchArgument(
        "shoulder_home_offset_counts",
        default_value="0",
        description="Raw count at the physical URDF zero pose for shoulder_joint.",
    )
    upperarm_home_offset_arg = DeclareLaunchArgument(
        "upperarm_home_offset_counts",
        default_value="-802816",
        description="Raw count at the physical URDF zero pose for upperarm_joint.",
    )
    lowerarm_home_offset_arg = DeclareLaunchArgument(
        "lowerarm_home_offset_counts",
        default_value="802816",
        description=(
            "Raw count at the physical URDF zero pose for lowerarm_joint when "
            "lowerarm_direction:=-1."
        ),
    )
    spur_gear_home_offset_arg = DeclareLaunchArgument(
        "spur_gear_home_offset_counts",
        default_value="0",
        description="Raw count at the physical URDF zero pose for spur_gear_joint.",
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
            " lowerarm_direction:=", LaunchConfiguration("lowerarm_direction"),
            " shoulder_home_offset_counts:=", LaunchConfiguration("shoulder_home_offset_counts"),
            " upperarm_home_offset_counts:=", LaunchConfiguration("upperarm_home_offset_counts"),
            " lowerarm_home_offset_counts:=", LaunchConfiguration("lowerarm_home_offset_counts"),
            " spur_gear_home_offset_counts:=", LaunchConfiguration("spur_gear_home_offset_counts"),
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
                "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
                "control_mode": LaunchConfiguration("control_mode"),
                "slave_indices": [0, 1, 2, 3],
                "sdo_delay_s": 0.05,
                "motion_timeout_s": 8.0,
                "verbose": True,
                "profile_velocity": 0,
                "profile_acceleration": 0,
                "profile_deceleration": 0,
                "pdo_cycle_ns": ParameterValue(
                    LaunchConfiguration("pdo_cycle_ns"), value_type=int
                ),
                "pdo_timeout_us": ParameterValue(
                    LaunchConfiguration("pdo_timeout_us"), value_type=int
                ),
                "enable_dc_sync": ParameterValue(
                    LaunchConfiguration("enable_dc_sync"), value_type=bool
                ),
                "homing_methods": [28, 28, 24, 24],
                "reference_inputs": [2, 2, 2, 1],
                "homing_offsets": [0, 0, 0, 0],
                "homing_search_speeds": [1000, 1000, 1000, 1000],
                "homing_zero_speeds": [200, 200, 200, 200],
                "homing_accelerations": [1000, 1000, 1000, 1000],
                "test_drive_index": 0,
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
        parameters=[
            robot_description,
            PathJoinSubstitution(
                [pkg_share, "config", LaunchConfiguration("controller_config")]
            ),
        ],
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
            pdo_cycle_ns_arg,
            pdo_timeout_us_arg,
            enable_dc_sync_arg,
            axis_counts_arg,
            gripper_counts_arg,
            lowerarm_direction_arg,
            shoulder_home_offset_arg,
            upperarm_home_offset_arg,
            lowerarm_home_offset_arg,
            spur_gear_home_offset_arg,
            start_bridge_arg,
            GroupAction(
                condition=IfCondition(LaunchConfiguration("start_bridge")),
                actions=[bridge_node],
            ),
            robot_state_publisher_node,
            TimerAction(period=2.0, actions=[ros2_control_node]),
            TimerAction(period=4.0, actions=[joint_state_broadcaster_spawner]),
            TimerAction(period=5.0, actions=[rascl_position_controller_spawner]),
        ]
    )
