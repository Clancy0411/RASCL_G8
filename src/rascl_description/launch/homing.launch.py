"""Start one bridge for reference Homing and the later CSP handoff.

The master starts with the proven SDO-only PRE-OP Homing path.  After home_all,
ros2_control reuses this bridge; its first ENTER_CSP_ALL lazily maps PDOs and
enters CSP without closing the master or sending a disable controlword.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    interface_arg = DeclareLaunchArgument(
        "interface",
        default_value="robot_interface",
        description="EtherCAT network interface, e.g. robot_interface or enx...",
    )
    host_arg = DeclareLaunchArgument("host", default_value="127.0.0.1")
    port_arg = DeclareLaunchArgument("port", default_value="15001")
    motion_timeout_arg = DeclareLaunchArgument(
        "motion_timeout_s",
        default_value="8.0",
        description="Maximum reference-search time per drive.",
    )
    test_drive_index_arg = DeclareLaunchArgument(
        "test_drive_index",
        default_value="0",
        description="Drive selected by the home_one service (0..3).",
    )
    pdo_cycle_ns_arg = DeclareLaunchArgument("pdo_cycle_ns", default_value="20000000")
    pdo_timeout_us_arg = DeclareLaunchArgument("pdo_timeout_us", default_value="5000")
    enable_dc_sync_arg = DeclareLaunchArgument("enable_dc_sync", default_value="false")
    ignore_spur_gear_arg = DeclareLaunchArgument(
        "ignore_spur_gear_in_csp",
        default_value="true",
        description=(
            "Temporary three-axis mode: skip Drive 3 Homing and keep it Disable Voltage in CSP."
        ),
    )

    bridge_node = Node(
        package="rascl_hardware_interface",
        executable="rascl_faulhaber_bridge.py",
        name="rascl_faulhaber_bridge",
        output="screen",
        parameters=[
            {
                "interface": LaunchConfiguration("interface"),
                "host": LaunchConfiguration("host"),
                "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
                "control_mode": "homing_csp",
                "slave_indices": [0, 1, 2, 3],
                "sdo_delay_s": 0.05,
                "motion_timeout_s": ParameterValue(
                    LaunchConfiguration("motion_timeout_s"), value_type=float
                ),
                "verbose": True,
                "pdo_cycle_ns": ParameterValue(
                    LaunchConfiguration("pdo_cycle_ns"), value_type=int
                ),
                "pdo_timeout_us": ParameterValue(
                    LaunchConfiguration("pdo_timeout_us"), value_type=int
                ),
                "enable_dc_sync": ParameterValue(
                    LaunchConfiguration("enable_dc_sync"), value_type=bool
                ),
                "ignore_spur_gear_in_csp": ParameterValue(
                    LaunchConfiguration("ignore_spur_gear_in_csp"), value_type=bool
                ),
                "homing_methods": [28, 28, 24, 24],
                "reference_inputs": [2, 2, 2, 1],
                "homing_offsets": [0, 0, 0, 0],
                "homing_search_speeds": [1000, 1000, 1000, 1000],
                "homing_zero_speeds": [200, 200, 200, 200],
                "homing_accelerations": [1000, 1000, 1000, 1000],
                "test_drive_index": ParameterValue(
                    LaunchConfiguration("test_drive_index"), value_type=int
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            interface_arg,
            host_arg,
            port_arg,
            motion_timeout_arg,
            test_drive_index_arg,
            pdo_cycle_ns_arg,
            pdo_timeout_us_arg,
            enable_dc_sync_arg,
            ignore_spur_gear_arg,
            bridge_node,
        ]
    )
