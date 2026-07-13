"""Start only the FAULHABER bridge for safe reference-switch homing.

The ros2_control CSP client is deliberately not started here.  Homing changes
the CiA 402 mode and controlword through SDO, which must not race the cyclic PDO
thread.  Stop this launch after homing, then start ros2_control.launch.py.
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
                "control_mode": "profile",
                "slave_indices": [0, 1, 2, 3],
                "sdo_delay_s": 0.05,
                "motion_timeout_s": ParameterValue(
                    LaunchConfiguration("motion_timeout_s"), value_type=float
                ),
                "verbose": True,
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
            bridge_node,
        ]
    )
