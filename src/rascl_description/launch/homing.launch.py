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
    center_reference_switch_home_arg = DeclareLaunchArgument(
        "center_reference_switch_home",
        default_value="true",
        description=(
            "After finding the first reference-switch edge, cross the active interval, "
            "return to its midpoint, and define that midpoint as Home."
        ),
    )
    homing_center_timeout_arg = DeclareLaunchArgument(
        "homing_center_timeout_s",
        default_value="30.0",
        description="Maximum time for each switch-interval scan or centering move.",
    )
    homing_center_max_scan_arg = DeclareLaunchArgument(
        "homing_center_max_scan_counts",
        default_value="100000",
        description=(
            "Maximum travel past the first Home edge while looking for the switch-off edge."
        ),
    )
    homing_center_release_samples_arg = DeclareLaunchArgument(
        "homing_center_release_confirm_samples",
        default_value="3",
        description="Consecutive inactive reference-input samples required at the second edge.",
    )
    homing_center_poll_interval_arg = DeclareLaunchArgument(
        "homing_center_poll_interval_s",
        default_value="0.02",
        description="Reference-input polling interval during the centered-Home scan.",
    )
    homing_center_tolerance_arg = DeclareLaunchArgument(
        "homing_center_position_tolerance_counts",
        default_value="100",
        description="Allowed Profile Position error at the exit edge and interval midpoint.",
    )
    test_drive_index_arg = DeclareLaunchArgument(
        "test_drive_index",
        default_value="0",
        description="Drive selected by the home_one service (0..3).",
    )
    pdo_cycle_ns_arg = DeclareLaunchArgument("pdo_cycle_ns", default_value="20000000")
    pdo_timeout_us_arg = DeclareLaunchArgument("pdo_timeout_us", default_value="5000")
    enable_dc_sync_arg = DeclareLaunchArgument("enable_dc_sync", default_value="false")
    csp_torque_limit_arg = DeclareLaunchArgument(
        "csp_torque_limit_per_mille",
        default_value="1000",
        description=(
            "Symmetric CSP directional torque limit for Drives 0-3; 1000 is rated."
        ),
    )
    clear_limit_switch_mappings_arg = DeclareLaunchArgument(
        "clear_limit_switch_mappings_for_csp",
        default_value="true",
        description=(
            "At CSP handoff, clear and verify volatile 0x2310:01/:02 "
            "lower/upper input mappings while preserving Homing reference, "
            "polarity, and position limits."
        ),
    )
    drive2_following_error_window_arg = DeclareLaunchArgument(
        "drive2_following_error_window_counts",
        default_value="25000",
        description="Drive 2 CSP following-error window in raw encoder counts.",
    )
    drive2_following_error_timeout_arg = DeclareLaunchArgument(
        "drive2_following_error_timeout_ms",
        default_value="250",
        description="Drive 2 CSP following-error timeout in milliseconds.",
    )
    csp_stall_error_arg = DeclareLaunchArgument(
        "csp_stall_error_counts",
        default_value="25000",
        description="Command/feedback error that arms live CSP stall diagnostics.",
    )
    csp_stall_progress_arg = DeclareLaunchArgument(
        "csp_stall_progress_counts",
        default_value="100",
        description="Minimum encoder progress that resets the CSP stall timer.",
    )
    csp_stall_timeout_arg = DeclareLaunchArgument(
        "csp_stall_timeout_ms",
        default_value="500",
        description="No-progress time before a staged drive diagnostic snapshot starts.",
    )
    ignore_spur_gear_arg = DeclareLaunchArgument(
        "ignore_spur_gear_in_csp",
        default_value="false",
        description=(
            "Emergency fallback only: skip Drive 3 Homing and keep it Disable Voltage in CSP."
        ),
    )
    skip_spur_homing_arg = DeclareLaunchArgument(
        "skip_spur_gear_homing",
        default_value="true",
        description=(
            "Skip Drive 3 sensor search. After Drives 0-2 Home, move Drive 3 by the "
            "configured relative increment and set the reached position to zero."
        ),
    )
    spur_reference_delta_arg = DeclareLaunchArgument(
        "spur_gear_reference_delta_counts",
        default_value="50000",
        description="Drive 3 relative move after Drives 0-2 Home and before Method 37 zeroing.",
    )
    spur_reference_timeout_arg = DeclareLaunchArgument(
        "spur_gear_reference_timeout_s",
        default_value="30.0",
        description="Timeout for the Drive 3 reference move and current-position zeroing.",
    )
    spur_reference_tolerance_arg = DeclareLaunchArgument(
        "spur_gear_reference_tolerance_counts",
        default_value="100",
        description="Allowed endpoint error for the Drive 3 reference move.",
    )
    spur_reference_velocity_arg = DeclareLaunchArgument(
        "spur_gear_reference_profile_velocity",
        default_value="3000",
        description="Profile Position velocity for the Drive 3 reference move [counts/s].",
    )
    spur_reference_acceleration_arg = DeclareLaunchArgument(
        "spur_gear_reference_profile_acceleration",
        default_value="1000",
        description="Profile Position acceleration for the Drive 3 reference move.",
    )
    spur_reference_deceleration_arg = DeclareLaunchArgument(
        "spur_gear_reference_profile_deceleration",
        default_value="1000",
        description="Profile Position deceleration for the Drive 3 reference move.",
    )
    spur_reference_following_error_confirm_arg = DeclareLaunchArgument(
        "spur_gear_reference_following_error_confirm_s",
        default_value="0.30",
        description=(
            "Time a Drive 3 following-error indication must remain active before the "
            "reference move is rejected."
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
                "center_reference_switch_home": ParameterValue(
                    LaunchConfiguration("center_reference_switch_home"), value_type=bool
                ),
                "homing_center_timeout_s": ParameterValue(
                    LaunchConfiguration("homing_center_timeout_s"), value_type=float
                ),
                "homing_center_max_scan_counts": ParameterValue(
                    LaunchConfiguration("homing_center_max_scan_counts"), value_type=int
                ),
                "homing_center_release_confirm_samples": ParameterValue(
                    LaunchConfiguration("homing_center_release_confirm_samples"),
                    value_type=int,
                ),
                "homing_center_poll_interval_s": ParameterValue(
                    LaunchConfiguration("homing_center_poll_interval_s"), value_type=float
                ),
                "homing_center_position_tolerance_counts": ParameterValue(
                    LaunchConfiguration("homing_center_position_tolerance_counts"),
                    value_type=int,
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
                "csp_torque_limit_per_mille": ParameterValue(
                    LaunchConfiguration("csp_torque_limit_per_mille"), value_type=int
                ),
                "clear_limit_switch_mappings_for_csp": ParameterValue(
                    LaunchConfiguration("clear_limit_switch_mappings_for_csp"),
                    value_type=bool,
                ),
                "drive2_following_error_window_counts": ParameterValue(
                    LaunchConfiguration("drive2_following_error_window_counts"), value_type=int
                ),
                "drive2_following_error_timeout_ms": ParameterValue(
                    LaunchConfiguration("drive2_following_error_timeout_ms"), value_type=int
                ),
                "csp_stall_error_counts": ParameterValue(
                    LaunchConfiguration("csp_stall_error_counts"), value_type=int
                ),
                "csp_stall_progress_counts": ParameterValue(
                    LaunchConfiguration("csp_stall_progress_counts"), value_type=int
                ),
                "csp_stall_timeout_ms": ParameterValue(
                    LaunchConfiguration("csp_stall_timeout_ms"), value_type=int
                ),
                "ignore_spur_gear_in_csp": ParameterValue(
                    LaunchConfiguration("ignore_spur_gear_in_csp"), value_type=bool
                ),
                "skip_spur_gear_homing": ParameterValue(
                    LaunchConfiguration("skip_spur_gear_homing"), value_type=bool
                ),
                "spur_gear_reference_delta_counts": ParameterValue(
                    LaunchConfiguration("spur_gear_reference_delta_counts"), value_type=int
                ),
                "spur_gear_reference_timeout_s": ParameterValue(
                    LaunchConfiguration("spur_gear_reference_timeout_s"), value_type=float
                ),
                "spur_gear_reference_tolerance_counts": ParameterValue(
                    LaunchConfiguration("spur_gear_reference_tolerance_counts"), value_type=int
                ),
                "spur_gear_reference_profile_velocity": ParameterValue(
                    LaunchConfiguration("spur_gear_reference_profile_velocity"), value_type=int
                ),
                "spur_gear_reference_profile_acceleration": ParameterValue(
                    LaunchConfiguration("spur_gear_reference_profile_acceleration"), value_type=int
                ),
                "spur_gear_reference_profile_deceleration": ParameterValue(
                    LaunchConfiguration("spur_gear_reference_profile_deceleration"), value_type=int
                ),
                "spur_gear_reference_following_error_confirm_s": ParameterValue(
                    LaunchConfiguration("spur_gear_reference_following_error_confirm_s"),
                    value_type=float,
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
            center_reference_switch_home_arg,
            homing_center_timeout_arg,
            homing_center_max_scan_arg,
            homing_center_release_samples_arg,
            homing_center_poll_interval_arg,
            homing_center_tolerance_arg,
            test_drive_index_arg,
            pdo_cycle_ns_arg,
            pdo_timeout_us_arg,
            enable_dc_sync_arg,
            csp_torque_limit_arg,
            clear_limit_switch_mappings_arg,
            drive2_following_error_window_arg,
            drive2_following_error_timeout_arg,
            csp_stall_error_arg,
            csp_stall_progress_arg,
            csp_stall_timeout_arg,
            ignore_spur_gear_arg,
            skip_spur_homing_arg,
            spur_reference_delta_arg,
            spur_reference_timeout_arg,
            spur_reference_tolerance_arg,
            spur_reference_velocity_arg,
            spur_reference_acceleration_arg,
            spur_reference_deceleration_arg,
            spur_reference_following_error_confirm_arg,
            bridge_node,
        ]
    )
