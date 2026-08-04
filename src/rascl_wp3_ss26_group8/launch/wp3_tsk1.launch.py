"""Launch WP3 Task 1 single-target minimum-jerk executor.

By default this launch file starts only the WP3 node.  Set start_robot:=true to
also launch the existing rascl_description ros2_control stack.  The same WP3
node works with fake hardware and real hardware because it only publishes ROS
joint position commands; the selected lower layer is controlled by
use_fake_hardware in rascl_description.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_robot = LaunchConfiguration("start_robot")
    start_bridge = LaunchConfiguration("start_bridge")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    interface = LaunchConfiguration("interface")
    control_mode = LaunchConfiguration("control_mode")
    controller_config = LaunchConfiguration("controller_config")
    lowerarm_direction = LaunchConfiguration("lowerarm_direction")
    shoulder_home_offset_counts = LaunchConfiguration("shoulder_home_offset_counts")
    upperarm_home_offset_counts = LaunchConfiguration("upperarm_home_offset_counts")
    lowerarm_home_offset_counts = LaunchConfiguration("lowerarm_home_offset_counts")
    spur_gear_home_offset_counts = LaunchConfiguration("spur_gear_home_offset_counts")

    target_x = LaunchConfiguration("target_x")
    target_y = LaunchConfiguration("target_y")
    target_z = LaunchConfiguration("target_z")
    apply_board_xy_compensation = LaunchConfiguration("apply_board_xy_compensation")
    duration = LaunchConfiguration("duration")
    rate_hz = LaunchConfiguration("rate_hz")
    execute = LaunchConfiguration("execute")
    save_csv = LaunchConfiguration("save_csv")
    input_csv = LaunchConfiguration("input_csv")
    output_csv = LaunchConfiguration("output_csv")
    start_delay_s = LaunchConfiguration("start_delay_s")

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rascl_description"),
                "launch",
                "ros2_control.launch.py",
            ])
        ),
        condition=IfCondition(start_robot),
        launch_arguments={
            "use_fake_hardware": use_fake_hardware,
            "interface": interface,
            "control_mode": control_mode,
            "controller_config": controller_config,
            "start_bridge": start_bridge,
            "lowerarm_direction": lowerarm_direction,
            "shoulder_home_offset_counts": shoulder_home_offset_counts,
            "upperarm_home_offset_counts": upperarm_home_offset_counts,
            "lowerarm_home_offset_counts": lowerarm_home_offset_counts,
            "spur_gear_home_offset_counts": spur_gear_home_offset_counts,
        }.items(),
    )

    wp3_node = Node(
        package="rascl_wp3_ss26_group8",
        executable="wp3_tsk1",
        name="wp3_tsk1",
        output="screen",
        parameters=[
            {
                "target_x": target_x,
                "target_y": target_y,
                "target_z": target_z,
                "apply_board_xy_compensation": apply_board_xy_compensation,
                "duration": duration,
                "rate_hz": rate_hz,
                "execute": execute,
                "save_csv": save_csv,
                "input_csv": input_csv,
                "output_csv": output_csv,
            }
        ],
    )

    delayed_wp3_node = TimerAction(
        period=start_delay_s,
        actions=[wp3_node],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_robot",
            default_value="false",
            description="If true, also launch rascl_description ros2_control stack.",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Passed to rascl_description when start_robot is true.",
        ),
        DeclareLaunchArgument(
            "interface",
            default_value="robot_interface",
            description="EtherCAT network interface for real hardware.",
        ),
        DeclareLaunchArgument(
            "control_mode",
            default_value="csp",
            description="Lower-level control mode passed to rascl_description: profile or csp.",
        ),
        DeclareLaunchArgument(
            "controller_config",
            default_value="controllers_csp.yaml",
            description="Controller YAML passed to rascl_description. Use controllers_csp.yaml for CSP tests.",
        ),
        DeclareLaunchArgument(
            "start_bridge",
            default_value="false",
            description="Passed to rascl_description; false reuses the Homing bridge.",
        ),
        DeclareLaunchArgument(
            "lowerarm_direction",
            default_value="1",
            description="Drive 2 encoder-to-URDF sign; pair +1 with home offset -802816.",
        ),
        DeclareLaunchArgument("shoulder_home_offset_counts", default_value="0"),
        DeclareLaunchArgument("upperarm_home_offset_counts", default_value="-802816"),
        DeclareLaunchArgument("lowerarm_home_offset_counts", default_value="-802816"),
        DeclareLaunchArgument("spur_gear_home_offset_counts", default_value="0"),
        DeclareLaunchArgument("target_x", default_value="0.25", description="TCP target x in base_link [m]."),
        DeclareLaunchArgument("target_y", default_value="0.00", description="TCP target y in base_link [m]."),
        DeclareLaunchArgument("target_z", default_value="0.08", description="TCP target z in base_link [m]."),
        DeclareLaunchArgument(
            "apply_board_xy_compensation",
            default_value="false",
            description="Apply the measured fixed-board affine XY correction before IK.",
        ),
        DeclareLaunchArgument("duration", default_value="4.0", description="Minimum-jerk duration [s]."),
        DeclareLaunchArgument("rate_hz", default_value="50.0", description="Command sample rate [Hz]."),
        DeclareLaunchArgument(
            "execute",
            default_value="false",
            description="If false, only plan and save CSV. Set true to publish commands.",
        ),
        DeclareLaunchArgument("save_csv", default_value="true", description="Save generated trajectory CSV."),
        DeclareLaunchArgument(
            "input_csv",
            default_value="",
            description="Optional offline joint trajectory CSV to validate and execute without replanning.",
        ),
        DeclareLaunchArgument(
            "output_csv",
            default_value=PathJoinSubstitution([
                FindPackageShare("rascl_wp3_ss26_group8"),
                "trajectories",
                "task1_output.csv",
            ]),
            description="Path for generated trajectory CSV.",
        ),
        DeclareLaunchArgument(
            "start_delay_s",
            default_value="6.0",
            description="Delay before starting the WP3 node when launching robot stack together.",
        ),
        robot_launch,
        delayed_wp3_node,
    ])
