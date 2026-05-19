from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    rtde_frequency = LaunchConfiguration("rtde_frequency")
    dataset_root = LaunchConfiguration("dataset_root")
    record_hz = LaunchConfiguration("record_hz")
    color_width = LaunchConfiguration("color_width")
    color_height = LaunchConfiguration("color_height")
    color_fps = LaunchConfiguration("color_fps")
    teleop_state_file = LaunchConfiguration("teleop_state_file")
    teleop_state_timeout_sec = LaunchConfiguration("teleop_state_timeout_sec")
    use_rtde_reader = LaunchConfiguration("use_rtde_reader")
    diagnostics_sec = LaunchConfiguration("diagnostics_sec")
    sample_warning_sec = LaunchConfiguration("sample_warning_sec")
    python_executable = LaunchConfiguration("python_executable")

    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.102"),
        DeclareLaunchArgument("rtde_frequency", default_value="-1.0"),
        DeclareLaunchArgument(
            "dataset_root",
            default_value=PathJoinSubstitution([
                EnvironmentVariable("UR_WORKSPACE", default_value="/data/jiaxuanLin/UR"),
                "dataset",
            ]),
        ),
        DeclareLaunchArgument("record_hz", default_value="30.0"),
        DeclareLaunchArgument("color_width", default_value="640"),
        DeclareLaunchArgument("color_height", default_value="480"),
        DeclareLaunchArgument("color_fps", default_value="30"),
        DeclareLaunchArgument(
            "teleop_state_file",
            default_value="/tmp/ur_teleop_state.json",
        ),
        DeclareLaunchArgument("teleop_state_timeout_sec", default_value="1.0"),
        DeclareLaunchArgument("use_rtde_reader", default_value="false"),
        DeclareLaunchArgument("diagnostics_sec", default_value="2.0"),
        DeclareLaunchArgument("sample_warning_sec", default_value="0.04"),
        DeclareLaunchArgument(
            "python_executable",
            default_value=PathJoinSubstitution([
                EnvironmentVariable("VIRTUAL_ENV", default_value="/usr"),
                "bin",
                "python3",
            ]),
        ),
        ExecuteProcess(
            cmd=[
                python_executable,
                "-m",
                "recorder.record",
                "--ros-args",
                "-r",
                "__node:=recorder",
                "-p",
                ["robot_ip:=", robot_ip],
                "-p",
                ["rtde_frequency:=", rtde_frequency],
                "-p",
                ["dataset_root:=", dataset_root],
                "-p",
                ["record_hz:=", record_hz],
                "-p",
                ["color_width:=", color_width],
                "-p",
                ["color_height:=", color_height],
                "-p",
                ["color_fps:=", color_fps],
                "-p",
                ["teleop_state_file:=", teleop_state_file],
                "-p",
                ["teleop_state_timeout_sec:=", teleop_state_timeout_sec],
                "-p",
                ["use_rtde_reader:=", use_rtde_reader],
                "-p",
                ["diagnostics_sec:=", diagnostics_sec],
                "-p",
                ["sample_warning_sec:=", sample_warning_sec],
            ],
            output="screen",
        ),
    ])
