from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    rtde_frequency = LaunchConfiguration("rtde_frequency")
    dataset_root = LaunchConfiguration("dataset_root")
    color_width = LaunchConfiguration("color_width")
    color_height = LaunchConfiguration("color_height")
    color_fps = LaunchConfiguration("color_fps")
    teleop_state_file = LaunchConfiguration("teleop_state_file")
    teleop_state_timeout_sec = LaunchConfiguration("teleop_state_timeout_sec")
    python_executable = LaunchConfiguration("python_executable")

    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.102"),
        DeclareLaunchArgument("rtde_frequency", default_value="-1.0"),
        DeclareLaunchArgument("dataset_root", default_value="/home/ustc/UR/dataset"),
        DeclareLaunchArgument("color_width", default_value="640"),
        DeclareLaunchArgument("color_height", default_value="480"),
        DeclareLaunchArgument("color_fps", default_value="30"),
        DeclareLaunchArgument(
            "teleop_state_file",
            default_value="/tmp/ur_teleop_state.json",
        ),
        DeclareLaunchArgument("teleop_state_timeout_sec", default_value="1.0"),
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
                ["color_width:=", color_width],
                "-p",
                ["color_height:=", color_height],
                "-p",
                ["color_fps:=", color_fps],
                "-p",
                ["teleop_state_file:=", teleop_state_file],
                "-p",
                ["teleop_state_timeout_sec:=", teleop_state_timeout_sec],
            ],
            output="screen",
        ),
    ])
