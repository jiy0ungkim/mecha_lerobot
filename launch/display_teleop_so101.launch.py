from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    follower_port = LaunchConfiguration("follower_port")
    leader_port = LaunchConfiguration("leader_port")
    follower_id = LaunchConfiguration("follower_id")
    leader_id = LaunchConfiguration("leader_id")
    fps = LaunchConfiguration("fps")
    udp_ip = LaunchConfiguration("udp_ip")
    udp_port = LaunchConfiguration("udp_port")
    degrees_to_radians = LaunchConfiguration("degrees_to_radians")

    urdf_file = PathJoinSubstitution([
        FindPackageShare("mecha"),
        "urdf",
        "so101_new_calib.urdf",
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare("mecha"),
        "rviz",
        "so101.rviz",
    ])

    robot_description = ParameterValue(
        Command(["cat ", urdf_file]),
        value_type=str,
    )

    conda_python = "/home/kjy/anaconda3/envs/lerobot/bin/python3"
    teleop_script = "/home/kjy/ros2_ws/src/mecha/mecha/so101_teleop.py"

    common_teleop_cmd = [
        conda_python,
        teleop_script,

        "--follower-port", follower_port,
        "--leader-port", leader_port,
        "--follower-id", follower_id,
        "--leader-id", leader_id,
        "--fps", fps,
        "--udp-ip", udp_ip,
        "--udp-port", udp_port,
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "follower_port",
            default_value="/dev/follower",
            description="SO101 follower arm serial port",
        ),
        DeclareLaunchArgument(
            "leader_port",
            default_value="/dev/leader",
            description="SO101 leader arm serial port",
        ),
        DeclareLaunchArgument(
            "follower_id",
            default_value="my_awesome_follower_arm",
            description="LeRobot follower calibration id",
        ),
        DeclareLaunchArgument(
            "leader_id",
            default_value="my_awesome_leader_arm",
            description="LeRobot leader calibration id",
        ),
        DeclareLaunchArgument(
            "fps",
            default_value="30",
            description="LeRobot teleoperation FPS",
        ),
        DeclareLaunchArgument(
            "udp_ip",
            default_value="127.0.0.1",
            description="UDP IP for joint state transfer",
        ),
        DeclareLaunchArgument(
            "udp_port",
            default_value="5005",
            description="UDP port for joint state transfer",
        ),
        DeclareLaunchArgument(
            "degrees_to_radians",
            default_value="true",
            description="Set true if LeRobot joint values are degrees",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {
                    "robot_description": robot_description,
                }
            ],
            output="screen",
        ),

        Node(
            package="mecha",
            executable="so101_rviz_node",
            name="so101_rviz_node",
            parameters=[
                {
                    "udp_ip": udp_ip,
                    "udp_port": udp_port,
                    "fps": 60,
                }
            ],
            output="screen",
        ),

        ExecuteProcess(
            cmd=common_teleop_cmd,
            condition=UnlessCondition(degrees_to_radians),
            output="screen",
        ),

        ExecuteProcess(
            cmd=common_teleop_cmd + ["--degrees-to-radians"],
            condition=IfCondition(degrees_to_radians),
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])