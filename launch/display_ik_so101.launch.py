from __future__ import annotations

import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def as_bool(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def prefixed_urdf_text(
    urdf_text: str,
    prefix: str = "target_",
    target_rgba: str = "0.0 0.25 1.0 0.45",
) -> str:
    root = ET.fromstring(urdf_text)
    root.set("name", prefix + root.get("name", "robot"))

    target_material_name = prefix + "blue_material"

    # Root-level material 추가
    target_material = ET.Element("material", {"name": target_material_name})
    ET.SubElement(target_material, "color", {"rgba": target_rgba})
    root.insert(0, target_material)

    # 기존 root-level material도 target URDF 안에서는 파란색으로 변경
    for material in root.findall("material"):
        color = material.find("color")
        if color is None:
            color = ET.SubElement(material, "color")
        color.set("rgba", target_rgba)

    for link in root.findall("link"):
        link.set("name", prefix + link.get("name"))

        # target link visual material을 파란색으로 강제 변경
        for visual in link.findall("visual"):
            material = visual.find("material")
            if material is None:
                material = ET.SubElement(visual, "material")

            material.attrib.clear()
            material.set("name", target_material_name)

            for child in list(material):
                material.remove(child)

            ET.SubElement(material, "color", {"rgba": target_rgba})

    for joint in root.findall("joint"):
        joint.set("name", prefix + joint.get("name"))

        parent = joint.find("parent")
        child = joint.find("child")

        if parent is not None and parent.get("link"):
            parent.set("link", prefix + parent.get("link"))

        if child is not None and child.get("link"):
            child.set("link", prefix + child.get("link"))

    for transmission in root.findall("transmission"):
        if transmission.get("name"):
            transmission.set("name", prefix + transmission.get("name"))

        for joint in transmission.findall("joint"):
            if joint.get("name"):
                joint.set("name", prefix + joint.get("name"))

    return ET.tostring(root, encoding="unicode")


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("mecha_lerobot")
    prefix = LaunchConfiguration("target_prefix").perform(context)
    target_base_frame = prefix + "base_link"

    urdf_path = f"{pkg_share}/urdf/so101_new_calib.urdf"

    with open(urdf_path, "r") as f:
        actual_description = f.read()

    target_description = prefixed_urdf_text(actual_description, prefix)

    dry_run_value = LaunchConfiguration("dry_run").perform(context)
    loop_value = LaunchConfiguration("loop").perform(context)

    ik_cmd = [
        LaunchConfiguration("conda_python").perform(context),
        LaunchConfiguration("ik_script").perform(context),
        "--follower-port", LaunchConfiguration("follower_port").perform(context),
        "--follower-id", LaunchConfiguration("follower_id").perform(context),
        "--fps", LaunchConfiguration("fps").perform(context),
        "--udp-ip", LaunchConfiguration("udp_ip").perform(context),
        "--udp-port", LaunchConfiguration("udp_port").perform(context),
        "--csv", LaunchConfiguration("csv").perform(context),
        "--trajectory", LaunchConfiguration("trajectory").perform(context),
        "--n-points", LaunchConfiguration("n_points").perform(context),
        "--scale", LaunchConfiguration("scale").perform(context),
        "--center", LaunchConfiguration("center").perform(context),
        "--plane", LaunchConfiguration("plane").perform(context),
        "--robot-action-units", LaunchConfiguration("robot_action_units").perform(context),
        "--observation-units", LaunchConfiguration("observation_units").perform(context),
    ]

    target_csv = LaunchConfiguration("target_csv").perform(context)
    if target_csv:
        ik_cmd += ["--target-csv", target_csv]

    # Default is loop:=true, so the heart/path is drawn repeatedly until Ctrl+C.
    if as_bool(loop_value):
        ik_cmd += ["--loop"]

    if as_bool(dry_run_value):
        ik_cmd += ["--dry-run"]

    rviz_config = f"{pkg_share}/rviz/so101_ik_dual.rviz"

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="actual_robot_state_publisher",
            parameters=[
                {
                    "robot_description": actual_description,
                }
            ],
            remappings=[
                ("joint_states", "/joint_states"),
            ],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="target_robot_state_publisher",
            namespace="target",
            parameters=[
                {
                    "robot_description": target_description,
                }
            ],
            remappings=[
                ("joint_states", "/joint_states"),
            ],
            output="screen",
        ),
        # The prefixed target URDF has a separate root frame, target_base_link.
        # target_y_offset=0.0 makes the target robot overlap with the actual robot.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="target_base_static_tf",
            arguments=[
                "0",
                LaunchConfiguration("target_y_offset").perform(context),
                "0",
                "0",
                "0",
                "0",
                "base_link",
                target_base_frame,
            ],
            output="screen",
        ),
        Node(
            package="mecha_lerobot",
            executable="so101_dual_rviz_node",
            name="so101_dual_rviz_node",
            parameters=[
                {
                    "udp_ip": LaunchConfiguration("udp_ip").perform(context),
                    "udp_port": int(LaunchConfiguration("udp_port").perform(context)),
                    "fps": 60,
                    "target_prefix": prefix,
                }
            ],
            output="screen",
        ),
        ExecuteProcess(
            cmd=ik_cmd,
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d",
                rviz_config,
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("follower_port", default_value="/dev/follower"),
            DeclareLaunchArgument("follower_id", default_value="my_awesome_follower_arm"),
            DeclareLaunchArgument("fps", default_value="30"),
            DeclareLaunchArgument("udp_ip", default_value="127.0.0.1"),
            DeclareLaunchArgument("udp_port", default_value="5005"),
            DeclareLaunchArgument("csv", default_value="auto"),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument("target_prefix", default_value="target_"),
            DeclareLaunchArgument("target_y_offset", default_value="0.0"),
            DeclareLaunchArgument("trajectory", default_value="heart"),
            DeclareLaunchArgument("target_csv", default_value=""),
            DeclareLaunchArgument("n_points", default_value="200"),
            DeclareLaunchArgument("scale", default_value="0.05"),
            DeclareLaunchArgument("center", default_value="0.18,0.0,0.12"),
            DeclareLaunchArgument("plane", default_value="YZ"),
            DeclareLaunchArgument("loop", default_value="true"),
            DeclareLaunchArgument("robot_action_units", default_value="degrees"),
            DeclareLaunchArgument("observation_units", default_value="degrees"),
            DeclareLaunchArgument(
                "conda_python",
                default_value="/home/kjy/anaconda3/envs/lerobot/bin/python3",
            ),
            DeclareLaunchArgument(
                "ik_script",
                default_value="/home/kjy/ros2_ws/src/mecha_lerobot/mecha_lerobot/so101_ik_follower.py",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
