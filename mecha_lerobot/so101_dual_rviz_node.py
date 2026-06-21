#!/usr/bin/env python3
from __future__ import annotations

import json
import socket

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from visualization_msgs.msg import Marker

try:
    from .so101_kinematics import ALL_JOINT_NAMES, ARM_JOINT_NAMES, ee_pos
except Exception:
    from so101_kinematics import ALL_JOINT_NAMES, ARM_JOINT_NAMES, ee_pos


class SO101DualRvizNode(Node):
    def __init__(self):
        super().__init__("so101_dual_rviz_node")
        self.declare_parameter("udp_ip", "127.0.0.1")
        self.declare_parameter("udp_port", 5005)
        self.declare_parameter("fps", 60)
        self.declare_parameter("target_prefix", "target_")

        self.udp_ip = self.get_parameter("udp_ip").value
        self.udp_port = int(self.get_parameter("udp_port").value)
        self.fps = int(self.get_parameter("fps").value)
        self.target_prefix = str(self.get_parameter("target_prefix").value)

        self.latest_actual = [0.0] * 6
        self.latest_target = [0.0] * 6
        self.latest_target_xyz = None
        self.actual_trail: list[Point] = []
        self.target_path: list[Point] = []

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.target_marker_pub = self.create_publisher(Marker, "/so101/target_path", 10)
        self.actual_marker_pub = self.create_publisher(Marker, "/so101/actual_ee_trail", 10)
        self.ee_marker_pub = self.create_publisher(Marker, "/so101/current_ee", 10)

        # rqt_plot-friendly motor-angle topics.
        # All angle topics are published in radians. Use rqt_plot expressions like:
        #   /so101/target/shoulder_pan/data,/so101/actual/shoulder_pan/data
        self.target_angles_pub = self.create_publisher(Float64MultiArray, "/so101/target_motor_angles", 10)
        self.actual_angles_pub = self.create_publisher(Float64MultiArray, "/so101/actual_motor_angles", 10)
        self.error_angles_pub = self.create_publisher(Float64MultiArray, "/so101/motor_angle_error", 10)

        self.target_joint_pubs = {
            name: self.create_publisher(Float64, f"/so101/target/{name}", 10)
            for name in ALL_JOINT_NAMES
        }
        self.actual_joint_pubs = {
            name: self.create_publisher(Float64, f"/so101/actual/{name}", 10)
            for name in ALL_JOINT_NAMES
        }
        self.error_joint_pubs = {
            name: self.create_publisher(Float64, f"/so101/error/{name}", 10)
            for name in ALL_JOINT_NAMES
        }

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.setblocking(False)
        self.get_logger().info(f"Listening SO101 IK packets on UDP {self.udp_ip}:{self.udp_port}")
        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)

    def timer_callback(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except BlockingIOError:
                break
            except Exception as e:
                self.get_logger().warn(f"UDP receive error: {repr(e)}")
                break
            try:
                packet = json.loads(data.decode("utf-8"))
                actual = packet.get("actual_positions") or packet.get("positions")
                target = packet.get("target_positions")
                if actual and len(actual) >= 6:
                    self.latest_actual = [float(x) for x in actual[:6]]
                if target and len(target) >= 6:
                    self.latest_target = [float(x) for x in target[:6]]
                if packet.get("target_path"):
                    self.target_path = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in packet["target_path"]]
                if packet.get("target_xyz"):
                    p = packet["target_xyz"]
                    self.latest_target_xyz = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
            except Exception as e:
                self.get_logger().warn(f"Bad UDP packet: {repr(e)}")

        self.publish_joint_states()
        self.publish_motor_angle_topics()
        self.publish_markers()

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ALL_JOINT_NAMES) + [self.target_prefix + n for n in ALL_JOINT_NAMES]
        msg.position = [float(x) for x in self.latest_actual] + [float(x) for x in self.latest_target]
        self.joint_pub.publish(msg)


    def publish_motor_angle_topics(self):
        target = [float(x) for x in self.latest_target]
        actual = [float(x) for x in self.latest_actual]
        error = [a - t for a, t in zip(actual, target)]

        target_msg = Float64MultiArray()
        target_msg.data = target
        self.target_angles_pub.publish(target_msg)

        actual_msg = Float64MultiArray()
        actual_msg.data = actual
        self.actual_angles_pub.publish(actual_msg)

        error_msg = Float64MultiArray()
        error_msg.data = error
        self.error_angles_pub.publish(error_msg)

        for i, name in enumerate(ALL_JOINT_NAMES):
            msg = Float64()
            msg.data = target[i]
            self.target_joint_pubs[name].publish(msg)

            msg = Float64()
            msg.data = actual[i]
            self.actual_joint_pubs[name].publish(msg)

            msg = Float64()
            msg.data = error[i]
            self.error_joint_pubs[name].publish(msg)

    def base_marker(self, ns: str, marker_id: int, marker_type: int, frame_id: str = "base_link") -> Marker:
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = frame_id
        m.ns = ns
        m.id = marker_id
        m.type = marker_type
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def publish_markers(self):
        if self.target_path:
            m = self.base_marker("target_path", 0, Marker.LINE_STRIP)
            m.scale.x = 0.003
            m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.2; m.color.a = 1.0
            m.points = self.target_path + [self.target_path[0]]
            self.target_marker_pub.publish(m)

        try:
            p = ee_pos(self.latest_actual[:5])
            pt = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
            self.actual_trail.append(pt)
            self.actual_trail = self.actual_trail[-2000:]
        except Exception:
            pt = None

        if len(self.actual_trail) >= 2:
            m = self.base_marker("actual_ee_trail", 1, Marker.LINE_STRIP)
            m.scale.x = 0.0035
            m.color.r = 1.0; m.color.g = 0.9; m.color.b = 0.1; m.color.a = 1.0
            m.points = self.actual_trail
            self.actual_marker_pub.publish(m)

        if pt is not None:
            m = self.base_marker("actual_ee", 2, Marker.SPHERE)
            m.scale.x = m.scale.y = m.scale.z = 0.012
            m.color.r = 1.0; m.color.g = 0.9; m.color.b = 0.1; m.color.a = 1.0
            m.pose.position = pt
            self.ee_marker_pub.publish(m)

        if self.latest_target_xyz is not None:
            m = self.base_marker("target_ee", 3, Marker.SPHERE)
            m.scale.x = m.scale.y = m.scale.z = 0.010
            m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.2; m.color.a = 1.0
            m.pose.position = self.latest_target_xyz
            self.ee_marker_pub.publish(m)

    def destroy_node(self):
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SO101DualRvizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()