#!/usr/bin/env python3

import json
import socket

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class SO101RvizNode(Node):
    def __init__(self):
        super().__init__("so101_rviz_node")

        self.declare_parameter("udp_ip", "127.0.0.1")
        self.declare_parameter("udp_port", 5005)
        self.declare_parameter("fps", 60)

        self.udp_ip = self.get_parameter("udp_ip").value
        self.udp_port = int(self.get_parameter("udp_port").value)
        self.fps = int(self.get_parameter("fps").value)

        self.joint_names = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
            "gripper",
        ]

        self.latest_positions = [0.0] * 6

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.setblocking(False)

        self.get_logger().info(
            f"Listening SO101 joint states on UDP {self.udp_ip}:{self.udp_port}"
        )

        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)

    def timer_callback(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            except Exception as e:
                self.get_logger().warn(f"UDP receive error: {repr(e)}")
                break

            try:
                packet = json.loads(data.decode("utf-8"))
                positions = packet.get("positions", [])

                if len(positions) >= 6:
                    self.latest_positions = [float(x) for x in positions[:6]]

            except Exception as e:
                self.get_logger().warn(f"Bad UDP packet: {repr(e)}")

        self.publish_joint_states(self.latest_positions)

    def publish_joint_states(self, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(x) for x in positions]
        self.joint_pub.publish(msg)

    def destroy_node(self):
        try:
            self.sock.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SO101RvizNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()