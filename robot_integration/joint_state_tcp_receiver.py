#!/usr/bin/env python3
"""Receive validated MH5 states over TCP and publish ROS 2 /real/joint_states."""

import argparse
import json
import math
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

JOINTS = ("joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind", default="192.168.50.20")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--topic", default="/real/joint_states")
    args = parser.parse_args()
    rclpy.init()
    node = Node("mh5_joint_state_tcp_receiver")
    publisher = node.create_publisher(
        JointState, args.topic, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    )
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.bind, args.port))
    server.listen(1)
    server.settimeout(1)
    node.get_logger().info(f"one-way receiver listening on {args.bind}:{args.port}")
    try:
        while rclpy.ok():
            try:
                conn, peer = server.accept()
            except socket.timeout:
                rclpy.spin_once(node, timeout_sec=0)
                continue
            node.get_logger().info(f"CYC sender connected from {peer[0]}")
            conn.settimeout(1)
            stream = conn.makefile("rb")
            try:
                while rclpy.ok():
                    line = stream.readline(16385)
                    if not line:
                        break
                    if len(line) > 16384:
                        raise ValueError("oversized frame")
                    record = json.loads(line)
                    positions = record.get("position_rad", [])
                    if record.get("schema") != 1 or tuple(record.get("names", ())) != JOINTS:
                        raise ValueError("invalid schema or joint order")
                    if len(positions) != 6 or not all(math.isfinite(float(v)) for v in positions):
                        raise ValueError("invalid joint positions")
                    age = (time.time_ns() - int(record["sent_ns"])) / 1e9
                    if age < -1 or age > 2:
                        continue
                    msg = JointState()
                    msg.header.stamp.sec = int(record["source_sec"])
                    msg.header.stamp.nanosec = int(record["source_nanosec"])
                    msg.name = list(JOINTS)
                    msg.position = [float(v) for v in positions]
                    publisher.publish(msg)
                    rclpy.spin_once(node, timeout_sec=0)
            except (OSError, ValueError, json.JSONDecodeError, KeyError) as exc:
                node.get_logger().warning(f"discarded CYC connection: {exc}")
            finally:
                stream.close()
                conn.close()
    finally:
        server.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
