#!/usr/bin/env python3
"""One-way ROS 1 JointState sender for the isolated MH5 mirror link."""

import argparse
import json
import math
import socket
import threading
import time

import rospy
from sensor_msgs.msg import JointState

JOINTS = ("joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.50.20")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--topic", default="/joint_states")
    args = parser.parse_args()
    latest = {"payload": None}
    lock = threading.Lock()

    def callback(msg):
        indices = {name: i for i, name in enumerate(msg.name)}
        if any(name not in indices or indices[name] >= len(msg.position) for name in JOINTS):
            return
        position = [float(msg.position[indices[name]]) for name in JOINTS]
        if not all(math.isfinite(value) for value in position):
            return
        payload = {
            "schema": 1, "source": "fs100_ros1_read_only", "names": JOINTS,
            "position_rad": position, "source_sec": int(msg.header.stamp.secs),
            "source_nanosec": int(msg.header.stamp.nsecs), "sent_ns": time.time_ns(),
        }
        encoded = (json.dumps(payload, separators=(",", ":")) + "\n").encode()
        with lock:
            latest["payload"] = encoded

    rospy.init_node("mh5_joint_state_tcp_sender", anonymous=False)
    rospy.Subscriber(args.topic, JointState, callback, queue_size=1)
    while not rospy.is_shutdown():
        try:
            with socket.create_connection((args.host, args.port), timeout=3) as conn:
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                rospy.loginfo("connected to Isaac receiver %s:%d", args.host, args.port)
                last = None
                while not rospy.is_shutdown():
                    with lock:
                        payload = latest["payload"]
                    if payload is not None and payload != last:
                        conn.sendall(payload)
                        last = payload
                    time.sleep(0.005)
        except OSError as exc:
            rospy.logwarn_throttle(5, "Isaac receiver unavailable: %s", exc)
            time.sleep(1)


if __name__ == "__main__":
    main()
