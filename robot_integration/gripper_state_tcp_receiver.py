#!/usr/bin/env python3
"""Publish validated RG2-FT feedback locally as ROS 2 JointState."""
import argparse,json,math,socket,time,rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--bind",default="192.168.50.20")
    ap.add_argument("--port",type=int,default=8767); args=ap.parse_args(); rclpy.init()
    node=Node("rg2ft_state_tcp_receiver"); pub=node.create_publisher(JointState,"/real/gripper_joint_states",1)
    srv=socket.socket(); srv.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1); srv.bind((args.bind,args.port)); srv.listen(1); srv.settimeout(1)
    node.get_logger().info(f"read-only RG2-FT receiver listening on {args.bind}:{args.port}")
    try:
        while rclpy.ok():
            try: conn,peer=srv.accept()
            except socket.timeout: rclpy.spin_once(node,timeout_sec=0); continue
            node.get_logger().info(f"CYC gripper reader connected from {peer[0]}")
            with conn,conn.makefile("rb") as stream:
                for line in stream:
                    try:
                        rec=json.loads(line); value=float(rec["finger_joint_rad"]); width=int(rec["width_tenth_mm"])
                        if rec.get("schema")!=1 or not 0<=width<=1000 or not math.isfinite(value) or not 0<=value<=1.18: continue
                        if abs((1000-width)/1000*1.18-value)>1e-6 or time.time_ns()-int(rec["sent_ns"])>2_000_000_000: continue
                        msg=JointState(); msg.header.stamp=node.get_clock().now().to_msg(); msg.name=["finger_joint"]; msg.position=[value]; pub.publish(msg)
                    except (ValueError,KeyError,json.JSONDecodeError): continue
    finally: srv.close(); node.destroy_node(); rclpy.shutdown()
if __name__=="__main__": main()
