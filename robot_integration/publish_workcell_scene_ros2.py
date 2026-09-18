#!/usr/bin/env python3
"""Publish reviewed fixed MH5 workcell geometry into MoveIt's planning scene."""
import time
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive

FRAME='base_link'

def pose(x,y,z):
    value=Pose(); value.position.x=x; value.position.y=y; value.position.z=z
    value.orientation.w=1.0; return value

def box(name,center,size):
    obj=CollisionObject(); obj.header.frame_id=FRAME; obj.id=name
    primitive=SolidPrimitive(); primitive.type=SolidPrimitive.BOX; primitive.dimensions=list(size)
    obj.primitives=[primitive]; obj.primitive_poses=[pose(*center)]; obj.operation=CollisionObject.ADD
    return obj

def cylinder(name,center,height,radius):
    obj=CollisionObject(); obj.header.frame_id=FRAME; obj.id=name
    primitive=SolidPrimitive(); primitive.type=SolidPrimitive.CYLINDER
    primitive.dimensions=[height,radius]
    obj.primitives=[primitive]; obj.primitive_poses=[pose(*center)]; obj.operation=CollisionObject.ADD
    return obj

def main():
    rclpy.init(); node=Node('mh5_fixed_workcell_scene')
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher=node.create_publisher(PlanningScene,'/planning_scene',qos)
    scene=PlanningScene(); scene.is_diff=True
    scene.world.collision_objects=[
        # Measured tabletop, reduced 10 mm at the top to avoid a numerical
        # contact with the robot base mounting plane at base_link Z=0.
        box('worktable',[0.600,0.000,-0.038],[2.000,1.200,0.056]),
        box('camera_frame_post',[0.410,0.620,0.2165],[0.040,0.040,2.043]),
        box('camera_frame_beam',[0.410,0.380,1.253],[0.030,0.600,0.030]),
        cylinder('opposing_mh5_keepout',[1.200,0.000,0.575],1.150,0.450),
        box('floor',[0.600,0.000,-0.855],[4.000,3.000,0.100]),
    ]
    deadline=time.monotonic()+5.0
    while publisher.get_subscription_count()==0 and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=0.1)
    if publisher.get_subscription_count()==0:
        raise RuntimeError('MoveIt planning_scene subscriber unavailable')
    for _ in range(5):
        publisher.publish(scene); rclpy.spin_once(node,timeout_sec=0.1)
    node.get_logger().info('published fixed objects: '+', '.join(
        obj.id for obj in scene.world.collision_objects))
    node.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
