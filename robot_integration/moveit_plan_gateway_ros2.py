#!/usr/bin/env python3
"""CYC ROS 2 joint-goal planning gateway. Plans only; never executes."""
import json, math, socket, threading, time
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan, GetPositionIK, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS=('joint_1_s','joint_2_l','joint_3_u','joint_4_r','joint_5_b','joint_6_t')
HOST='192.168.50.10'; PORT=8770
# grasp_link is tool0 translated 0.215 m and rotated Ry(-90 deg), so the link's
# own +X axis is the approach direction. A top-down grasp therefore needs
# grasp_link +X aligned with base_link -Z.
TIP_LINK='grasp_link'; PLANNING_FRAME='base_link'

def top_down_matrix(yaw):
    """Rotation whose +X is straight down and whose +Z is the yaw direction."""
    c,s=math.cos(float(yaw)),math.sin(float(yaw))
    x_axis=(0.0,0.0,-1.0); z_axis=(c,s,0.0); y_axis=(-s,c,0.0)
    return [[x_axis[i],y_axis[i],z_axis[i]] for i in range(3)]

def pick_nearest(solutions,reference):
    """Choose the joint solution with the smallest worst-axis move.

    /compute_ik returns *a* solution, not the nearest one, so an equivalent
    grasp can arrive with the wrist swung most of a turn. Callers hand in the
    half-turn-equivalent candidates and this keeps the tamest one.
    """
    best=None; best_cost=None
    for solution in solutions:
        if solution is None: continue
        cost=max(abs(float(a)-float(b)) for a,b in zip(solution,reference))
        if best_cost is None or cost<best_cost: best,best_cost=solution,cost
    if best is None: raise RuntimeError('no IK solution to choose from')
    return best,best_cost

def matrix_quat(m):
    """Rotation matrix to xyzw quaternion via the numerically stable branch."""
    trace=m[0][0]+m[1][1]+m[2][2]
    if trace>0.0:
        d=math.sqrt(trace+1.0)*2.0
        w=0.25*d; x=(m[2][1]-m[1][2])/d; y=(m[0][2]-m[2][0])/d; z=(m[1][0]-m[0][1])/d
    elif m[0][0]>m[1][1] and m[0][0]>m[2][2]:
        d=math.sqrt(1.0+m[0][0]-m[1][1]-m[2][2])*2.0
        w=(m[2][1]-m[1][2])/d; x=0.25*d; y=(m[0][1]+m[1][0])/d; z=(m[0][2]+m[2][0])/d
    elif m[1][1]>m[2][2]:
        d=math.sqrt(1.0+m[1][1]-m[0][0]-m[2][2])*2.0
        w=(m[0][2]-m[2][0])/d; x=(m[0][1]+m[1][0])/d; y=0.25*d; z=(m[1][2]+m[2][1])/d
    else:
        d=math.sqrt(1.0+m[2][2]-m[0][0]-m[1][1])*2.0
        w=(m[1][0]-m[0][1])/d; x=(m[0][2]+m[2][0])/d; y=(m[1][2]+m[2][1])/d; z=0.25*d
    n=math.sqrt(x*x+y*y+z*z+w*w)
    return (x/n,y/n,z/n,w/n)

class Planner(Node):
    def __init__(self):
        super().__init__('mh5_moveit_plan_gateway')
        self.positions=None; self.stamp=0.0
        self.create_subscription(JointState,'/joint_states',self.joint_cb,10)
        self.client=self.create_client(GetMotionPlan,'/plan_kinematic_path')
        self.validity_client=self.create_client(GetStateValidity,'/check_state_validity')
        self.ik_client=self.create_client(GetPositionIK,'/compute_ik')
    def joint_cb(self,msg):
        idx={name:i for i,name in enumerate(msg.name)}
        if all(name in idx and idx[name]<len(msg.position) for name in JOINTS):
            values=tuple(float(msg.position[idx[name]]) for name in JOINTS)
            if all(math.isfinite(v) for v in values): self.positions,self.stamp=values,time.monotonic()
    def check_validity(self, positions, label):
        if not self.validity_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('MoveIt state-validity service unavailable')
        request=GetStateValidity.Request(); request.group_name='manipulator'
        request.robot_state.joint_state.name=list(JOINTS)
        request.robot_state.joint_state.position=[float(v) for v in positions]
        future=self.validity_client.call_async(request)
        deadline=time.monotonic()+3.0
        while not future.done() and time.monotonic()<deadline: time.sleep(0.01)
        if not future.done() or future.result() is None:
            raise RuntimeError('MoveIt state-validity check timed out for '+label)
        response=future.result()
        if not response.valid:
            contacts=[]
            for contact in response.contacts[:8]:
                pair=f'{contact.contact_body_1}<->{contact.contact_body_2}'
                if pair not in contacts: contacts.append(pair)
            detail=', '.join(contacts) if contacts else 'collision or constraint violation'
            raise RuntimeError(f'{label} state is invalid: {detail}')

    def _await(self,future,timeout,label):
        deadline=time.monotonic()+timeout
        while not future.done() and time.monotonic()<deadline: time.sleep(0.01)
        if not future.done() or future.result() is None:
            raise RuntimeError(label+' timed out')
        return future.result()

    def solve_ik(self,position,orientation_xyzw,seed=None):
        """Cartesian grasp_link pose to a collision-free joint solution."""
        if self.positions is None or time.monotonic()-self.stamp>0.5:
            raise RuntimeError('joint feedback is stale')
        if len(position)!=3 or not all(math.isfinite(float(v)) for v in position):
            raise ValueError('position must contain three finite metres')
        if len(orientation_xyzw)!=4 or not all(math.isfinite(float(v)) for v in orientation_xyzw):
            raise ValueError('orientation must contain four finite quaternion terms')
        if not self.ik_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('MoveIt IK service unavailable')
        seed=tuple(float(v) for v in seed) if seed else self.positions
        if len(seed)!=6: raise ValueError('seed must contain six radians')
        request=GetPositionIK.Request(); ik=request.ik_request
        ik.group_name='manipulator'; ik.ik_link_name=TIP_LINK
        ik.avoid_collisions=True; ik.timeout=Duration(sec=1,nanosec=0)
        ik.robot_state.joint_state.name=list(JOINTS)
        ik.robot_state.joint_state.position=[float(v) for v in seed]
        target=PoseStamped(); target.header.frame_id=PLANNING_FRAME
        target.pose.position.x,target.pose.position.y,target.pose.position.z=(float(v) for v in position)
        (target.pose.orientation.x,target.pose.orientation.y,
         target.pose.orientation.z,target.pose.orientation.w)=(float(v) for v in orientation_xyzw)
        ik.pose_stamped=target
        response=self._await(self.ik_client.call_async(request),4.0,'MoveIt IK')
        if response.error_code.val!=MoveItErrorCodes.SUCCESS:
            raise RuntimeError('MoveIt IK found no collision-free solution, error_code='
                               +str(response.error_code.val))
        order={name:i for i,name in enumerate(response.solution.joint_state.name)}
        if any(name not in order for name in JOINTS):
            raise RuntimeError('IK solution is missing arm joints')
        solution=tuple(float(response.solution.joint_state.position[order[name]]) for name in JOINTS)
        if not all(math.isfinite(v) for v in solution):
            raise RuntimeError('IK solution contains non-finite joint values')
        return solution

    def plan_pose(self,position,orientation_xyzw,yaw=None,seed=None,start=None,
                  half_turn=False):
        """Plan to a Cartesian grasp_link pose. Still plan-only.

        A top-down grasp is unchanged by a half turn about the approach axis, so
        with half_turn set both yaw and yaw+pi are solved and the solution that
        moves the wrist least is used.
        """
        reference=tuple(seed) if seed else (tuple(start) if start else self.positions)
        if reference is None: raise RuntimeError('joint feedback is stale')
        chosen_yaw=None
        if orientation_xyzw is not None:
            solution=self.solve_ik(position,orientation_xyzw,reference)
            cost=max(abs(a-b) for a,b in zip(solution,reference))
        else:
            base=0.0 if yaw is None else float(yaw)
            candidates=[base,base+math.pi] if half_turn else [base]
            solutions=[]; errors=[]
            for candidate in candidates:
                quat=matrix_quat(top_down_matrix(candidate))
                try: solutions.append((candidate,self.solve_ik(position,quat,reference)))
                except RuntimeError as exc: errors.append(str(exc))
            if not solutions: raise RuntimeError('; '.join(errors) or 'IK failed')
            best,cost=pick_nearest([item[1] for item in solutions],reference)
            chosen_yaw=next(c for c,item in solutions if item is best)
            solution=best
            orientation_xyzw=matrix_quat(top_down_matrix(chosen_yaw))
        result=self.plan(solution,start)
        result['ik_solution_rad']=list(solution)
        result['ik_worst_axis_move_rad']=float(cost)
        if chosen_yaw is not None:
            result['chosen_yaw_rad']=float(chosen_yaw)
            result['yaw_candidates_tried']=len(candidates)
        result['requested_position_m']=[float(v) for v in position]
        result['requested_orientation_xyzw']=[float(v) for v in orientation_xyzw]
        return result

    def plan(self,target,start=None):
        if self.positions is None or time.monotonic()-self.stamp>0.5:
            raise RuntimeError('joint feedback is stale')
        if len(target)!=6 or not all(math.isfinite(float(v)) for v in target):
            raise ValueError('target must contain six finite radians')
        target=tuple(float(v) for v in target)
        # A caller-supplied start chains virtual grasp stages together. It is
        # plan-only by construction: the execution gateway independently
        # rejects any trajectory whose first waypoint is not fresh physical
        # feedback, so a synthetic start can never reach the FS100.
        virtual_start=start is not None
        if virtual_start:
            if len(start)!=6 or not all(math.isfinite(float(v)) for v in start):
                raise ValueError('start must contain six finite radians')
            start=tuple(float(v) for v in start)
        else:
            start=self.positions
        self.check_validity(start,'start')
        self.check_validity(target,'goal')
        if not self.client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('MoveIt planning service unavailable')
        request=GetMotionPlan.Request(); m=request.motion_plan_request
        m.group_name='manipulator'; m.num_planning_attempts=5; m.allowed_planning_time=3.0
        m.max_velocity_scaling_factor=0.1; m.max_acceleration_scaling_factor=0.1
        m.start_state.joint_state.name=list(JOINTS); m.start_state.joint_state.position=list(start)
        constraint=Constraints()
        for name,value in zip(JOINTS,target):
            joint=JointConstraint(); joint.joint_name=name; joint.position=float(value)
            joint.tolerance_above=0.001; joint.tolerance_below=0.001; joint.weight=1.0
            constraint.joint_constraints.append(joint)
        m.goal_constraints=[constraint]
        future=self.client.call_async(request)
        deadline=time.monotonic()+5.0
        while not future.done() and time.monotonic()<deadline: time.sleep(0.01)
        if not future.done() or future.result() is None: raise RuntimeError('MoveIt planning timed out')
        response=future.result().motion_plan_response
        if response.error_code.val!=MoveItErrorCodes.SUCCESS:
            raise RuntimeError('MoveIt rejected plan, error_code='+str(response.error_code.val))
        trajectory=response.trajectory.joint_trajectory
        order={name:i for i,name in enumerate(trajectory.joint_names)}
        if any(name not in order for name in JOINTS): raise RuntimeError('planned joint order is incomplete')
        raw=[]
        for point in trajectory.points:
            sec=float(point.time_from_start.sec)+float(point.time_from_start.nanosec)/1e9
            raw.append(([float(point.positions[order[name]]) for name in JOINTS],sec))
        if len(raw)<2: raise RuntimeError('MoveIt returned an empty trajectory')
        # Preserve MoveIt's geometric path but use a conservative five-second
        # execution window. ROS-Industrial requires a positive first stamp.
        raw_end=max(raw[-1][1],1e-6); points=[]
        for positions,stamp in raw:
            normalized=1.0+4.0*(stamp/raw_end)
            points.append({'positions':positions,'time_from_start':normalized})
        return {'ok':True,'planner':'MoveIt/OMPL','start':list(start),
                'virtual_start':bool(virtual_start),
                'points':points,'planning_time_s':float(response.planning_time),
                'execution_duration_s':5.0}

def main():
    rclpy.init(); node=Planner(); stop=threading.Event()
    def serve():
        server=socket.socket(); server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        server.bind((HOST,PORT)); server.listen(2); server.settimeout(1)
        node.get_logger().info(f'plan-only gateway listening on {HOST}:{PORT}')
        try:
            while rclpy.ok() and not stop.is_set():
                try: connection,_=server.accept()
                except socket.timeout: continue
                with connection:
                    try:
                        raw=connection.makefile('r').readline(65537)
                        request=json.loads(raw)
                        if 'target_pose' in request:
                            pose=request['target_pose'] or {}
                            reply=node.plan_pose(pose.get('position_m',()),
                                                 pose.get('orientation_xyzw'),
                                                 pose.get('yaw_rad'),
                                                 request.get('seed_rad'),
                                                 request.get('start_rad'),
                                                 bool(pose.get('half_turn_equivalent',
                                                               True)))
                        else:
                            reply=node.plan(request.get('target_rad',()),
                                            request.get('start_rad'))
                    except Exception as exc: reply={'ok':False,'error':str(exc)}
                    connection.sendall((json.dumps(reply,separators=(',',':'))+'\n').encode())
        finally: server.close()
    thread=threading.Thread(target=serve,daemon=True); thread.start()
    try: rclpy.spin(node)
    finally: stop.set(); node.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
