#!/usr/bin/env python3
"""Fail-closed CYC-side TCP to ROS 1 trajectory gateway for the MH5."""
import json, math, os, socket, threading, time, uuid
import rospy
from industrial_msgs.msg import RobotStatus
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from staged_envelope import StageRejected, check_preconditions, validate_stage

JOINTS=('joint_1_s','joint_2_l','joint_3_u','joint_4_r','joint_5_b','joint_6_t')
HOST='192.168.50.10'; PORT=8769; TOKEN='MH5_LOCAL_TEST'
MAX_START_ERROR_RAD=0.02; MAX_SEGMENT_RAD=0.035; MAX_DURATION_S=30.0
AUDIT_PATH='/tmp/mh5_trajectory_audit.jsonl'
state={'mode':'MIRROR','armed':False,'positions':None,'joint_stamp':0.0,
       'robot':None,'robot_stamp':0.0,'pending':None,'nonce':None}
lock=threading.Lock()
audit_lock=threading.Lock()

def audit(event,**fields):
    record={'timestamp_unix':time.time(),'event':event,**fields}
    line=json.dumps(record,separators=(',',':'),sort_keys=True)+'\n'
    with audit_lock:
        with open(AUDIT_PATH,'a',encoding='utf-8') as stream:
            stream.write(line); stream.flush(); os.fsync(stream.fileno())
    rospy.loginfo('AUDIT %s',line.rstrip())

def joint_cb(msg):
    indices={name:i for i,name in enumerate(msg.name)}
    if any(name not in indices or indices[name]>=len(msg.position) for name in JOINTS): return
    values=tuple(float(msg.position[indices[name]]) for name in JOINTS)
    if all(math.isfinite(value) for value in values):
        with lock: state['positions'],state['joint_stamp']=values,time.monotonic()

def status_cb(msg):
    robot={'drives_powered':bool(msg.drives_powered.val),
           'motion_possible':bool(msg.motion_possible.val),
           'e_stopped':bool(msg.e_stopped.val),'in_error':bool(msg.in_error.val),
           'in_motion':bool(msg.in_motion.val),'error_code':int(msg.error_code)}
    with lock:
        state['robot'],state['robot_stamp']=robot,time.monotonic()
        if not robot['motion_possible'] or robot['e_stopped'] or robot['in_error']:
            state['armed']=False; state['nonce']=None
        pending=state['pending']; completion=None
        if pending and robot['in_motion']: pending['saw_motion']=True
        if pending and pending['saw_motion'] and not robot['in_motion']:
            actual=state['positions']; goal=pending['goal']
            error=max(abs(a-b) for a,b in zip(actual,goal)) if actual else None
            completion=dict(pending,actual=list(actual) if actual else None,max_error_rad=error)
            state['pending']=None
    if completion: audit('trajectory_completed',**completion)

def snapshot():
    with lock: result=dict(state)
    result['positions']=list(result['positions']) if result['positions'] else None
    result['joint_age_s']=time.monotonic()-result.pop('joint_stamp')
    result['robot_age_s']=time.monotonic()-result.pop('robot_stamp')
    result['ok']=True
    return result

def validate_trajectory(record):
    now=time.monotonic()
    with lock:
        mode,armed,actual=state['mode'],state['armed'],state['positions']
        joint_age=now-state['joint_stamp']; robot=state['robot']
        robot_age=now-state['robot_stamp']
    if mode!='COMMAND' or not armed: raise ValueError('gateway is not armed in COMMAND mode')
    if actual is None or joint_age>0.25: raise ValueError('joint feedback is stale')
    if robot is None or robot_age>0.25: raise ValueError('robot status is stale')
    if (not robot['drives_powered'] or not robot['motion_possible'] or robot['e_stopped']
            or robot['in_error'] or robot['in_motion']):
        raise ValueError('robot status does not permit a new trajectory')
    if record.get('schema')!=2 or tuple(record.get('joints',()))!=JOINTS:
        raise ValueError('invalid schema or joint order')
    points=record.get('points',())
    if not 2<=len(points)<=200: raise ValueError('trajectory requires 2..200 points')
    parsed=[]; previous=None; previous_t=0.0
    for item in points:
        positions=tuple(float(value) for value in item.get('positions',()))
        stamp=float(item.get('time_from_start',0.0))
        if len(positions)!=6 or not all(math.isfinite(value) for value in positions):
            raise ValueError('invalid position vector')
        if not math.isfinite(stamp) or stamp<=previous_t or stamp>MAX_DURATION_S:
            raise ValueError('invalid time_from_start')
        if previous is not None and max(abs(a-b) for a,b in zip(positions,previous))>MAX_SEGMENT_RAD:
            raise ValueError('trajectory segment exceeds step limit')
        parsed.append((positions,stamp)); previous,previous_t=positions,stamp
    if max(abs(a-b) for a,b in zip(parsed[0][0],actual))>MAX_START_ERROR_RAD:
        raise ValueError('first waypoint does not match physical feedback')
    if max(abs(a-b) for a,b in zip(parsed[-1][0],actual))>MAX_SEGMENT_RAD:
        raise ValueError('total trajectory displacement exceeds step limit')
    return parsed

def handle(record,publisher):
    command=record.get('cmd')
    if command=='status': return snapshot()
    if record.get('token')!=TOKEN: return {'ok':False,'error':'authentication failed'}
    if command=='set_mode':
        mode=record.get('mode')
        if mode not in ('MIRROR','COMMAND','STAGED'):
            return {'ok':False,'error':'invalid mode'}
        with lock: state['mode'],state['armed'],state['nonce']=mode,False,None
        audit('mode_changed',mode=mode,armed=False)
        return {'ok':True,'mode':mode,'armed':False}
    if command=='arm':
        with lock:
            robot=state['robot']; fresh=time.monotonic()-state['robot_stamp']<=0.25
            allowed=(state['mode'] in ('COMMAND','STAGED') and fresh and robot
                     and robot['drives_powered'] and robot['motion_possible']
                     and not robot['e_stopped'] and not robot['in_error']
                     and not robot['in_motion'])
            state['armed']=bool(allowed)
            # A staged move must quote the nonce issued by this arm, so a
            # duplicated or replayed trajectory cannot fire a second stage.
            state['nonce']=(uuid.uuid4().hex if allowed and state['mode']=='STAGED'
                            else None)
            nonce=state['nonce']
        audit('arm_result',accepted=bool(allowed),mode=state['mode'],
              staged=bool(nonce))
        return {'ok':bool(allowed),'mode':state['mode'],'armed':bool(allowed),
                'nonce':nonce,
                'error':None if allowed else 'preconditions not met'}
    if command=='disarm':
        with lock: state['armed'],state['nonce']=False,None
        audit('disarmed')
        return {'ok':True,'armed':False}
    if command=='audit':
        try:
            with audit_lock:
                with open(AUDIT_PATH,encoding='utf-8') as stream: lines=stream.readlines()[-50:]
            return {'ok':True,'events':[json.loads(line) for line in lines]}
        except FileNotFoundError: return {'ok':True,'events':[]}
    if command=='staged_trajectory':
        command_id=uuid.uuid4().hex
        try:
            now=time.monotonic()
            with lock:
                mode,armed,nonce=state['mode'],state['armed'],state['nonce']
                actual_raw=state['positions']; robot=state['robot']
                joint_age=now-state['joint_stamp']; robot_age=now-state['robot_stamp']
            actual=check_preconditions(mode,armed,nonce,record.get('nonce'),
                                       actual_raw,joint_age,robot,robot_age)
            stage,points=validate_stage(record,actual)
            message=JointTrajectory(); message.joint_names=list(JOINTS)
            for positions,stamp in points:
                point=JointTrajectoryPoint(); point.positions=list(positions)
                point.velocities=[0.0]*6
                point.time_from_start=rospy.Duration.from_sec(stamp)
                message.points.append(point)
            total=max(abs(a-b) for a,b in zip(points[-1][0],actual))
            with lock:
                state['pending']={'command_id':command_id,'start':list(actual),
                                  'goal':list(points[-1][0]),
                                  'duration_s':points[-1][1],'saw_motion':False,
                                  'stage':stage}
            publisher.publish(message)
            audit('staged_trajectory_published',command_id=command_id,stage=stage,
                  start=list(actual),goal=list(points[-1][0]),
                  duration_s=points[-1][1],point_count=len(points),
                  total_rad=total)
            return {'ok':True,'published':True,'armed':False,'stage':stage,
                    'command_id':command_id,'duration_s':points[-1][1],
                    'point_count':len(points),'total_rad':total}
        except (StageRejected,TypeError,ValueError) as exc:
            audit('staged_trajectory_rejected',command_id=command_id,
                  stage=record.get('stage'),error=str(exc))
            return {'ok':False,'error':str(exc),'armed':False}
        finally:
            # One stage per arm, always. The operator re-arms for the next one.
            with lock:
                state['armed'],state['mode'],state['nonce']=False,'MIRROR',None
    if command=='trajectory':
        command_id=uuid.uuid4().hex
        try:
            points=validate_trajectory(record); message=JointTrajectory(); message.joint_names=list(JOINTS)
            for positions,stamp in points:
                point=JointTrajectoryPoint(); point.positions=list(positions)
                point.velocities=[0.0]*6; point.time_from_start=rospy.Duration.from_sec(stamp)
                message.points.append(point)
            with lock:
                actual=list(state['positions']); state['pending']={
                    'command_id':command_id,'start':actual,'goal':list(points[-1][0]),
                    'duration_s':points[-1][1],'saw_motion':False}
            publisher.publish(message)
            audit('trajectory_published',command_id=command_id,start=actual,
                  goal=list(points[-1][0]),duration_s=points[-1][1],point_count=len(points))
            return {'ok':True,'published':True,'armed':False,'command_id':command_id}
        except (TypeError,ValueError) as exc:
            audit('trajectory_rejected',command_id=command_id,error=str(exc))
            return {'ok':False,'error':str(exc),'armed':False}
        finally:
            with lock:
                state['armed'],state['mode'],state['nonce']=False,'MIRROR',None
    return {'ok':False,'error':'unknown command'}

def main():
    rospy.init_node('mh5_joint_trajectory_tcp_receiver')
    publisher=rospy.Publisher('/joint_path_command',JointTrajectory,queue_size=1)
    rospy.Subscriber('/joint_states',JointState,joint_cb,queue_size=1)
    rospy.Subscriber('/robot_status',RobotStatus,status_cb,queue_size=1)
    server=socket.socket(); server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    server.bind((HOST,PORT)); server.listen(1); server.settimeout(1.0)
    rospy.logwarn('MH5 gateway ready in MIRROR mode and DISARMED')
    audit('gateway_started',mode='MIRROR',armed=False)
    try:
        while not rospy.is_shutdown():
            try: connection,peer=server.accept()
            except socket.timeout: continue
            connection.settimeout(2.0)
            with connection:
                stream=connection.makefile('r')
                try:
                    for raw in stream:
                        if len(raw)>65536: raise ValueError('oversized request')
                        try: reply=handle(json.loads(raw),publisher)
                        except Exception as exc:
                            with lock: state['armed'],state['nonce']=False,None
                            reply={'ok':False,'error':str(exc),'armed':False}
                        connection.sendall((json.dumps(reply,separators=(',',':'))+'\n').encode())
                finally:
                    stream.close()
                    with lock: state['armed'],state['nonce']=False,None
                    rospy.loginfo('gateway client disconnected: %s',peer[0])
    finally: server.close()

if __name__=='__main__': main()
