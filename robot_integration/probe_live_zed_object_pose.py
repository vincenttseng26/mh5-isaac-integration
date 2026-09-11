#!/usr/bin/env python3
"""Read one live ZED frame and estimate the red cube pose; never emits actions."""
import argparse, hashlib, json, math, socket, struct, time
from pathlib import Path
import numpy as np
import yaml

POINT_DTYPE=np.dtype([('xyz','<f4',3),('rgba','u1',4),('uv','<u2',2)])

def exact(stream,n):
    data=b''
    while len(data)<n:
        block=stream.recv(n-len(data))
        if not block: raise EOFError('ZED stream closed')
        data+=block
    return data

def quat_matrix(q):
    x,y,z,w=(float(v) for v in q); n=math.sqrt(x*x+y*y+z*z+w*w); x,y,z,w=x/n,y/n,z/n,w/n
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def rpy_matrix(values):
    r,p,y=(float(v) for v in values); cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])

def transform(rotation,translation):
    value=np.eye(4); value[:3,:3]=rotation; value[:3,3]=translation; return value

def main():
    p=argparse.ArgumentParser(); p.add_argument('--host',default='127.0.0.1'); p.add_argument('--port',type=int,default=8765)
    p.add_argument('--calibration',type=Path,required=True); p.add_argument('--correction',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True); p.add_argument('--minimum-points',type=int,default=40)
    a=p.parse_args(); config=yaml.safe_load(a.calibration.read_text()); eob=config['eye_on_base']; conv=config['zed_ros_frame_conventions']
    base_camera=transform(quat_matrix(eob['quaternion_xyzw']),np.asarray(eob['translation_xyz_m']))
    camera_optical=transform(rpy_matrix(conv['optical_rpy_from_camera_frame_rad']),np.asarray(conv['optical_offset_from_camera_frame_xyz_m']))
    correction=np.asarray(json.loads(a.correction.read_text())['total_correction_matrix_4x4'],dtype=float)
    optical_base=correction@base_camera@camera_optical
    with socket.create_connection((a.host,a.port),timeout=3) as stream:
        header=json.loads(exact(stream,struct.unpack('>I',exact(stream,4))[0])); payload=exact(stream,struct.unpack('>I',exact(stream,4))[0])
    checksum=hashlib.sha256(payload).hexdigest()==header['payload_sha256']; points=np.frombuffer(payload[:header['point_payload_bytes']],dtype=POINT_DTYPE)
    rgb=points['rgba'][:,:3].astype(float); xyz=points['xyz'].astype(float)
    red=(rgb[:,0]>=195)&(rgb[:,0]<=255)&(rgb[:,1]<=145)&(rgb[:,2]<=130)&((rgb[:,0]-rgb[:,1])>55)
    red&=np.isfinite(xyz).all(axis=1)&(xyz[:,2]>0.4)&(xyz[:,2]<2.0); selected=xyz[red]
    report={'mode':'live_read_only','command_authority':'none','actions_emitted':False,'real_commands_sent':0,
            'schema_version':header.get('schema_version'),'sequence':header.get('seq'),'checksum_ok':checksum,
            'source_frame':header.get('frame'),'valid_points':int(len(points)),'target_points':int(len(selected)),
            'timestamp_unix':time.time(),'valid':False,'rejection_reason':None}
    if not checksum: report['rejection_reason']='payload_checksum_failed'
    elif len(selected)<a.minimum_points: report['rejection_reason']='insufficient_red_cube_points'
    else:
        median=np.median(selected,axis=0); distance=np.linalg.norm(selected-median,axis=1); selected=selected[distance<=np.quantile(distance,0.90)]
        center_optical=np.median(selected,axis=0); center_base=(optical_base@np.append(center_optical,1))[:3]; center_base[2]-=0.03
        report.update(valid=bool(np.isfinite(center_base).all()),rejection_reason=None,
                      confidence=min(1.0,len(selected)/(4.0*a.minimum_points)),
                      position_base_link_m=center_base.tolist(),target_points_after_outlier_filter=int(len(selected)))
    a.output.parent.mkdir(parents=True,exist_ok=True); a.output.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n')
    print(json.dumps(report,indent=2,sort_keys=True))

if __name__=='__main__': main()
