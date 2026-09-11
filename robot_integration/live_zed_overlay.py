"""Read-only ZED stream receiver and accepted optical-to-base alignment."""
import hashlib, json, math, socket, struct, threading, time
from pathlib import Path
import numpy as np
import yaml
from .zed_stream_wire import rgb_image

POINT_DTYPE=np.dtype([('xyz','<f4',3),('rgba','u1',4),('uv','<u2',2)])

def _quat(q):
    x,y,z,w=(float(v) for v in q); n=math.sqrt(sum(float(v)*float(v) for v in q)); x,y,z,w=x/n,y/n,z/n,w/n
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def _rpy(v):
    r,p,y=(float(x) for x in v); cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])

def _T(rotation,translation):
    result=np.eye(4); result[:3,:3]=rotation; result[:3,3]=translation; return result

def accepted_transform(calibration, correction, translation_delta_m=(0.0, 0.0, 0.0)):
    cfg=yaml.safe_load(Path(calibration).read_text()); e=cfg['eye_on_base']; c=cfg['zed_ros_frame_conventions']
    base_camera=_T(_quat(e['quaternion_xyzw']),np.asarray(e['translation_xyz_m']))
    camera_optical=_T(_rpy(c['optical_rpy_from_camera_frame_rad']),np.asarray(c['optical_offset_from_camera_frame_xyz_m']))
    aligned=np.asarray(json.loads(Path(correction).read_text())['total_correction_matrix_4x4'],dtype=float)
    delta=np.asarray(translation_delta_m,dtype=float).reshape(3)
    if not np.isfinite(delta).all(): raise ValueError('translation delta must be finite')
    adjusted=np.eye(4); adjusted[:3,3]=delta
    return adjusted@aligned@base_camera@camera_optical

class LiveZedOverlay:
    def __init__(self,transform,host='127.0.0.1',port=8765):
        self.transform=np.asarray(transform); self.host=host; self.port=port
        self.lock=threading.Lock(); self.xyz=None; self.rgb=None; self.rgb_image=None; self.seq=-1; self.stamp=0.0; self.error=None
        self.stop=threading.Event(); self.thread=threading.Thread(target=self._run,daemon=True); self.thread.start()
    @staticmethod
    def _exact(stream,n):
        result=b''
        while len(result)<n:
            block=stream.recv(n-len(result))
            if not block: raise EOFError('ZED stream closed')
            result+=block
        return result
    def _run(self):
        while not self.stop.is_set():
            try:
                with socket.create_connection((self.host,self.port),timeout=3) as stream:
                    stream.settimeout(3)
                    while not self.stop.is_set():
                        header=json.loads(self._exact(stream,struct.unpack('>I',self._exact(stream,4))[0]))
                        payload=self._exact(stream,struct.unpack('>I',self._exact(stream,4))[0])
                        if hashlib.sha256(payload).hexdigest()!=header.get('payload_sha256'): raise ValueError('checksum mismatch')
                        raw=np.frombuffer(payload[:header['point_payload_bytes']],dtype=POINT_DTYPE)
                        image=rgb_image(header, payload)
                        xyz=raw['xyz'].astype(np.float64); world=(self.transform[:3,:3]@xyz.T).T+self.transform[:3,3]
                        finite=np.isfinite(world).all(axis=1)
                        world=world[finite].astype(np.float32); rgb=(raw['rgba'][finite,:3].astype(np.float32)/255.0)
                        with self.lock:
                            self.xyz,self.rgb,self.rgb_image,self.seq,self.stamp,self.error=(
                                world,rgb,image,int(header['seq']),time.monotonic(),None)
            except Exception as exc:
                with self.lock: self.error=str(exc)
                self.stop.wait(1.0)
    def latest(self):
        with self.lock:
            return self.xyz,self.rgb,self.seq,(time.monotonic()-self.stamp if self.stamp else float('inf')),self.error
    def latest_rgb_image(self):
        with self.lock:
            return self.rgb_image
    def close(self): self.stop.set()
