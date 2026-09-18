#!/usr/bin/env python3
"""Isaac GUI panel for guarded, one-shot MH5 virtual-to-real commands."""
import argparse, json, math, socket, sys, time
from pathlib import Path
from isaacsim import SimulationApp

JOINTS=('joint_1_s','joint_2_l','joint_3_u','joint_4_r','joint_5_b','joint_6_t')
# Physical tabletop-edge validation on 2026-09-11 found the live cloud was
# displaced toward -X by about 25 mm.  Keep this as a reversible runtime
# correction; the authored calibration JSON remains immutable.
DEFAULT_ZED_TRANSLATION_DELTA_M=(0.025, 0.0, 0.0)
TOKEN='MH5_LOCAL_TEST'; GATEWAY=('192.168.50.10',8769)
PLANNER=('192.168.50.10',8770)

def exchange(records):
    replies=[]
    with socket.create_connection(GATEWAY,timeout=2) as connection:
        stream=connection.makefile('r')
        for record in records:
            connection.sendall((json.dumps(record,separators=(',',':'))+'\n').encode())
            line=stream.readline()
            if not line: raise ConnectionError('gateway closed connection')
            replies.append(json.loads(line))
    return replies

def status(): return exchange([{'cmd':'status'}])[0]
def mirror(): return exchange([{'cmd':'set_mode','mode':'MIRROR','token':TOKEN}])[0]

def _plan_request(payload,timeout=8):
    with socket.create_connection(PLANNER,timeout=timeout) as connection:
        connection.sendall((json.dumps(payload)+'\n').encode())
        reply=json.loads(connection.makefile('r').readline())
    if not reply.get('ok'): raise RuntimeError('MoveIt: '+str(reply.get('error')))
    return reply

def request_plan(goal):
    return _plan_request({'target_rad':list(goal)},timeout=3)

def request_pose_plan(position,yaw,start=None):
    """Plan grasp_link to a top-down Cartesian pose. Plan-only."""
    payload={'target_pose':{'position_m':[float(v) for v in position],
                            'yaw_rad':float(yaw)}}
    if start is not None: payload['start_rad']=[float(v) for v in start]
    return _plan_request(payload)

def execute_once(points):
    records=[{'cmd':'set_mode','mode':'COMMAND','token':TOKEN},
             {'cmd':'arm','token':TOKEN},
             {'schema':2,'cmd':'trajectory','token':TOKEN,'joints':JOINTS,
              'points':points}]
    replies=exchange(records)
    # The CYC gateway also returns to MIRROR after publication. Reassert it so
    # a network retry or older gateway can never leave command mode selected.
    mirror()
    return replies

def parse_args():
    parser=argparse.ArgumentParser()
    parser.add_argument('--usd',default='/home/vincent/Desktop/mh5_physical_workcell_persistent_zed_visual/mh5_physical_workcell.usda')
    parser.add_argument('--prim',default='/World/mh5_rg2ft')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--shadow-actions',default='shadow_policy_candidate.jsonl',
                        help='six-axis delta_rad JSONL used only by PLAY SHADOW')
    parser.add_argument('--zed-calibration',default='/home/vincent/Desktop/isaac_pc_setup_zed_model_20260821/zed_sn24180_hd1080.yaml')
    parser.add_argument('--zed-correction',default='/home/vincent/Desktop/mh5_isaaclab_reach_lift_v0/zed_tabletop_edge_alignment_charuco_centered_20260908.json')
    parser.add_argument('--zed-translation-delta-m', nargs=3, type=float,
                        default=DEFAULT_ZED_TRANSLATION_DELTA_M,
                        metavar=('DX','DY','DZ'),
                        help='reversible runtime delta applied after correction (metres)')
    parser.add_argument('--object-profile',
                        default='/home/vincent/Desktop/mh5_isaaclab_reach_lift_v0/'
                                'object_profiles/blue_tape_measure.yaml')
    parser.add_argument('--detection-report',
                        default='/home/vincent/Desktop/mh5_isaaclab_reach_lift_v0/'
                                'zed_object_detection_latest.json')
    parser.add_argument('--smoke-test',action='store_true',
                        help='initialize, sync once in MIRROR, then exit without commands')
    return parser.parse_args()

def main():
    args=parse_args(); usd=Path(args.usd).expanduser().resolve()
    if not usd.is_file(): raise SystemExit('USD not found: '+str(usd))
    app=SimulationApp({'headless':args.headless,'hide_ui':args.headless})
    window=None; zed_overlay=None
    try:
        import numpy as np
        import omni.ui as ui
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.stage import open_stage
        import omni.usd
        from pxr import UsdGeom, Vt
        from robot_integration.live_zed_overlay import LiveZedOverlay, accepted_transform
        from robot_integration.grasp_pipeline import (detect_objects, grasp_stages,
                                                      load_profile)
        print('[INIT] opening USD',flush=True)
        if not open_stage(str(usd)): raise RuntimeError('could not open USD')
        print('[INIT] USD opened; creating world',flush=True)
        app.update(); World.clear_instance()
        world=World(stage_units_in_meters=1.0,backend='numpy',device='cpu')
        world.get_physics_context().set_gravity(0.0)
        robot=world.scene.add(SingleArticulation(prim_path=args.prim,name='mh5_command_preview',
                                                reset_xform_properties=False))
        print('[INIT] articulation added; resetting world',flush=True)
        world.reset()
        print('[INIT] world reset complete',flush=True)
        if tuple(robot.dof_names[:6])!=JOINTS: raise RuntimeError('unexpected MH5 joint order')
        stage=omni.usd.get_context().get_stage()
        zed_points=UsdGeom.Points.Define(stage,'/World/PhysicalZEDLiveAligned')
        zed_colors=zed_points.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex)
        # Initialise the three arrays together and re-assert vertex
        # interpolation, mirroring zed_live_stream_isaac_viewer.py, which is
        # the implementation known to render a coloured cloud on this build.
        zed_points.GetPointsAttr().Set([])
        zed_points.GetDisplayColorAttr().Set([])
        zed_points.GetWidthsAttr().Set([])
        UsdGeom.PrimvarsAPI(zed_points).GetPrimvar(
            'displayColor').SetInterpolation(UsdGeom.Tokens.vertex)
        zed_overlay=LiveZedOverlay(accepted_transform(
            args.zed_calibration, args.zed_correction, args.zed_translation_delta_m))
        print(f'[ZED] runtime translation delta m={tuple(args.zed_translation_delta_m)}', flush=True)
        # RGB is a separate 2-D diagnostic view.  It is never inserted into
        # the USD world as geometry; the point cloud remains the metric view.
        rgb_provider=ui.ByteImageProvider()
        rgb_window=ui.Window('ZED RGB (live, read-only)',width=640,height=400)
        with rgb_window.frame:
            with ui.VStack():
                ui.Label('ZED left RGB — live diagnostic image')
                ui.ImageWithProvider(rgb_provider)
        object_profile,profile_motion_allowed=load_profile(args.object_profile)
        print(f'[PROFILE] {object_profile.name} loaded; '
              f'robot_motion_allowed={profile_motion_allowed}',flush=True)
        marker=UsdGeom.Points.Define(stage,'/World/DetectedObjectMarker')
        marker_colors=marker.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex)
        indices=np.arange(6,dtype=np.int32); limits=robot.dof_properties[:6]
        models=[]; ui_state={'physical':None,'preview':None,'plan':None,'confirm_until':0.0,
                            'last_poll':0.0,'last_feedback_apply':0.0,'last_logged':None,
                            'shadow_queue':[],'shadow_index':0,'shadow_next':0.0,
                            'shadow_active':False,'shadow_hold':False,'shadow_goal':None}
        zed_state={'seq':-1,'last_apply':0.0,'points':0,'age':float('inf'),'error':None}
        grasp_state={'detection':None,'stages':None,'queue':[],'index':0,
                     'next':0.0,'active':False,'hold':False,
                     'detections':[],'selected':0,'seq':-1,'age':0.0}

        window=ui.Window('MH5 Guarded Command',width=520,height=500)
        print('[INIT] building command panel',flush=True)
        with window.frame:
            with ui.VStack(spacing=5):
                title=ui.Label('Starting in MIRROR / DISARMED',height=28)
                state_label=ui.Label('Connecting to CYC gateway...',word_wrap=True,height=48)
                delta_label=ui.Label('No preview captured',word_wrap=True,height=44)
                ui.Separator()
                for name in JOINTS:
                    with ui.HStack(height=28):
                        ui.Label(name,width=130)
                        model=ui.SimpleFloatModel(0.0); models.append(model)
                        ui.FloatField(model=model,width=180)
                        ui.Label('rad')

                def set_fields(values):
                    for model,value in zip(models,values): model.set_value(float(value))

                def get_fields(): return tuple(float(model.get_value_as_float()) for model in models)

                def sync_physical():
                    reply=status(); values=reply.get('positions')
                    if not reply.get('ok') or not values: raise RuntimeError('no physical feedback')
                    ui_state['physical']=tuple(values); ui_state['preview']=None; ui_state['plan']=None
                    ui_state['shadow_active']=False; ui_state['shadow_hold']=False
                    ui_state['shadow_goal']=None
                    grasp_state.update(active=False,hold=False,queue=[],index=0)
                    set_fields(values); robot.set_joint_positions(np.asarray(values,dtype=np.float32),joint_indices=indices)
                    mirror(); title.text='MIRROR / DISARMED — physical pose synchronized'

                def preview():
                    reply=status(); actual=tuple(reply.get('positions') or ())
                    goal=get_fields()
                    if len(actual)!=6 or not all(math.isfinite(v) for v in goal):
                        raise RuntimeError('invalid physical or target values')
                    for i,(value,prop) in enumerate(zip(goal,limits)):
                        if value<float(prop['lower']) or value>float(prop['upper']):
                            raise RuntimeError(JOINTS[i]+' exceeds USD joint limit')
                    delta=[b-a for a,b in zip(actual,goal)]
                    if max(abs(v) for v in delta)>0.035:
                        raise RuntimeError('preview exceeds 0.035 rad (about 2 deg) limit')
                    plan=request_plan(goal)
                    robot.set_joint_positions(np.asarray(goal,dtype=np.float32),joint_indices=indices)
                    ui_state['physical']=actual; ui_state['preview']=goal; ui_state['plan']=plan
                    ui_state['confirm_until']=0.0
                    delta_label.text='delta deg: '+' '.join(f'{math.degrees(v):+.3f}' for v in delta)
                    title.text=(f"MOVEIT PLAN OK ({len(plan['points'])} points, "
                                f"{plan['execution_duration_s']:.1f}s) — ARM then EXECUTE")

                def play_shadow():
                    reply=status(); actual=tuple(reply.get('positions') or ())
                    if reply.get('mode')!='MIRROR' or reply.get('armed'):
                        raise RuntimeError('shadow requires MIRROR / DISARMED')
                    path=Path(args.shadow_actions).expanduser().resolve()
                    if not path.is_file(): raise RuntimeError('shadow actions not found: '+str(path))
                    shadow=actual; queue=[]
                    for line_number,line in enumerate(path.read_text(encoding='utf-8').splitlines(),1):
                        if not line.strip(): continue
                        delta=tuple(float(v) for v in json.loads(line).get('delta_rad',()))
                        if len(delta)!=6 or not all(math.isfinite(v) for v in delta):
                            raise RuntimeError(f'invalid shadow action at line {line_number}')
                        if max(abs(v) for v in delta)>0.01:
                            raise RuntimeError(f'shadow step exceeds 0.01 rad at line {line_number}')
                        shadow=tuple(a+b for a,b in zip(shadow,delta))
                        if max(abs(a-b) for a,b in zip(shadow,actual))>0.035:
                            raise RuntimeError('shadow path exceeds 0.035 rad from physical pose')
                        if max(abs(a-b) for a,b in zip(shadow,actual))>=1e-7:
                            request_plan(shadow)
                        queue.append(shadow)
                    if not queue: raise RuntimeError('shadow action file is empty')
                    mirror()
                    ui_state['shadow_queue']=queue; ui_state['shadow_index']=0
                    ui_state['shadow_next']=time.monotonic(); ui_state['shadow_active']=True
                    ui_state['shadow_hold']=True; ui_state['shadow_goal']=queue[-1]
                    ui_state['confirm_until']=0.0; ui_state['preview']=None; ui_state['plan']=None
                    title.text=f'SHADOW PLAYBACK — 0/{len(queue)}; REAL COMMANDS DISABLED'

                def stage_shadow_goal():
                    if ui_state['shadow_active']:
                        raise RuntimeError('wait for shadow playback to finish')
                    goal=ui_state.get('shadow_goal')
                    if goal is None: raise RuntimeError('run PLAY SHADOW first')
                    reply=status(); actual=tuple(reply.get('positions') or ())
                    if reply.get('mode')!='MIRROR' or reply.get('armed'):
                        raise RuntimeError('staging requires MIRROR / DISARMED')
                    if len(actual)!=6: raise RuntimeError('fresh physical feedback unavailable')
                    delta=[b-a for a,b in zip(actual,goal)]
                    if max(abs(v) for v in delta)>0.035:
                        raise RuntimeError('policy goal exceeds 0.035 rad from current physical pose')
                    if max(abs(v) for v in delta)<1e-7:
                        raise RuntimeError('policy final goal equals current physical pose (no movement)')
                    plan=request_plan(goal)
                    set_fields(goal); robot.set_joint_positions(
                        np.asarray(goal,dtype=np.float32),joint_indices=indices)
                    ui_state['physical']=actual; ui_state['preview']=goal; ui_state['plan']=plan
                    ui_state['shadow_hold']=True; ui_state['confirm_until']=0.0
                    delta_label.text='policy delta deg: '+' '.join(
                        f'{math.degrees(v):+.3f}' for v in delta)
                    title.text=(f'POLICY GOAL STAGED ({len(plan["points"])} points) — '
                                'inspect, then ARM and EXECUTE ONCE')

                def show_marker(points,colors,widths=None):
                    if not points:
                        marker.GetPointsAttr().Set(Vt.Vec3fArray()); return
                    if widths is None:
                        widths=[0.030,0.018,0.018,0.018][:len(points)]
                    marker.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(
                        np.asarray(points,dtype=np.float32)))
                    marker.GetWidthsAttr().Set(Vt.FloatArray.FromNumpy(
                        np.asarray(widths,dtype=np.float32)))
                    marker_colors.Set(Vt.Vec3fArray.FromNumpy(
                        np.asarray(colors,dtype=np.float32)))

                def draw_detections():
                    """Green = selected object, grey = the others, plus stages."""
                    found=grasp_state.get('detections') or []
                    index=grasp_state.get('selected',0)
                    stages=grasp_state.get('stages')
                    points=[]; colors=[]; widths=[]
                    for i,d in enumerate(found):
                        points.append(d.centroid_m)
                        selected=(i==index)
                        colors.append((0.0,1.0,0.2) if selected else (0.55,0.55,0.6))
                        widths.append(0.030 if selected else 0.020)
                    if stages is not None and stages.valid:
                        points+=[stages.pre_grasp_m,stages.grasp_m,stages.lift_m]
                        colors+=[(0.2,0.6,1.0),(1.0,0.9,0.0),(0.2,0.6,1.0)]
                        widths+=[0.018,0.018,0.018]
                    show_marker(points,colors,widths)

                def apply_selection():
                    """Re-stage and re-report for whichever object is selected.

                    The report keeps a single `grasp_stages` block naming the
                    SELECTED object, because run_staged_grasp.py drives the real
                    arm from that block. `detections` lists everything found, so
                    a second object is visible without changing what the
                    physical path reads.
                    """
                    found=grasp_state.get('detections') or []
                    if not found: raise RuntimeError('no detections to select from')
                    index=max(0,min(grasp_state.get('selected',0),len(found)-1))
                    grasp_state['selected']=index
                    detection=found[index]
                    stages=grasp_stages(detection,object_profile)
                    grasp_state.update(detection=detection,stages=stages)
                    draw_detections()
                    seq=grasp_state.get('seq',-1); age=grasp_state.get('age',0.0)
                    report={'profile':object_profile.name,
                            'robot_motion_allowed':bool(profile_motion_allowed),
                            'zed_sequence':int(seq),'zed_age_s':float(age),
                            'object_count':len(found),'selected_index':index,
                            'detections':[d.as_dict() for d in found],
                            'detection':detection.as_dict(),
                            'grasp_stages':stages.as_dict(),
                            'timestamp_unix':time.time(),
                            'actions_emitted':False,'real_commands_sent':0}
                    Path(args.detection_report).write_text(
                        json.dumps(report,indent=2,sort_keys=True)+'\n')
                    print(f'[DETECT] {len(found)} object(s); selected {index+1}/{len(found)} '
                          +json.dumps(report['detection'],sort_keys=True),flush=True)
                    print('[DETECT] stages '+json.dumps(report['grasp_stages'],
                                                        sort_keys=True),flush=True)
                    centre=detection.centroid_m
                    delta_label.text=(f'[{index+1}/{len(found)}] centroid base_link m: '+
                        ' '.join(f'{v:+.4f}' for v in centre)+
                        f' | {detection.point_count} pts')
                    where=f'OBJECT {index+1}/{len(found)}'
                    if not stages.valid:
                        title.text=where+' — grasp staging rejected: '+str(stages.reason)
                    else:
                        title.text=(f'{where} — yaw {math.degrees(stages.yaw_rad):+.1f} deg, '
                                    f'open {stages.gripper_open_m*1000:.0f} mm; '
                                    'verify with a ruler')
                    return detection,stages

                def detect_object_now():
                    """Detect every profile object in the live cloud. No motion."""
                    xyz,rgb,seq,age,error=zed_overlay.latest()
                    if xyz is None:
                        raise RuntimeError('no ZED frame yet: '+str(error))
                    if age>1.5:
                        raise RuntimeError(f'ZED frame is {age:.2f} s stale')
                    found,reason=detect_objects(xyz,rgb,object_profile)
                    if not found:
                        grasp_state.update(detections=[],detection=None,
                                           stages=None,selected=0)
                        show_marker([],[])
                        raise RuntimeError('detection failed: '+str(reason))
                    grasp_state.update(detections=found,selected=0,
                                       seq=int(seq),age=float(age))
                    detection,stages=apply_selection()

                def select_next_object():
                    """Cycle the grasp target. Never moves anything."""
                    found=grasp_state.get('detections') or []
                    if len(found)<2:
                        raise RuntimeError(f'only {len(found)} object detected; '
                                           'nothing to cycle to')
                    grasp_state['selected']=(grasp_state.get('selected',0)+1)%len(found)
                    # A staged plan belongs to the previous target; drop it.
                    ui_state['preview']=None; ui_state['plan']=None
                    ui_state['confirm_until']=0.0
                    apply_selection()

                def plan_play_grasp():
                    """Plan and animate the grasp in Isaac only. Never commands."""
                    stages=grasp_state.get('stages')
                    if stages is None or not stages.valid:
                        raise RuntimeError('run DETECT OBJECT for a valid grasp first')
                    reply=status(); actual=tuple(reply.get('positions') or ())
                    if reply.get('mode')!='MIRROR' or reply.get('armed'):
                        raise RuntimeError('virtual grasp requires MIRROR / DISARMED')
                    if len(actual)!=6:
                        raise RuntimeError('fresh physical feedback unavailable')
                    queue=[]; start=actual
                    for label,point in (('pre-grasp',stages.pre_grasp_m),
                                        ('grasp',stages.grasp_m),
                                        ('lift',stages.lift_m)):
                        plan=request_pose_plan(point,stages.yaw_rad,start)
                        for entry in plan['points']:
                            queue.append(tuple(float(v) for v in entry['positions']))
                        start=queue[-1]
                        print(f'[VIRTUAL] {label} planned, {len(plan["points"])} points; '
                              'real_transport_touched=false',flush=True)
                    mirror()
                    grasp_state.update(queue=queue,index=0,next=time.monotonic(),
                                       active=True,hold=True)
                    ui_state['preview']=None; ui_state['plan']=None
                    ui_state['confirm_until']=0.0
                    title.text=f'VIRTUAL GRASP — 0/{len(queue)}; REAL COMMANDS DISABLED'

                def arm_preview():
                    if ui_state['preview'] is None or ui_state['plan'] is None:
                        raise RuntimeError('capture a valid MoveIt preview first')
                    ui_state['confirm_until']=time.monotonic()+5.0
                    title.text='CONFIRM ARMED FOR 5 s — next EXECUTE click can move the real MH5'

                def execute():
                    if ui_state['preview'] is None or time.monotonic()>ui_state['confirm_until']:
                        raise RuntimeError('confirmation expired; ARM PREVIEW again')
                    replies=execute_once(ui_state['plan']['points'])
                    ui_state['confirm_until']=0.0
                    if not all(item.get('ok') for item in replies): raise RuntimeError(str(replies))
                    ui_state['shadow_active']=False; ui_state['shadow_hold']=False
                    ui_state['shadow_goal']=None
                    command_id=next((item.get('command_id') for item in replies
                                     if item.get('command_id')),None)
                    title.text=('COMMAND PUBLISHED id='+str(command_id or 'unknown')+
                                ' — gateway auto-disarmed; monitoring feedback')

                def safe_call(callback):
                    def wrapped():
                        try: callback()
                        except Exception as exc:
                            try: mirror()
                            except Exception: pass
                            ui_state['confirm_until']=0.0; title.text='REJECTED / MIRROR: '+str(exc)
                    return wrapped

                with ui.HStack(height=34):
                    ui.Button('SYNC PHYSICAL',clicked_fn=safe_call(sync_physical))
                    ui.Button('PREVIEW VIRTUAL',clicked_fn=safe_call(preview))
                with ui.HStack(height=34):
                    ui.Button('DETECT OBJECT (ZED)',clicked_fn=safe_call(detect_object_now))
                    ui.Button('SELECT NEXT OBJECT',
                              clicked_fn=safe_call(select_next_object))
                    ui.Button('PLAN + PLAY GRASP (VIRTUAL)',
                              clicked_fn=safe_call(plan_play_grasp))
                ui.Button('PLAY SHADOW (VIRTUAL ONLY)',height=34,clicked_fn=safe_call(play_shadow))
                ui.Button('STAGE SHADOW GOAL (NO MOTION)',height=34,
                          clicked_fn=safe_call(stage_shadow_goal))
                with ui.HStack(height=38):
                    ui.Button('ARM PREVIEW (5 s)',clicked_fn=safe_call(arm_preview))
                    ui.Button('EXECUTE ONCE',clicked_fn=safe_call(execute))
                ui.Button('RETURN TO MIRROR / DISARM',height=34,clicked_fn=safe_call(sync_physical))

        sync_physical()
        print('[INIT] physical pose synchronized; panel ready',flush=True)
        if args.smoke_test:
            print('[PASS] Isaac MH5 panel initialized in MIRROR / DISARMED',flush=True)
            return 0
        while app.is_running():
            app.update(); now=time.monotonic()
            if ui_state['shadow_active'] and now>=ui_state['shadow_next']:
                reply=status()
                if reply.get('mode')!='MIRROR' or reply.get('armed'):
                    ui_state['shadow_active']=False
                    title.text='SHADOW ABORTED — gateway left MIRROR / DISARMED'
                else:
                    index=ui_state['shadow_index']; target=ui_state['shadow_queue'][index]
                    robot.set_joint_positions(np.asarray(target,dtype=np.float32),joint_indices=indices)
                    index+=1; ui_state['shadow_index']=index; ui_state['shadow_next']=now+0.75
                    print(f'[SHADOW] virtual step {index}/{len(ui_state["shadow_queue"])}; '
                          'real_transport_touched=false',flush=True)
                    title.text=f'SHADOW PLAYBACK — {index}/{len(ui_state["shadow_queue"])}; REAL COMMANDS DISABLED'
                    if index>=len(ui_state['shadow_queue']):
                        ui_state['shadow_active']=False
                        title.text='SHADOW COMPLETE — real MH5 unchanged; RETURN TO MIRROR when ready'
            if grasp_state['active'] and now>=grasp_state['next']:
                reply=status()
                if reply.get('mode')!='MIRROR' or reply.get('armed'):
                    grasp_state['active']=False
                    title.text='VIRTUAL GRASP ABORTED — gateway left MIRROR / DISARMED'
                else:
                    index=grasp_state['index']; target=grasp_state['queue'][index]
                    robot.set_joint_positions(np.asarray(target,dtype=np.float32),
                                              joint_indices=indices)
                    index+=1; grasp_state['index']=index
                    grasp_state['next']=now+0.10
                    total=len(grasp_state['queue'])
                    title.text=f'VIRTUAL GRASP — {index}/{total}; REAL COMMANDS DISABLED'
                    if index>=total:
                        grasp_state['active']=False
                        print('[VIRTUAL] grasp playback complete; '
                              'real_commands_sent=0',flush=True)
                        title.text=('VIRTUAL GRASP COMPLETE — real MH5 unchanged; '
                                    'RETURN TO MIRROR when ready')
            if now-ui_state['last_poll']>0.2:
                ui_state['last_poll']=now
                try:
                    reply=status(); r=reply.get('robot') or {}
                    state_label.text=(f"FS100 motion_possible={r.get('motion_possible')} "
                                      f"servo={r.get('drives_powered')} moving={r.get('in_motion')} "
                                      f"error={r.get('in_error')} | gateway={reply.get('mode')} "
                                      f"armed={reply.get('armed')} | ZED seq={zed_state['seq']} "
                                      f"points={zed_state['points']} age={zed_state['age']:.2f}s")
                    # MIRROR continuously follows physical feedback. During a
                    # one-shot command it also follows motion until completion.
                    if (not ui_state['shadow_active'] and not ui_state['shadow_hold'] and
                        not grasp_state['active'] and not grasp_state['hold'] and
                        (reply.get('mode')=='MIRROR' or r.get('in_motion') or title.text.startswith('COMMAND PUBLISHED'))):
                        values=reply.get('positions')
                        if values:
                            robot.set_joint_positions(np.asarray(values,dtype=np.float32),joint_indices=indices)
                            observed=tuple(float(v) for v in robot.get_joint_positions(joint_indices=indices))
                            previous=ui_state['last_logged']
                            if previous is None or max(abs(a-b) for a,b in zip(observed,previous))>1e-4:
                                print('[MIRROR] Isaac applied '+' '.join(
                                    f'{name}={value:.6f}' for name,value in zip(JOINTS,observed)),flush=True)
                                ui_state['last_logged']=observed
                    if (r.get('e_stopped') or r.get('in_error') or not r.get('motion_possible')):
                        ui_state['confirm_until']=0.0
                except Exception as exc: state_label.text='STALE/DISCONNECTED: '+str(exc)
            if now-zed_state['last_apply']>0.2:
                xyz,rgb,seq,age,error=zed_overlay.latest(); zed_state.update(age=age,error=error)
                if xyz is not None and seq!=zed_state['seq'] and age<1.5:
                    zed_points.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(xyz))
                    zed_points.GetWidthsAttr().Set(Vt.FloatArray.FromNumpy(
                        np.full(len(xyz),0.004,dtype=np.float32)))
                    zed_colors.Set(Vt.Vec3fArray.FromNumpy(rgb))
                    rgb_image=zed_overlay.latest_rgb_image()
                    if rgb_image is not None and rgb_image.ndim==3 and rgb_image.shape[2]>=3:
                        rgba=np.concatenate((rgb_image[:,:,:3],
                            np.full((*rgb_image.shape[:2],1),255,dtype=np.uint8)),axis=2)
                        rgb_provider.set_bytes_data(list(rgba.tobytes()),
                                                    [int(rgba.shape[1]),int(rgba.shape[0])])
                    if zed_state['seq']<0:
                        print(f'[ZED] aligned live point cloud seq={seq} points={len(xyz)} age={age:.3f}s',flush=True)
                        # Read the primvar back so a colourless cloud can be
                        # blamed on the data, the USD write or the renderer
                        # rather than guessed at.
                        primvar=UsdGeom.PrimvarsAPI(zed_points).GetPrimvar('displayColor')
                        stored=primvar.Get()
                        print(f'[ZED] colour source rgb {rgb.dtype} shape={rgb.shape} '
                              f'min={float(rgb.min()):.3f} max={float(rgb.max()):.3f} '
                              f'mean={float(rgb.mean()):.3f} '
                              f'distinct_rows={len(np.unique(rgb,axis=0))}',flush=True)
                        print(f'[ZED] displayColor primvar stored={0 if stored is None else len(stored)} '
                              f'interpolation={primvar.GetInterpolation()} '
                              f'first={None if not stored else tuple(round(v,3) for v in stored[0])}',
                              flush=True)
                    zed_state.update(seq=seq,last_apply=now,points=len(xyz))
            world.step(render=True)
    finally:
        if zed_overlay is not None: zed_overlay.close()
        if 'rgb_window' in locals() and rgb_window is not None: rgb_window.visible=False
        try: mirror()
        except Exception: pass
        if window is not None: window.visible=False
        app.close()

if __name__=='__main__': main()
