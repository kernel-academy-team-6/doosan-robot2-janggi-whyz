#!/usr/bin/env python3
"""Read-only ruler between simulated inner pad surfaces, not commanded opening.
Scene geometry and physical link poses come from Gazebo transport, without ROS
motion clients, joint-state FK, or a stroke-to-gap conversion.
"""
import argparse
import csv
import math
from pathlib import Path
import struct
import threading
import time
from urllib.parse import unquote, urlparse
import numpy as np
from scipy.optimize import lsq_linear

LINKS=('gripper_rh_p12_rn_r2','gripper_rh_p12_rn_l2')


def pose_matrix(pose):
    """Gazebo xyzw quaternion + translation to homogeneous link transform."""
    q=pose.orientation
    x,y,z,w=np.array([q.x,q.y,q.z,q.w],float)/np.linalg.norm([q.x,q.y,q.z,q.w])
    result=np.eye(4)
    result[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                  [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                  [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    result[:3,3]=[pose.position.x,pose.position.y,pose.position.z]
    return result


def transform(points,matrix):
    return np.asarray(points)@matrix[:3,:3].T+matrix[:3,3]


def inner_pad(path,scale,side):
    """Extract the inward flat rectangular face from the loaded RH-P12 STL.
    No width/kinematic constants: right is mesh minimum Y, left maximum Y.
    Reject a changed nonrectangular CAD instead of silently guessing a ruler.
    """
    data=Path(path).read_bytes()
    count=struct.unpack_from('<I',data,80)[0]
    if len(data)!=84+50*count:raise ValueError('Expected binary STL: '+str(path))
    dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')])
    triangles=np.frombuffer(data,offset=84,dtype=dtype)['vertices'].astype(float)*np.asarray(scale)
    y=triangles[:,:,1];level=y.min() if side=='right' else y.max()
    face=triangles[np.all(np.abs(y-level)<1e-7,axis=1)]
    if not len(face):raise ValueError('No inward planar pad in '+str(path))
    points=np.unique(np.round(face.reshape(-1,3),10),axis=0)
    low=points.min(axis=0);high=points.max(axis=0)
    if len(points)!=4 or high[0]-low[0]<1e-6 or high[2]-low[2]<1e-6:
        raise ValueError('Pad CAD changed; update surface selection for '+str(path))
    return np.array([[low[0],level,low[2]],[high[0],level,low[2]],
                     [high[0],level,high[2]],[low[0],level,high[2]]])


def measure_pads(right,left):
    """True closest points on two finite rectangles, including skewed pads."""
    right,left=np.asarray(right),np.asarray(left)
    matrix=np.column_stack((right[1]-right[0],right[3]-right[0],
                            -(left[1]-left[0]),-(left[3]-left[0])))
    solved=lsq_linear(matrix,left[0]-right[0],bounds=(0.,1.),tol=1e-12,lsq_solver='exact')
    if not solved.success:raise ValueError('Closest-surface solve failed')
    u,v,s,t=solved.x
    a=right[0]+u*(right[1]-right[0])+v*(right[3]-right[0])
    b=left[0]+s*(left[1]-left[0])+t*(left[3]-left[0])
    normal_r=np.cross(right[1]-right[0],right[3]-right[0]);normal_r/=np.linalg.norm(normal_r)
    normal_l=np.cross(left[1]-left[0],left[3]-left[0]);normal_l/=np.linalg.norm(normal_l)
    angle=math.degrees(math.acos(np.clip(abs(normal_r@normal_l),0.,1.)))
    return dict(gap_mm=float(np.linalg.norm(b-a)*1000),
                center_mm=float(np.linalg.norm(right.mean(axis=0)-left.mean(axis=0))*1000),
                tilt_deg=angle),a,b


def mesh_path(uri):
    if uri.startswith('file://'):return Path(unquote(urlparse(uri).path))
    if uri.startswith('package://'):
        from ament_index_python.packages import get_package_share_directory
        package,relative=uri[len('package://'):].split('/',1)
        return Path(get_package_share_directory(package))/relative
    path=Path(uri)
    if not path.is_absolute():raise ValueError('Unsupported mesh URI: '+uri)
    return path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world',default='empty');parser.add_argument('--model',default='janggi_physics_robot')
    parser.add_argument('--once',action='store_true');parser.add_argument('--rate',type=float,default=2.)
    parser.add_argument('--timeout',type=float,default=10.);parser.add_argument('--seconds',type=float)
    parser.add_argument('--csv',type=Path);parser.add_argument('--label',default='',help='Manual comparison label, e.g. stroke_700')
    args=parser.parse_args()
    if any(not math.isfinite(v) or v<=0 for v in [args.rate,args.timeout]+([args.seconds] if args.seconds is not None else [])):
        parser.error('Times/rate must be finite and positive')
    if args.csv and args.csv.exists():parser.error('CSV already exists; choose a new path')
    from gz.transport13 import Node
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.scene_pb2 import Scene
    from gz.msgs10.pose_v_pb2 import Pose_V
    node=Node();topic=f'/world/{args.world}/dynamic_pose/info'
    ok,scene=node.request(f'/world/{args.world}/scene/info',Empty(),Empty,Scene,int(args.timeout*1000))
    if not ok:parser.exit(1,'Gazebo scene unavailable. Check running world and GZ_PARTITION.\n')
    model=next((m for m in scene.model if m.name==args.model),None)
    if model is None:parser.exit(1,f'Model not found: {args.model}\n')
    pads={};ids={};provenance=[]
    for name,side in zip(LINKS,('right','left')):
        link=next((l for l in model.link if l.name.split('::')[-1]==name),None)
        if link is None:parser.exit(1,f'Physical finger link not found: {name}\n')
        visuals=[v for v in link.visual if v.geometry.HasField('mesh')]
        if len(visuals)!=1:parser.exit(1,f'Expected one pad mesh for {name}; got {len(visuals)}\n')
        visual=visuals[0];mesh=visual.geometry.mesh
        scale=[mesh.scale.x,mesh.scale.y,mesh.scale.z]
        if not all(v>0 for v in scale):parser.exit(1,'Invalid scale in Gazebo mesh\n')
        path=mesh_path(mesh.filename)
        pads[name]=transform(inner_pad(path,scale,side),pose_matrix(visual.pose))
        ids[name]=link.id;provenance.append(f'{name}: {path}, scale={scale}, entity={link.id}')
    print('Gazebo physical pad ruler (read-only)\n'+'\n'.join(provenance),flush=True)
    print('gap_mm=closest inner STL surfaces; center_mm=pad centers; tilt_deg=nonparallel angle.\n'
          'Bullet contact margin is not included. No commanded stroke / FK conversion is used.',flush=True)
    condition=threading.Condition();latest=[None];sequence=[0]
    def receive(message):
        # One complete physics pose snapshot; never mix old/new finger frames.
        by_id={p.id:p for p in message.pose}
        if not all(i in by_id for i in ids.values()):return
        with condition:
            latest[0]=(message.header.stamp.sec+message.header.stamp.nsec*1e-9,
                       {name:pose_matrix(by_id[i]) for name,i in ids.items()},time.monotonic())
            sequence[0]+=1;condition.notify_all()
    if not node.subscribe(Pose_V,topic,receive):parser.exit(1,'Could not subscribe: '+topic+'\n')
    stream=None;writer=None
    start=time.monotonic();last_sequence=0;last_print=-math.inf
    try:
        while True:
            with condition:
                condition.wait_for(lambda:sequence[0]>last_sequence,timeout=min(args.timeout,1.))
                now=time.monotonic()
                if args.seconds is not None and now-start>=args.seconds:break
                if latest[0] is None or now-latest[0][2]>args.timeout:
                    if now-start<args.timeout:continue
                    parser.exit(1,'No fresh physical link poses. Check simulation / pause state / model respawn.\n')
                if sequence[0]==last_sequence:continue
                last_sequence=sequence[0];sim_s,frames,received=latest[0]
            if now-last_print<1/args.rate:continue
            result,a,b=measure_pads(*(transform(pads[n],frames[n]) for n in LINKS))
            row=dict(label=args.label,wall_unix_s=time.time(),sim_s=sim_s,**result)
            for prefix,point in [('right_nearest',a),('left_nearest',b)]:
                row.update({f'{prefix}_{axis}_m':float(value) for axis,value in zip('xyz',point)})
            print(f"sim={sim_s:.3f}s  gap={result['gap_mm']:.3f} mm  centers={result['center_mm']:.3f} mm  tilt={result['tilt_deg']:.4f} deg",flush=True)
            if args.csv:
                if writer is None:
                    args.csv.parent.mkdir(parents=True,exist_ok=True)
                    stream=args.csv.open('x',newline='')  # do not overwrite previous measurements
                    writer=csv.DictWriter(stream,fieldnames=list(row));writer.writeheader()
                writer.writerow(row);stream.flush()
            last_print=now
            if args.once:break
    except KeyboardInterrupt:
        pass
    finally:
        node.unsubscribe(topic)
        if stream:stream.close();print('Saved:',args.csv.resolve())


if __name__=='__main__':main()
