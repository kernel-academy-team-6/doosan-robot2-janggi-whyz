"""Offline E0509 URDF kinematics cross-check; no ROS or robot connection."""
from pathlib import Path
import json
import math
import sys
import xml.etree.ElementTree as ET
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/janggi_pick_place2'))
from janggi_pick_place2.executor import Settings
from janggi_pick_place2.geometry import rotation_zyz
from janggi_pick_place2.locked_wrist import LockedWristPlanner


def rx(a):
    c,s=np.cos(a),np.sin(a);return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def ry(a):
    c,s=np.cos(a),np.sin(a);return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def rz(a):
    c,s=np.cos(a),np.sin(a);return np.array([[c,-s,0],[s,c,0],[0,0,1]])


class Kinematics:
    def __init__(self,initial):
        r=ET.parse(ROOT/'src/doosan-robot2/dsr_description2/xacro/macro.e0509.white.xacro').getroot()
        self.transforms=[];self.seed=np.radians(initial)
        for i in range(1,7):
            origin=r.find(f".//joint[@name='joint_{i}']/origin")
            roll,pitch,yaw=map(float,origin.get('rpy','0 0 0').split())
            t=np.eye(4);t[:3,:3]=rz(yaw)@ry(pitch)@rx(roll)
            t[:3,3]=list(map(float,origin.get('xyz','0 0 0').split()))
            self.transforms.append(t)

    def matrix(self,q):
        t=np.eye(4);origins=[];axes=[]
        for fixed,angle in zip(self.transforms,q):
            t=t@fixed;origins.append(t[:3,3].copy());axes.append(t[:3,2].copy())
            rotation=np.eye(4);rotation[:3,:3]=rz(angle);t=t@rotation
        return t,origins,axes

    def forward(self,q,check):
        check();t,_,_=self.matrix(np.radians(q));r=t[:3,:3]
        b=math.acos(max(-1.,min(1.,r[2,2])))
        if abs(math.sin(b))>1e-5:
            a,c=math.atan2(r[1,2],r[0,2]),math.atan2(r[2,1],-r[2,0])
        elif b>math.pi/2:
            a,b,c=math.atan2(-r[1,0],-r[0,0]),math.pi,0.
        else:
            a,b,c=math.atan2(r[1,0],r[0,0]),0.,0.
        return [*map(float,t[:3,3]*1000),*map(math.degrees,(a,b,c))]

    def inverse(self,pose,solution,check):
        check();target_p=np.array(pose[:3])/1000;target_r=np.array(rotation_zyz(pose[3:]))
        q=self.seed.copy()
        for _ in range(60):
            t,origins,axes=self.matrix(q);r=target_r@t[:3,:3].T
            angle=math.acos(max(-1.,min(1.,(np.trace(r)-1)/2)))
            vector=np.array([r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1]])/2
            if angle>1e-6:vector*=angle/math.sin(angle)
            error=np.r_[target_p-t[:3,3],vector]
            if np.linalg.norm(error)<1e-8:
                self.seed=q;return np.degrees(q).tolist()
            j=np.vstack([np.array([np.cross(z,t[:3,3]-p) for z,p in zip(axes,origins)]).T,np.array(axes).T])
            delta=np.linalg.lstsq(j,error,rcond=None)[0]
            q+=delta*min(1.,.2/max(1e-9,np.max(np.abs(delta))))
        raise RuntimeError('offline numerical IK failed')


def main():
    params=yaml.safe_load((ROOT/'src/janggi_pick_place2/config/pick_place.yaml').read_text())['/janggi_pick_place2']['ros__parameters']
    fields=Settings.__dataclass_fields__
    settings=Settings(**{k:v for k,v in params.items() if k in fields}).validate()
    k=Kinematics(settings.home_joint_deg)
    planner=LockedWristPlanner(settings,k.inverse,k.forward)
    home=k.forward(settings.home_joint_deg,lambda:None)
    pose=home;joints=list(settings.home_joint_deg);reports=[]
    for name,target in [('above',[406.5872,-119.455,home[2]]),('small_pick',[406.5872,-119.455,143.]),
                         ('retreat',[406.5872,-119.455,home[2]]),('far_corner',[642.,236.,home[2]]),
                         ('large_place',[642.,236.,145.]),('retreat2',[642.,236.,home[2]]),
                         ('medium_above',[288.,-119.,home[2]]),('medium_pick',[288.,-119.,144.])]:
        path=planner.plan(pose,joints,[*target,0.,180.,0.],0,lambda:None)
        assert all(q[5]==90. for q in path.joints)
        reports.append(dict(stage=name,points=len(path.joints),target_flange_mm=target,
                            max_joint6_error_deg=max(abs(q[5]-90.) for q in path.joints),
                            endpoint_error_mm=math.dist(path.target_pose[:3],target),duration_sec=path.duration))
        print(name,reports[-1],flush=True)
        pose=path.target_pose;joints=path.joints[-1]
    result=dict(validation='offline URDF + independent numerical IK, not physical execution',home_pose=home,paths=reports)
    Path(__file__).with_name('results.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
