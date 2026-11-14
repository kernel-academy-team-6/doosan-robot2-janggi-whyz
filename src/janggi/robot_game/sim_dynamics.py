"""URDF gravity feedforward for the Gazebo-only effort controller (SI units)."""
import xml.etree.ElementTree as ET
import numpy as np
from gz_scene import origin_T, axis_rot


class GravityModel:
    def __init__(self, urdf):
        root=ET.fromstring(urdf)
        self.masses={}
        for link in root.findall('link'):
            inertial=link.find('inertial')
            if inertial is not None:
                self.masses[link.get('name')]=(float(inertial.find('mass').get('value')),
                                              origin_T(inertial.find('origin'))[:3,3])
        self.joints=[]
        for joint in root.findall('joint'):
            mimic=joint.find('mimic');axis=joint.find('axis')
            self.joints.append(dict(name=joint.get('name'),type=joint.get('type'),
                parent=joint.find('parent').get('link'),child=joint.find('child').get('link'),
                origin=origin_T(joint.find('origin')),axis=np.array([float(v) for v in axis.get('xyz').split()]) if axis is not None else np.array([0.,0.,1.]),
                mimic=(mimic.get('joint'),float(mimic.get('multiplier','1')),float(mimic.get('offset','0'))) if mimic is not None else None))
        links={link.get('name') for link in root.findall('link')}
        self.roots=links-{j['child'] for j in self.joints}
        self.descendants={}
        for joint in self.joints:
            below={joint['child']}
            while True:
                expanded=below|{j['child'] for j in self.joints if j['parent'] in below}
                if expanded==below:break
                below=expanded
            self.descendants[joint['name']]=below

    def _frames(self,positions):
        links={name:np.eye(4) for name in self.roots};frames={};pending=list(self.joints)
        while pending:
            rest=[]
            for joint in pending:
                if joint['parent'] not in links:rest.append(joint);continue
                frame=links[joint['parent']]@joint['origin'];frames[joint['name']]=frame
                q=positions.get(joint['name'],0.)
                if joint['mimic']:
                    leader,mul,offset=joint['mimic'];q=positions.get(leader,0.)*mul+offset
                motion=np.eye(4)
                if joint['type'] in ('revolute','continuous'):motion[:3,:3]=axis_rot(joint['axis'],q)
                elif joint['type']=='prismatic':motion[:3,3]=joint['axis']*q
                links[joint['child']]=frame@motion
            if len(rest)==len(pending):raise ValueError('Unresolved URDF gravity tree')
            pending=rest
        return links,frames

    def torques(self,positions):
        links,frames=self._frames(positions)
        com={name:links[name][:3,:3]@point+links[name][:3,3] for name,(_,point) in self.masses.items()}
        result={}
        for joint in self.joints:
            if not joint['name'].startswith('joint_'):continue
            frame=frames[joint['name']];axis=frame[:3,:3]@joint['axis'];axis/=np.linalg.norm(axis)
            moment=np.zeros(3)
            for name in self.descendants[joint['name']]:
                if name in com:
                    mass,_=self.masses[name]
                    moment+=np.cross(com[name]-frame[:3,3],[0.,0.,-9.80665*mass])
            result[joint['name']]=-float(axis@moment)
        return result
