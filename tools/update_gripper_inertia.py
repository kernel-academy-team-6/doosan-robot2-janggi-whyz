"""Reproduce uniform-density STL inertias (SI); not manufacturer CAD inertias.

The 0.500 kg manufacturer total includes the actuator: retain finger masses and
assign the missing 0.084 kg to the housing. Internal mass distribution is unknown.
"""
from pathlib import Path
import json
import re
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'src/RH-P12-RN-A/rh_p12_rn_a_description'
DTYPE = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])

def properties(path, mass):
    triangles = np.frombuffer(path.read_bytes()[84:], dtype=DTYPE)['vertices'].astype(float) * .001
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    volume = np.einsum('ij,ij->i', a, np.cross(b, c)) / 6
    sums = triangles.sum(axis=1)
    center = np.einsum('i,ij->j', volume, sums) / (4 * volume.sum())
    second = np.einsum('i,ijk->jk', volume,
        np.einsum('nki,nkj->nij', triangles, triangles) + np.einsum('ni,nj->nij', sums, sums)) / (20 * volume.sum())
    covariance = second - np.outer(center, center)
    inertia = mass * (np.trace(covariance) * np.eye(3) - covariance)
    if volume.sum() <= 0 or np.linalg.eigvalsh(inertia).min() <= 0:
        raise ValueError(f'Invalid mesh inertia: {path}')
    return center, inertia

def main():
    path = PACKAGE / 'urdf/rh_p12_rn_a.xacro'
    text = path.read_text()
    report = {}
    for name, mass in [('base', .320), ('r1', .068), ('r2', .022), ('l1', .068), ('l2', .022)]:
        center, inertia = properties(PACKAGE / f'meshes/{name}.stl', mass)
        link_name = 'rh_p12_rn_base' if name == 'base' else 'rh_p12_rn_' + name
        pattern = r'(<link name="\$\{prefix\}' + link_name + r'">.*?<inertial>).*?(</inertial>)'
        values = {key: inertia[i,j] for key,i,j in [('ixx',0,0),('iyy',1,1),('izz',2,2),('ixy',0,1),('ixz',0,2),('iyz',1,2)]}
        body = ('\n        <!-- Uniform-density STL estimate; see tools/update_gripper_inertia.py. -->\n'
                f'        <origin xyz="{" ".join(f"{v:.12g}" for v in center)}" rpy="0 0 0"/>\n'
                f'        <mass value="{mass:.3f}"/>\n        <inertia ' + ' '.join(f'{k}="{v:.12g}"' for k,v in values.items()) + '/>\n      ')
        text, count = re.subn(pattern, lambda m:m[1]+body+m[2], text, flags=re.S)
        if count != 1:raise ValueError(link_name)
        report[name] = dict(mass_kg=mass, com_m=center.tolist(), inertia_kg_m2=inertia.tolist())
    path.write_text(text)
    out = ROOT / 'docs/analysis/gazebo_dynamics_fix/gripper_inertia.json'
    out.write_text(json.dumps(report, indent=2)+'\n')

if __name__ == '__main__':main()
