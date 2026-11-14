"""The ruler uses measured rigid poses and finite CAD surfaces, not joint goals."""
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from gazebo_gripper_gap import inner_pad,measure_pads,transform

MESH=Path(__file__).resolve().parents[2]/'src/RH-P12-RN-A/rh_p12_rn_a_description/meshes'


def rectangle(y):
    return np.array([[-.013,y,.02],[.013,y,.02],[.013,y,.04],[-.013,y,.04]])


def test_parallel_faces_measure_surface_gap_not_link_origin_distance():
    right=inner_pad(MESH/'r2.stl',[.001]*3,'right')
    left=inner_pad(MESH/'l2.stl',[.001]*3,'left')
    right[:,1]+=.03;left[:,1]-=.03
    result,_,_=measure_pads(right,left)
    assert result['gap_mm']==pytest.approx(52.2,abs=1e-5)
    assert result['center_mm']==pytest.approx(52.2,abs=1e-5)
    assert result['tilt_deg']==pytest.approx(0.)


def test_ruler_is_invariant_to_arm_position_and_orientation():
    right,left=rectangle(.02),rectangle(-.02)
    rotation=np.array([[0,-1,0],[0,0,-1],[1,0,0]])
    frame=np.eye(4);frame[:3,:3]=rotation;frame[:3,3]=[10.,-20.,30.]
    result,a,b=measure_pads(transform(right,frame),transform(left,frame))
    assert result['gap_mm']==pytest.approx(40.)
    assert np.linalg.norm(a-b)*1000==pytest.approx(40.)


def test_skewed_pads_report_true_minimum_and_nonparallel_angle():
    right,left=rectangle(.02),rectangle(-.02)
    angle=np.radians(10)
    frame=np.eye(4);frame[:3,:3]=[[1,0,0],[0,np.cos(angle),-np.sin(angle)],[0,np.sin(angle),np.cos(angle)]]
    frame[:3,3]=left.mean(axis=0)-frame[:3,:3]@left.mean(axis=0)
    result,_,_=measure_pads(right,transform(left,frame))
    assert result['tilt_deg']==pytest.approx(10.)
    assert result['gap_mm']==pytest.approx((.04-.01*np.sin(angle))*1000,abs=1e-6)
    assert result['center_mm']==pytest.approx(40.)


def test_finite_pad_edges_do_not_report_infinite_plane_distance():
    right,left=rectangle(.02),rectangle(-.02);right[:,0]+=.1
    result,_,_=measure_pads(right,left)
    assert result['gap_mm']==pytest.approx(np.hypot(.1-.026,.04)*1000)


def test_loaded_mesh_scale_is_used_instead_of_fixed_pad_offset():
    pad=inner_pad(MESH/'r2.stl',[.002]*3,'right')
    assert pad[:,1]==pytest.approx([-.0078]*4,abs=1e-9)
    assert np.ptp(pad[:,0])==pytest.approx(.052)
