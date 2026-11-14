import json
import math

import pytest

from janggi_tcp_test.profile import Profile,CompensationPlan,endpoints
from janggi_tcp_test.report import save_report
from janggi_pick_place2.geometry import command_from_width,tcp_length,width_from_command


def test_unit_conversion_and_documented_acceleration_time():
    profile = Profile(0.,100.,50,100,.088)
    assert profile.velocity == pytest.approx(3./.088)
    assert profile.acceleration == pytest.approx(10./.088)
    assert profile.acceleration_time == pytest.approx(.3)
    assert profile.kind == 'trapezoidal'
    assert profile.duration == pytest.approx(100./profile.velocity+.3)


@pytest.mark.parametrize('u0,u1',[(0.,700.),(700.,0.),(200.,201.),(201.,200.)])
def test_profile_boundaries_monotonicity_and_continuity(u0,u1):
    profile = Profile(u0,u1)
    assert profile.position(-1.) == u0
    assert profile.position(profile.duration+1.) == u1
    points=[profile.position(profile.duration*i/1000) for i in range(1001)]
    assert all(min(u0,u1)-1e-9 <= u <= max(u0,u1)+1e-9 for u in points)
    assert all((b-a)*(u1-u0) >= -1e-9 for a,b in zip(points,points[1:]))
    for t in (profile.acceleration_time,profile.acceleration_time+profile.cruise_time):
        assert abs(profile.position(t+1e-6)-profile.position(t-1e-6)) < .001


def test_short_motion_is_triangular():
    profile=Profile(200.,201.)
    assert profile.kind=='triangular' and profile.cruise_time==0.
    assert profile.duration==pytest.approx(2*math.sqrt(1/profile.acceleration))


@pytest.mark.parametrize('initial,target',[(40.,60.),(60.,30.),(106.,6.),(6.,106.)])
def test_compensation_preserves_model_tip_and_only_changes_z(initial,target):
    u0,u1=endpoints(initial,target)
    plan=CompensationPlan(Profile(u0,u1),[400.,50.,450.,25.,180.,10.])
    for i in range(101):
        t=plan.profile.duration*i/100
        sample=plan.sample(t)
        assert sample['predicted_tip_z_mm']==pytest.approx(plan.tip_z,abs=1e-10)
        pose=plan.target_pose(t)
        assert pose[:2]==[400.,50.] and pose[3:]==[25.,180.,10.]
    from janggi_pick_place2.geometry import height_from_command
    assert plan.target_pose(plan.profile.duration)[2] == pytest.approx(
        450+height_from_command(u1)-height_from_command(u0))


def test_tilt_uses_actual_z_component():
    u0,u1=endpoints(40.,60.)
    plan=CompensationPlan(Profile(u0,u1),[400.,0.,450.,0.,179.,0.])
    assert plan.sample(plan.profile.duration)['predicted_tip_z_mm']==pytest.approx(plan.tip_z)
    with pytest.raises(ValueError,match='point down'):
        CompensationPlan(Profile(u0,u1),[400.,0.,450.,0.,170.,0.])


@pytest.mark.parametrize('initial,target',[(5.,30.),(30.,107.),(float('nan'),30.),(30.,float('inf')),(30.,30.)])
def test_invalid_gaps(initial,target):
    with pytest.raises(ValueError):endpoints(initial,target)


def test_motion_limits_reject_infeasible_profiles():
    u0,u1=endpoints(30.,60.)
    plan=CompensationPlan(Profile(u0,u1),[400.,0.,450.,0.,180.,0.])
    with pytest.raises(ValueError):plan.validate(120.,.01,.01,.01,-200.,1000.)
    stats=plan.validate(120.,100.,10000.,100000.,-200.,1000.)
    assert stats['peak_z_velocity_mm_s']>0
    with pytest.raises(ValueError,match='duration'):
        plan.validate(.0001,30.,5.,10.,-200.,1000.)
    with pytest.raises(ValueError,match='height bounds'):
        plan.validate(120.,30.,5.,10.,500.,1000.)


def test_offline_report_distinguishes_simulation(tmp_path):
    rows=[{'kind':'model','elapsed_sec':0.,'model_width_mm':40.,'desired_flange_z_mm':450.}]
    measured=[{'kind':'feedback','elapsed_sec':0.,'estimated_tip_z_mm':290.,
               'tip_error_mm':0.,'source':'mock_model_only'}]
    save_report(tmp_path,rows,measured,{'backend':'mock','message':'<not physical>'})
    assert 'mock_model_only' in (tmp_path/'samples.csv').read_text()
    assert json.loads((tmp_path/'summary.json').read_text())['backend']=='mock'
    text=(tmp_path/'report.html').read_text()
    assert '<svg' in text and '&lt;not physical&gt;' in text
    assert 'No gripper position or current is read' in text
    assert 'not sent to the gripper' in text
