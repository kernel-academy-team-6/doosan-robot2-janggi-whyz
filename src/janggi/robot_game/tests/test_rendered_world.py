from pathlib import Path
import sys,time
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import sim_app
from simulation_calibration import ensure_simulation_calibration
from gazebo_scene import RenderedWorld


def test_scene_has_no_ground_truth_detection_or_automatic_turn_interface():
    w=RenderedWorld(ensure_simulation_calibration())
    assert len(w.pieces)==32
    for name in ('detections','fresh_state','human_move'):
        assert not hasattr(w,name)


def test_no_attachment_or_manual_coordinate_moves_are_allowed():
    w=RenderedWorld(ensure_simulation_calibration())
    before=[p['pos'].copy() for p in w.pieces]
    with pytest.raises(RuntimeError,match='disabled'):w.grip(700)
    with pytest.raises(RuntimeError,match='disabled'):w.move_piece([0,6],[0,5])
    for old,p in zip(before,w.pieces):np.testing.assert_array_equal(old,p['pos'])


def test_gray_table_is_horizontal_with_top_exactly_30mm_below_base():
    import xml.etree.ElementTree as ET
    from gz_scene import GzScene
    from scene_models import CalibratedLayout
    path=ensure_simulation_calibration();world=RenderedWorld(path)
    scene=GzScene.__new__(GzScene);scene.w=world
    scene.layout=CalibratedLayout.from_world(world,path)
    scene.n,scene.R,scene.center=scene.layout.normal,scene.layout.rotation,scene.layout.center
    scene._refresh_existing_models=lambda:None
    scene._remove=lambda name:True
    models=[];scene._create=lambda sdf:models.append(ET.fromstring(sdf).find('model'))
    scene.set_camera=lambda *args:None
    scene.build()
    table=models[0]
    pose=np.fromstring(table.findtext('pose'),sep=' ')
    size=np.fromstring(table.findtext('link/visual/geometry/box/size'),sep=' ')
    np.testing.assert_allclose(pose[3:],[0,0,0],atol=1e-12)
    assert pose[2]+size[2]/2==pytest.approx(-.030,abs=1e-12)


@pytest.mark.parametrize('board_present,gui_ready,expected_error',[(True,True,None),(False,True,'janggi_board'),(True,False,'GUI')])
def test_focus_command_checks_existing_board_without_recreating_scene(monkeypatch,board_present,gui_ready,expected_error):
    import gazebo_scene
    events=[]
    class Scene:
        def __init__(self,*args,**kwargs):
            self._existing_models={'janggi_table','piece_0'}|({'janggi_board'} if board_present else set())
        def _refresh_existing_models(self):events.append('inspect')
        def focus_board(self):events.append('focus');return gui_ready
        def stop(self):events.append('stop')
        def start(self,*args,**kwargs):pytest.fail('Focus recreated the scene')
        def build(self):pytest.fail('Focus recreated the scene')
    monkeypatch.setattr(gazebo_scene,'GzScene',Scene)
    monkeypatch.setattr(gazebo_scene,'RenderedWorld',lambda calibration:object())
    if expected_error:
        with pytest.raises(RuntimeError,match=expected_error):gazebo_scene.focus_existing_board()
    else:gazebo_scene.focus_existing_board()
    assert events==(['inspect','focus','stop'] if board_present else ['inspect','stop'])


