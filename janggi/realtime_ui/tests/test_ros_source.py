from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from ros_source import image_to_bgr


def test_rgb_conversion_respects_row_padding_and_owns_memory():
    data=bytearray([255,0,0,0,255,0,9,9, 0,0,255,1,2,3,9,9])
    msg=SimpleNamespace(width=2,height=2,step=8,encoding='rgb8',data=data)
    frame=image_to_bgr(msg)
    assert frame.tolist()==[[[0,0,255],[0,255,0]],[[255,0,0],[3,2,1]]]
    data[0]=0
    assert frame[0,0,2]==255


@pytest.mark.parametrize('encoding,data,step',[('32FC1',bytes(8),8),('rgb8',bytes(3),2),('rgb8',bytes(2),3)])
def test_invalid_camera_messages_fail_instead_of_returning_bad_frames(encoding,data,step):
    with pytest.raises(ValueError):image_to_bgr(SimpleNamespace(width=1,height=1,step=step,encoding=encoding,data=data))


def test_gazebo_source_defaults_to_real_hybrid_and_derived_calibration(monkeypatch):
    import app
    monkeypatch.setattr(sys,'argv',['app','--source','gazebo'])
    args=app.parse_args()
    assert args.model_mode=='hybrid'
    assert args.model.name=='weights_only_piece.pt'
    assert args.calibration.name=='gazebo_calibration.yaml'


def test_ui_gazebo_board_clicks_cannot_teleport_pieces(monkeypatch):
    import app
    ui=app.App.__new__(app.App);requests=[]
    ui.worker=SimpleNamespace(source=SimpleNamespace(move_piece=lambda a,b:requests.append((a,b))))
    ui.root=None;ui.sel=(0,6);ui.drag_xy=(1,1)
    monkeypatch.setattr(app.messagebox, 'showinfo', lambda *args,**kwargs:None)
    assert not ui._gazebo_human_input()
    ui._move_gazebo_piece((0,6),(0,5))
    assert requests==[] and ui.sel is None


@pytest.mark.parametrize('mode,host',[('real','127.0.0.1'),('virtual','192.168.1.2'),('','localhost')])
def test_gazebo_control_rejects_non_virtual_or_remote_driver(mode,host):
    from ros_source import require_virtual_description
    text=f'<robot><ros2_control><hardware><param name="mode">{mode}</param><param name="host">{host}</param></hardware></ros2_control></robot>'
    with pytest.raises(RuntimeError,match='virtual'):require_virtual_description(text)


def test_gazebo_control_accepts_local_virtual_driver():
    from ros_source import require_virtual_description
    require_virtual_description('<robot><ros2_control><hardware><param name="mode">virtual</param><param name="host">127.0.0.1</param></hardware></ros2_control></robot>')


def test_mouse_release_during_robot_turn_does_not_edit_the_game_board():
    import app
    ui=app.App.__new__(app.App)
    ui.args=SimpleNamespace(source='gazebo',human_side='blue')
    ui.rg=object();ui.game=SimpleNamespace(turn='red');ui.robot_busy=True
    ui.sel=(0,6);ui.drag_xy=(1,1)
    ui.play=lambda *args:pytest.fail('Mouse release edited game during robot turn')
    ui.on_release((0,5),None)
    assert ui.sel is None


def test_native_gazebo_rgb_pixels_match_ros_conversion():
    from ros_source import gazebo_image_to_bgr
    image_pb=pytest.importorskip('gz.msgs10.image_pb2')
    message=image_pb.Image(width=2,height=1,step=6,data=bytes([255,0,0,0,255,0]))
    message.pixel_format_type=message.DESCRIPTOR.fields_by_name['pixel_format_type'].enum_type.values_by_name['RGB_INT8'].number
    assert gazebo_image_to_bgr(message).tolist()==[[[0,0,255],[0,255,0]]]


def test_ros_fallback_uses_reliable_fresh_frame_qos():
    pytest.importorskip('rclpy')
    from rclpy.qos import ReliabilityPolicy,HistoryPolicy
    from ros_source import ros_image_qos
    qos=ros_image_qos()
    assert qos.reliability==ReliabilityPolicy.RELIABLE
    assert qos.history==HistoryPolicy.KEEP_LAST and qos.depth==1
