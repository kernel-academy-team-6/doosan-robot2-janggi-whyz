from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from gazebo_recorder import CameraTimeline


def test_30fps_camera_timeline_records_one_minute_without_duplicates():
    timeline=CameraTimeline(30.,60.);output=[]
    for index in range(1801):output.extend(timeline.push(123.+index/30.,index,object()))
    assert len(output)==1800 and timeline.index==1800
    assert [entry[2] for entry in output]==list(range(1800))


def test_missing_sensor_frame_is_held_and_identifiable_by_source_sequence():
    timeline=CameraTimeline(30.,1.)
    assert list(timeline.push(0.,0,'first'))==[]
    result=list(timeline.push(2/30.,1,'third'))
    assert [item[2] for item in result]==[0,0]
    assert [item[3] for item in result]==['first','first']
    result=list(timeline.push(3/30.,2,'fourth'))
    assert result[0][2]==1 and result[0][0]==2


def test_time_reset_does_not_silently_mix_recordings():
    timeline=CameraTimeline(30.,60.)
    list(timeline.push(10.,1,'frame'))
    with pytest.raises(RuntimeError,match='되돌아'):list(timeline.push(0.,2,'new world'))


def test_long_time_gap_is_not_filled_with_a_misleading_long_freeze():
    timeline=CameraTimeline(30.,60.)
    list(timeline.push(0.,1,'frame'))
    with pytest.raises(RuntimeError,match='2초'):list(timeline.push(10.,2,'frame'))
