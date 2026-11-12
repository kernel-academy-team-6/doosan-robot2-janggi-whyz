"""The refinement wrapper must work with Worker session metadata, not just inference."""
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import app
from piece_detection.refinement import RefinedClassifier


def test_refined_classifier_worker_session_initialization(tmp_path,monkeypatch):
    model=tmp_path/'main.pt'
    model.write_bytes(b'main checkpoint metadata')
    base=SimpleNamespace(path=model,classes=['blue_cha','red_cha'],size=224)
    classifier=RefinedClassifier(base)
    detector=SimpleNamespace(classifier=classifier,classification_conf=.5,device='cpu',iou=.5,imgsz=640,
                             detect_positions=lambda *args:[],classify_observations=lambda *args:{})
    args=SimpleNamespace(window=2,full_frame=True,log_counts=False,calibration=None,model=model,
                         state_dir=tmp_path/'states',source='synthetic',conf=.35,max_dist=.45)
    # Exercise the real constructor without starting cameras, robot clients or a UI.
    monkeypatch.setattr(app.Worker,'_run',lambda self:None)
    worker=app.Worker(SimpleNamespace(),detector,None,args)
    try:
        metadata=json.loads((worker.state_store.directory/'session.json').read_text())
        assert classifier.path is model
        assert classifier.path.resolve()==base.path.resolve()
        assert metadata['classifier']==str(model)
        assert metadata['classifier_sha256']==hashlib.sha256(model.read_bytes()).hexdigest()
    finally:
        worker.stop()
        worker.classifier_worker.close()
