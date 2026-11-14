"""Run janggi/realtime_ui/app.py unchanged, accepting a hierarchical bundle .json as --classifier.

A normal best.pt keeps the original MobileNetClassifier path.
"""
import runpy, sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]; UI = ROOT/'janggi/realtime_ui'
sys.path[:0] = [str(UI), str(ROOT/'janggi'), str(Path(__file__).resolve().parent)]
import piece_detection.classifier as classifier_module
from hierarchical import HierarchicalClassifier
_original = classifier_module.MobileNetClassifier


def _classifier(path, device=None, batch_size=32, **kwargs):
    if Path(path).suffix == '.json':
        model = HierarchicalClassifier.from_bundle(path, device, batch_size)
        print(f'[분류] 계층형 번들 {path}: 전문 분류기 {[e[1]["classes"] for e in model.experts]}, 색 보정 {model.colour_strength}', flush=True)
        return model
    return _original(path, device, batch_size, **kwargs)


classifier_module.MobileNetClassifier = _classifier
sys.argv[0] = str(UI/'app.py')
runpy.run_path(str(UI/'app.py'), run_name='__main__')
