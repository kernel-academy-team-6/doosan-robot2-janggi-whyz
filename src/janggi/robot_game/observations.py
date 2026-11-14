"""Recoverable vision errors and pure multi-frame stability checks."""
import math


class ObservationError(RuntimeError):
    """Transient camera/observation failure; never a robot motion failure."""


class SnapshotTimeout(ObservationError):
    pass


class TargetNotFound(ObservationError):
    pass


def stable_views(views, samples, settle_sec=.3, tolerance_px=4.):
    """Return recent stable frames after a continuous clear, stationary window.

    Keep off-grid pieces, ignore CNN label flicker, and require nonempty scenes.
    A blocked/empty/changing frame restarts the stability window.
    """
    window=[];reference=None
    for view in views:
        objects=view['objects']
        points={}
        for i,obj in enumerate(objects):
            cell=obj.get('cell');token=obj.get('piece_id')
            key=('cell',tuple(cell)) if cell is not None else ('id',token) if token else ('index',i)
            points[key]=obj['pixel']
        if (not view.get('clear',True) or not points or len(points)!=len(objects)
                or any(not all(math.isfinite(float(v)) for v in p) for p in points.values())):
            window=[];reference=None;continue
        same=(reference is not None and reference.keys()==points.keys() and
              all(math.dist(reference[k],points[k])<=tolerance_px for k in points))
        if not same:window=[];reference=points
        window.append(view)
    if len(window)>=samples and window[-1]['stamp']-window[0]['stamp']>=settle_sec:
        return window[-samples:]
    return []
