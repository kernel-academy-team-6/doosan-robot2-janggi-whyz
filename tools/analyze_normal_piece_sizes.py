#!/usr/bin/env python3
"""Measure raw box sizes on conservatively normal frames in a recorded run.

Uses original video + saved detections, without model inference or hardware.
Recomputes global obstruction status; original per-piece diagnostic regions are
also required to be empty. Does not fit or modify runtime size weights.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
from collections import Counter, defaultdict

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'janggi'),str(ROOT/'janggi/realtime_ui')]
from diagnostics import ObstructionMonitor, BoxSizeEstimator
from piece_detection.board_mapping import BoardMapper
import cv2

KINDS={'king':'궁','cha':'차','po':'포','horse':'마','sang':'상','sa':'사','soldier':'졸·병'}
SIZE_GROUPS={'king':('궁',('king',)),
             'medium':('중간말',('cha','po','horse','sang')),
             'small':('작은말',('sa','soldier'))}

def summary(values):
    return dict(n=len(values), mean_mm=statistics.mean(values) if values else None,
        sample_variance_mm2=statistics.variance(values) if len(values)>1 else None,
        sample_std_mm=statistics.stdev(values) if len(values)>1 else None,
        min_mm=min(values) if values else None,max_mm=max(values) if values else None)

def size_group_summaries(records):
    """Pool original observations, including between-kind variation."""
    results=[]
    for group,(name,kinds) in SIZE_GROUPS.items():
        values=[float(r['geometric_size_mm']) for r in records if r['kind'] in kinds]
        row=dict(group=group,group_ko=name,kinds=','.join(kinds),**summary(values))
        if row['sample_std_mm'] is not None:
            row.update(mean_minus_6sd_mm=row['mean_mm']-6*row['sample_std_mm'],
                       mean_plus_6sd_mm=row['mean_mm']+6*row['sample_std_mm'])
        results.append(row)
    return results

def sha(path):
    with Path(path).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--session',type=Path,default=ROOT/'janggi/realtime_ui/evaluations/counts_20261111_055048_087201')
    ap.add_argument('--output',type=Path,default=ROOT/'docs/analysis/normal_piece_sizes')
    a=ap.parse_args();session=a.session.resolve();out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
    metadata=json.loads((session/'counts.summary.json').read_text())['metadata']
    state=Path(metadata['state_directory']);state_meta=json.loads((state/'session.json').read_text())
    calibration=state/'calibration.yaml'
    video=Path(metadata['source'])
    if not video.is_absolute():video=(ROOT/'janggi'/video).resolve()
    mapper=BoardMapper.from_calibration(calibration,piece_height_mm=state_meta['piece_height_mm'])
    estimator=BoxSizeEstimator.from_calibration(calibration,state_meta['piece_height_mm'])
    grid=mapper.grid_pixels();polygon=[grid[0,0],grid[0,-1],grid[-1,-1],grid[-1,0]]
    monitor=ObstructionMonitor(window=2.,settle=.8,motion_threshold=.03)
    counts=list(csv.DictReader((session/'counts.csv').open()))
    cap=cv2.VideoCapture(str(video))
    if not cap.isOpened():raise RuntimeError('Cannot open original video')
    current=-1;context=None;filtered=Counter();status_counts=Counter();records=[];frame_audit=[]
    with (session/'classifications.jsonl').open() as stream:
        for number,line in enumerate(stream):
            det_frame=json.loads(line)
            if number>=len(counts):raise RuntimeError('Detection rows exceed count rows')
            row=counts[number]
            assert int(row['frame'])==det_frame['inference_frame']+1
            assert int(row['source_frame'])==det_frame['frame']
            assert int(row['source_loop'])==det_frame['loop']
            if int(row['source_loop'])!=0:
                filtered['repeat_loop_frames']+=1;continue
            target=int(row['source_frame'])
            if target<current:raise RuntimeError('Non-monotonic first-playback frames')
            image=None
            while current<target:
                ok,image=cap.read();current+=1
                if not ok:raise RuntimeError('Original video ended early')
            if image is None:raise RuntimeError('Duplicate first-playback source frame')
            new_context=(row['source_loop'],row['confidence_threshold'],row['roi_used'])
            if new_context!=context:monitor.reset();context=new_context
            view=monitor.update(image,int(row['deduplicated']),int(row['assigned']),float(row['elapsed_s']),polygon)
            status_counts[view['state']]+=1
            reasons=[]
            if view['state']!='stable':reasons.append('global_'+view['state'])
            regions=det_frame.get('piece_motion_regions')
            if regions is None:reasons.append('local_diagnostics_missing')
            elif regions:reasons.append('local_diagnostic_regions')
            if any(d.get('tracking_only') for d in det_frame['detections']):reasons.append('tracking_only_present')
            frame_audit.append(dict(source_frame=target,elapsed_s=row['elapsed_s'],global_state=view['state'],
                motion=view['motion'],local_regions=len(regions) if regions is not None else None,
                used=not reasons,excluded_reasons=';'.join(reasons)))
            if reasons:
                filtered.update(reasons);continue
            filtered['normal_frames']+=1
            for d in det_frame['detections']:
                label=d['label'];kind=label.split('_',1)[1] if '_' in label else None
                if (kind not in KINDS or d.get('classification_preview') or
                    d.get('classification_conf',0)<metadata['classifier_threshold'] or
                    d['detection_conf']<float(row['confidence_threshold'])):
                    filtered['unconfirmed_or_low_confidence_boxes']+=1;continue
                size=estimator.measure(tuple(d['box']),(image.shape[1],image.shape[0]))
                if size is None:filtered['invalid_geometry_boxes']+=1;continue
                w,h=size;geom=math.sqrt(w*h)
                records.append(dict(source_frame=target,source_time_s=row['source_time_s'],
                    piece_id=d['piece_id'],label=label,kind=kind,kind_ko=KINDS[kind],
                    width_mm=w,height_mm=h,geometric_size_mm=geom,detection_conf=d['detection_conf'],
                    classification_conf=d['classification_conf']))
    cap.release()
    if not records:raise RuntimeError('No normal observations passed the filters')
    grouped=defaultdict(list);by_piece=defaultdict(list)
    for r in records:grouped[r['kind']].append(r['geometric_size_mm']);by_piece[(r['piece_id'],r['label'])].append(r['geometric_size_mm'])
    results=[]
    for kind,name in KINDS.items():
        row=dict(kind=kind,kind_ko=name,**summary(grouped[kind]))
        row['track_ids']=len({r['piece_id'] for r in records if r['kind']==kind})
        # Within-track variability separates temporal box jitter from differences
        # between distinct tracks of the same class. IDs are not physical truth.
        groups=[v for (pid,label),v in by_piece.items() if label.split('_',1)[1]==kind]
        dof=sum(len(v)-1 for v in groups)
        pooled=sum(sum((x-statistics.mean(v))**2 for x in v) for v in groups)/dof if dof else None
        row['pooled_within_track_variance_mm2']=pooled
        row['pooled_within_track_std_mm']=math.sqrt(pooled) if pooled is not None else None
        results.append(row)
    per_piece=[dict(piece_id=pid,label=label,**summary(v)) for (pid,label),v in sorted(by_piece.items())]
    size_groups=size_group_summaries(records)
    for name,rows in [('normal_observations.csv',records),('kind_summary.csv',results),('size_group_summary.csv',size_groups),('per_track_summary.csv',per_piece),('frame_filter_audit.csv',frame_audit)]:
        if rows:
            with (out/name).open('w',newline='') as f:
                writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    report=dict(source_session=str(session.relative_to(ROOT)),source_video=str(video.relative_to(ROOT)),
        calibration=str(calibration.relative_to(ROOT)),frame_counts=dict(status_counts),filters=dict(filtered),
        total_normal_observations=len(records),kinds=results,size_groups=size_groups,
        metric='sqrt(width_mm * height_mm) from the current raw detection box; not the stored rolling median',
        variance='sample variance, denominator n-1; frame observations are temporally correlated',
        normal_definition='first playback only; replayed ObstructionMonitor stable; no local diagnostic regions anywhere; no tracking-only boxes; confirmed labels and configured confidence thresholds',
        obstruction_replay=dict(window_seconds=2.,settle_seconds=.8,motion_threshold=.03,
            timestamp='recorded counts.csv elapsed_s; polygon from the saved calibration',
            limitations='Original global state/manual obstruction-reset actions were not logged. Normal states are recomputed with current code and default monitor parameters, not claimed as exact original runtime states.'),
        additional_limitations=['Heuristic normal is not proof of no stationary occlusion.','Labels are model predictions, not independently checked class ground truth.','No size-based outlier rejection is added; empirical dispersion is not truncated by the runtime size prior.','A track ID is not a guaranteed unique physical piece.','These empirical statistics do not replace the current fixed 1.8 mm Gaussian width.'],
        input_sha256={p.name:sha(p) for p in [video,calibration,session/'counts.csv',session/'classifications.jsonl']})
    (out/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False))
    print(json.dumps(dict(filters=dict(filtered),global_status=dict(status_counts),kinds=results),ensure_ascii=False,indent=2),flush=True)
    print('Saved:',out,flush=True)

if __name__=='__main__':main()
