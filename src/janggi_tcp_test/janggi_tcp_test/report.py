"""Offline CSV + HTML/SVG report, without plotting/network dependencies."""
import csv
import html
import json
import math
from pathlib import Path


COLUMNS = ['kind', 'elapsed_sec', 'model_u', 'model_width_mm', 'model_length_mm',
           'desired_flange_z_mm', 'command_flange_z_mm', 'predicted_tip_z_mm',
           'measured_flange_z_mm', 'estimated_tip_z_mm', 'tip_error_mm',
           'measurement_span_sec', 'source', 'gripper_feedback_received', 'arm_pose_received']


def chart(rows, title, lines):
    series = []
    for key, color, label in lines:
        points = []
        for row in rows:
            x, y = row.get('elapsed_sec'), row.get(key)
            if isinstance(x, (int,float)) and isinstance(y, (int,float)) and math.isfinite(x) and math.isfinite(y):
                points.append((x,y))
        if points:
            series.append((points,color,label))
    if not series:
        return '<p>No samples: '+html.escape(title)+'</p>'
    xs = [x for points,_,_ in series for x,_ in points]
    ys = [y for points,_,_ in series for _,y in points]
    xmin,xmax,ymin,ymax = min(xs),max(xs),min(ys),max(ys)
    xmax = max(xmax,xmin+.1)
    padding = max((ymax-ymin)*.1,.05)
    ymin,ymax = ymin-padding,ymax+padding
    out = [f'<h2>{html.escape(title)}</h2><svg viewBox="0 0 900 270" role="img">',
           '<rect x="65" y="15" width="810" height="205" fill="#f5f7fa"/>']
    for i in range(5):
        y = ymin+(ymax-ymin)*i/4
        py = 220-i*205/4
        out.append(f'<text x="58" y="{py+4}" text-anchor="end">{y:.2f}</text>')
    for i in range(6):
        x = xmin+(xmax-xmin)*i/5
        px = 65+i*810/5
        out.append(f'<text x="{px}" y="240" text-anchor="middle">{x:.2f}</text>')
    out.append('<text x="450" y="264" text-anchor="middle">time (s)</text>')
    for points,color,_ in series:
        coords = ' '.join(f'{65+(x-xmin)/(xmax-xmin)*810:.2f},{220-(y-ymin)/(ymax-ymin)*205:.2f}' for x,y in points)
        out.append(f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="2"/>')
    out.append('</svg><p>'+ ' · '.join(f'<span style="color:{color}">{html.escape(label)}</span>' for _,color,label in series)+'</p>')
    return ''.join(out)


def save_report(directory, plan_rows, feedback_rows, metadata):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    rows = sorted([*plan_rows,*feedback_rows], key=lambda row:row.get('elapsed_sec',0.))
    with (directory/'samples.csv').open('w',newline='') as stream:
        writer = csv.DictWriter(stream,fieldnames=COLUMNS,extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)
    (directory/'summary.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2,allow_nan=False))
    plots = chart(rows,'Predicted finger gap (mm): no gripper feedback', [('model_width_mm','#2563eb','model only')])
    plots += chart(rows,'Flange Z (mm)', [('desired_flange_z_mm','#2563eb','desired now'),('command_flange_z_mm','#059669','servo target with lookahead'),('measured_flange_z_mm','#dc2626','measured / simulated flange')])
    plots += chart(rows,'Estimated tip-height error using model gap (mm)', [('tip_error_mm','#dc2626','model gap + arm pose / simulated error')])
    (directory/'report.html').write_text('<!doctype html><html lang="en"><meta charset="utf-8"><title>TCP height test</title>'
        '<style>body{font:16px system-ui;max-width:1000px;margin:30px auto;padding:0 20px}svg{width:100%}svg text{font:12px system-ui}pre{white-space:pre-wrap;background:#f5f7fa;padding:15px}h2{font-size:20px}</style>'
        '<h1>Gripper / Z compensation test</h1><p>Predicted curves are a model, not proof of constant physical tip height. '
        'No gripper position or current is read. Real-mode estimated height combines a predicted gap with arm pose. '
        'A small plotted error does not prove the physical fingers kept their height. '
        'Model velocity/acceleration are not sent to the gripper. Mock and virtual traces are simulated.</p>'+plots+
        '<h2>Run metadata</h2><pre>'+html.escape(json.dumps(metadata,ensure_ascii=False,indent=2))+'</pre></html>')
