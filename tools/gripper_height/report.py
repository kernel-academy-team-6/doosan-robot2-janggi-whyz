"""Standalone circle-fit artifact; no robot settings are written."""
from dataclasses import asdict
import csv
import html
import json
import os
import numpy as np

from .model import fit_circle


def export_fit(session):
    fit = fit_circle(session)
    out = session.directory
    out.mkdir(parents=True,exist_ok=True)
    data = dict(schema_version=1,**asdict(fit),backend=session.backend,
                equation='(width_mm/2 - center_half_width_mm)^2 + (relative_z_mm - center_relative_z_mm)^2 = radius_mm^2',
                compensation='Z_new = Z_old + height_from_open(new_width) - height_from_open(old_width)',
                command_width_interpolation='piecewise linear measured width versus command',
                measured_width_range_mm=[min(s.width_mm for s in session.samples),max(s.width_mm for s in session.samples)],
                automatic_robot_compensation_applied=False)
    (out/'fit.json').write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    commands = np.array([s.command for s in session.samples][::-1])
    widths = np.array([s.width_mm for s in session.samples][::-1])
    with (out/'lookup.csv').open('w',newline='',encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['command','interpolated_width_mm','height_change_from_command_0_mm'])
        for command in range(701):
            width = float(np.interp(command,commands,widths))
            writer.writerow([command,width,fit.height_from_open(width)])
    with (out/'fit_residuals.csv').open('w',newline='',encoding='utf-8') as stream:
        writer=csv.writer(stream)
        writer.writerow(['command','width_mm','measured_relative_z_mm','circle_relative_z_mm','z_residual_mm'])
        for s in session.samples:
            observed=s.flange_pose[2]-session.reference_z
            predicted=fit.curve_z(s.width_mm)
            writer.writerow([s.command,s.width_mm,observed,predicted,observed-predicted])
    os.environ.setdefault('MPLCONFIGDIR','/tmp/janggi_gripper_height_matplotlib')
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(12,5),layout='constrained')
    measured=np.array([[s.width_mm,s.flange_pose[2]-session.reference_z] for s in session.samples])
    grid=np.linspace(measured[:,0].min(),measured[:,0].max(),300)
    axes[0].scatter(measured[:,0],measured[:,1],label='Measured: Z - Z(command 0)',zorder=3)
    axes[0].plot(grid,[fit.height_from_open(w) for w in grid],label='Circle height, anchored at command 0')
    axes[0].set(xlabel='Measured full jaw gap (mm)',ylabel='Relative flange Z (mm)',title='Height compensation versus gap')
    axes[0].grid(alpha=.3);axes[0].legend(fontsize=8)
    theta=np.linspace(0,2*np.pi,400)
    axes[1].plot(fit.center_half_width_mm+fit.radius_mm*np.cos(theta),
                 fit.center_relative_z_mm+fit.radius_mm*np.sin(theta),color='#94a3b8',label='Fitted circle')
    axes[1].scatter(measured[:,0]/2,measured[:,1],color='#2563eb',zorder=3,label='Measurements')
    axes[1].plot(fit.center_half_width_mm,fit.center_relative_z_mm,'r+',label='Center')
    axes[1].set(xlabel='Gap / 2 (mm)',ylabel='Z - Z(command 0) (mm)',title='Geometric circle fit')
    axes[1].set_aspect('equal');axes[1].grid(alpha=.3);axes[1].legend(fontsize=8,loc='upper left')
    fig.savefig(out/'fit.png',dpi=160);plt.close(fig)
    rows=''.join(f'<tr><td>{s.command}</td><td>{s.width_mm:.4f}</td><td>{s.flange_pose[2]:.4f}</td>'
                 f'<td>{s.flange_pose[2]-session.reference_z:.4f}</td></tr>' for s in session.samples)
    equation=f'(w/2 − ({fit.center_half_width_mm:.6f}))² + (h − ({fit.center_relative_z_mm:.6f}))² = {fit.radius_mm:.6f}²'
    warning='<p class="note">MOCK 측정 데이터입니다. 실제 하드웨어 보정값이 아닙니다.</p>' if session.backend=='mock' else ''
    (out/'report.html').write_text('''<!doctype html><html lang="ko"><meta charset="utf-8">
<title>그리퍼 열림·높이 보정 결과</title><style>body{font:16px system-ui;max-width:1100px;margin:30px auto;padding:0 20px;line-height:1.7}
img{width:100%}table{border-collapse:collapse;width:100%}td,th{padding:8px;border-bottom:1px solid #ddd;text-align:right}
pre{white-space:pre-wrap;background:#f1f5f9;padding:16px}.note{background:#fff3cd;padding:14px}</style>
<h1>그리퍼 열림·플랜지 높이 보정</h1>'''+warning+
        f'<p>명령값 0의 실제 플랜지 Z = {session.reference_z:.4f}mm, 기준 너비 = {fit.reference_width_mm:.4f}mm.</p>'
        '<p>w는 자로 측정한 전체 손가락 간격(mm), h는 각 플랜지 Z에서 명령값 0의 플랜지 Z를 뺀 상대 높이(mm)입니다.</p>'
        f'<pre>{html.escape(equation)}\n반지름: {fit.radius_mm:.6f}mm\n방사 방향 RMSE: {fit.radial_rmse_mm:.6f}mm'
        f'\n최대 방사 오차: {fit.radial_max_error_mm:.6f}mm\n가지 부호: {fit.branch:+d}</pre>'
        '<img src="fit.png" alt="높이 곡선과 원 피팅 그래프">'
        '<p>높이 보정 함수는 피팅한 원의 동일한 가지를 사용하고, 명령값 0의 측정 너비에서 값을 다시 빼서 기준 높이를 정확히 0으로 둡니다.</p>'
        '<pre>F(w) = center_z + branch × sqrt(R² − (w/2 − center_x)²)\n'
        'H(w) = F(w) − F(w_at_command_0)\nZ_new = Z_old + H(new_width) − H(old_width)</pre>'
        '<p>같은 바닥에 동일하게 접근했다는 조건에서 플랜지 높이 차이를 손가락 길이 차이로 해석합니다. '
        '바닥 접촉 직전의 판단·자로 잰 너비·관절 위치 오차가 포함되며 자동 접촉 검출 결과는 아닙니다. '
        '측정 너비 범위 밖으로 외삽하지 마세요. 기존 로봇/게임 설정에는 자동 적용하지 않았습니다.</p>'
        '<p>lookup.csv의 명령값별 너비는 실측 점 사이 선형 보간이며 모터의 실제 위치 피드백이 아닙니다.</p>'
        '<table><tr><th>그리퍼 명령</th><th>실측 너비 mm</th><th>플랜지 Z mm</th><th>상대 높이 mm</th></tr>'+
        rows+'</table>',encoding='utf-8')
    return fit
