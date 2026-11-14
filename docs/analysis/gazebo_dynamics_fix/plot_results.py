"""Render the measured joint tracking and CAD gap; no AI-generated data."""
from pathlib import Path
import csv, gzip, json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT=Path(__file__).parent
fig,axes=plt.subplots(1,2,figsize=(12,4.2),layout='constrained')
summary={}
for label,title,color in [('before','Before','#d34a43'),('final_verified','Corrected','#087b8e')]:
 path=OUT/(label+'.csv');opener=open
 if not path.exists():path=path.with_suffix('.csv.gz');opener=gzip.open
 with opener(path,'rt') as f:rows=list(csv.DictReader(f))
 t=np.array([float(r['wall_s']) for r in rows]);q=np.degrees([float(r['joint_6']) for r in rows]);target=np.degrees([float(r['joint_6_target']) for r in rows])
 axes[0].plot(t,q-target,label=title,color=color,lw=1.1)
 idx=(t>5.2)&(t<7.8);summary[label]={'j6_moving_rmse_deg':float(np.sqrt(np.mean((q[idx]-target[idx])**2)))}
axes[0].set(xlabel='Time after test start [s]',ylabel='J6 tracking error [deg]',title='Same 20 Hz trajectory, bounded force control',xlim=(4.8,10));axes[0].legend();axes[0].grid(alpha=.25)
q=np.linspace(0,1.119189125709,300);gap=2*(.008+.0493634*np.cos(q)-.0285*np.sin(q)-.0039)*1000
axes[1].plot(np.degrees(q),gap,color='#087b8e')
for angle,name in [(1.,'Old 700 mapping'),(1.101,'Old URDF limit'),(1.119189125709,'CAD closure')]:
 y=2*(.008+.0493634*np.cos(angle)-.0285*np.sin(angle)-.0039)*1000
 axes[1].scatter(np.degrees(angle),y);axes[1].annotate(f'{name}\n{y:.2f} mm',(np.degrees(angle),y),xytext=(-12,15 if angle==1 else 55 if angle==1.101 else 90),textcoords='offset points',ha='right',fontsize=8,arrowprops={'arrowstyle':'-','alpha':.5})
axes[1].set(xlabel='Linkage angle [deg]',ylabel='Inner flat-face gap [mm]',title='Gap from supplied STL and linkage origins');axes[1].grid(alpha=.25)
fig.savefig(OUT/'tracking_and_gap.png',dpi=160)
with gzip.open(OUT/'contact_verified.csv.gz','rt') as f:contact=list(csv.DictReader(f))
tail=[r for r in contact if float(r['wall_s'])>9]
joints=['gripper_rh_r1','gripper_rh_r2','gripper_rh_l1','gripper_rh_l2']
summary['contact_max_pair_spread_deg']=max(np.ptp([float(r[j]) for j in joints])*180/np.pi for r in tail)
(OUT/'comparison.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
