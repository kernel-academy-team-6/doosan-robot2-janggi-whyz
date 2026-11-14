"""Uniform 1ms resampling; 101-point cubic smoothing separates motion ripple."""
import csv,gzip,json
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT=Path(__file__).parent
fig,axes=plt.subplots(1,2,figsize=(12,4),layout='constrained');report={}
for name,title,color in [('baseline_disturb','Before','#d34a43'),('final_disturb','After','#087b8e')]:
 path=OUT/(name+'.csv');opener=open
 if not path.exists():path=path.with_suffix('.csv.gz');opener=gzip.open
 with opener(path,'rt') as f:rows=list(csv.DictReader(f))
 wall=np.array([float(r['wall_s']) for r in rows]);sim=np.array([float(r['sim_s']) for r in rows]);sim,index=np.unique(sim,return_index=True)
 grid=np.arange(sim[0],sim[-1],.001);t=np.interp(grid,sim,wall[index]);moving=(t>5.2)&(t<7.8)
 metrics={}
 for j in range(1,7):
  q=np.interp(grid,sim,np.degrees([float(r[f'joint_{j}']) for r in rows])[index])
  ripple=q-savgol_filter(q,101,3)
  metrics[f'joint_{j}_ripple_rms_deg']=float(np.sqrt(np.mean(ripple[moving]**2)))
  if j==6:
   zoom=(t>6)&(t<6.4);axes[0].plot(t[zoom],ripple[zoom],color=color,label=title)
   pre=np.mean(q[(t>8.8)&(t<8.95)]);pulse=(t>8.8)&(t<10.5)
   axes[1].plot(t[pulse],q[pulse]-pre,color=color,label=title)
 result=json.loads((OUT/(name+'.json')).read_text());metrics['disturbance_peak_deg']=result['j6_disturbance_peak_deg'];report[title]=metrics
axes[0].set(xlabel='Test time [s]',ylabel='J6 high-frequency ripple [deg]',title='Moving ripple (same sinusoidal motion)')
axes[1].axvspan(9,9.5,color='grey',alpha=.15,label='2 Nm torque pulse');axes[1].set(xlabel='Test time [s]',ylabel='J6 displacement [deg]',title='Holding response to external torque')
for ax in axes:ax.grid(alpha=.2);ax.legend()
fig.savefig(OUT/'comparison.png',dpi=160)
(OUT/'comparison.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
