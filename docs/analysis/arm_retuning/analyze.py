"""Recompute the matched stress-test metrics from compressed telemetry."""
import csv,gzip,json
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter
OUT=Path(__file__).parent
result={}
for label in ['recent','previous','traj_gravity','traj_gravity_tuned']:
 rows=list(csv.DictReader(gzip.open(OUT/f'recovered_stress_{label}.csv.gz','rt')))
 move=np.array([r['phase']=='move' for r in rows]);v=np.degrees(np.array([[float(r[f'v{i}']) for i in range(1,7)] for r in rows]))
 # 1 kHz physics samples, 101 ms cubic Savitzky-Golay trend removal.
 ripple=v-savgol_filter(v,101,3,axis=0)
 result[label]={'moving_velocity_ripple_rms_deg_s':np.sqrt(np.mean(ripple[move]**2,axis=0)).tolist()}
(OUT/'ripple_metrics.json').write_text(json.dumps(result,indent=2)+'\n')
try:
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 fig,axes=plt.subplots(1,2,figsize=(11,4.3),layout='constrained')
 x=np.arange(6)
 for i,(label,title) in enumerate([('recent','Recent high gains'),('previous','Previous gains'),('traj_gravity_tuned','Retuned + gravity')]):
  axes[0].bar(x+(i-1)*.25,result[label]['moving_velocity_ripple_rms_deg_s'],.25,label=title)
  data=json.loads((OUT/'recovered_stress.json').read_text())['results'][label]
  axes[1].bar(x+(i-1)*.25,data['hold_peak_deg'],.25,label=title)
 for ax in axes:ax.set_xticks(x,[f'J{i}' for i in range(1,7)]);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
 axes[0].set(ylabel='Velocity ripple RMS (deg/s)',title='Moving: matched fast + feedback-gap test')
 axes[1].set(ylabel='Peak holding error (deg)',title='Hold: after 0.4 seconds settling')
 axes[0].legend(fontsize=8)
 fig.savefig(OUT/'comparison.png',dpi=160)
except ImportError:
 pass
print(json.dumps(result,indent=2))
