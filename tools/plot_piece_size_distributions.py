"""Descriptive size distributions; correlated frames are not iid test samples."""
from pathlib import Path
import csv,json,collections
import numpy as np
from scipy.stats import norm,skew,kurtosis
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
root=Path(__file__).resolve().parents[1];folder=root/'docs/analysis/normal_piece_sizes'
rows=list(csv.DictReader((folder/'normal_observations.csv').open()))
groups=[('king','궁',{'king'}),('medium','중간말',{'cha','po','horse','sang'}),('small','작은말',{'sa','soldier'})]
fontpath=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
if fontpath.exists():plt.rcParams['font.family']=FontProperties(fname=str(fontpath)).get_name()
plt.rcParams['axes.unicode_minus']=False
fig,axes=plt.subplots(3,2,figsize=(12,10),layout='constrained')
fig.suptitle('정상 관측 크기: 실제 분포와 정규분포 비교',fontsize=17)
report=[]
for i,(key,name,kinds) in enumerate(groups):
 selected=[r for r in rows if r['kind'] in kinds];v=np.array([float(r['geometric_size_mm']) for r in selected]);mu=v.mean();sd=v.std(ddof=1)
 ax=axes[i,0];color=['#9361a9','#158e91','#377ac1'][i]
 ax.hist(v,bins='fd',density=True,color=color,alpha=.65,label='실제 관측 히스토그램')
 xx=np.linspace(v.min()-.15,v.max()+.15,400)
 ax.plot(xx,norm.pdf(xx,mu,sd),color='#bd583c',lw=2,label='같은 평균·표준편차의 정규분포')
 ax.set(title=f'{name} · n={len(v):,}',xlabel='박스 크기 √(가로×세로) (mm)',ylabel='확률밀도 (1/mm)')
 ax.grid(axis='y',alpha=.15);ax.spines[['top','right']].set_visible(False)
 if i==0:ax.legend(fontsize=9)
 ps=np.linspace(.001,.999,500);theory=norm.ppf(ps);observed=np.quantile((v-mu)/sd,ps)
 ax=axes[i,1];ax.scatter(theory,observed,s=8,color=color,alpha=.7)
 ax.plot([-3.1,3.1],[-3.1,3.1],color='#bd583c',lw=1.5,label='정규분포일 때의 기준선')
 ax.set(title=f'{name} · Q–Q 비교',xlabel='정규분포의 이론적 분위수',ylabel='실제 표준화 크기의 분위수')
 ax.grid(alpha=.15);ax.spines[['top','right']].set_visible(False)
 track=collections.defaultdict(list)
 for r in selected:track[(r['piece_id'],r['label'])].append(float(r['geometric_size_mm']))
 means=[{'piece_id':pid,'label':label,'n':len(x),'mean_mm':float(np.mean(x)),'std_mm':float(np.std(x,ddof=1)) if len(x)>1 else None} for (pid,label),x in track.items()]
 item=dict(group=key,group_ko=name,n=len(v),mean_mm=float(mu),std_mm=float(sd),skewness=float(skew(v,bias=False)),excess_kurtosis=float(kurtosis(v,bias=False)),within_sigma={str(k):float(np.mean(np.abs(v-mu)<=k*sd)) for k in [1,2,3,6]},per_track=means)
 report.append(item)
 print(name,'skew',round(item['skewness'],3),'excess kurtosis',round(item['excess_kurtosis'],3),'coverage',item['within_sigma'])
 if key=='king':print('King tracks:',means)
fig.savefig(folder/'normality_diagnostics.png',dpi=150)
(folder/'normality_diagnostics.json').write_text(json.dumps({'groups':report,'method':'Descriptive histogram and normal Q-Q comparison; no iid normality-test p-values because consecutive frames are correlated.','limitations':'Selected normal frames; model-predicted labels; one recording and calibration. Tail probabilities beyond the sample are not established.'},ensure_ascii=False,indent=2))
print('saved',folder/'normality_diagnostics.png')
