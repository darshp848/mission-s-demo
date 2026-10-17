"""Render an eight-times-slower view of actual 1 kHz valve telemetry."""
from pathlib import Path
import csv,json,subprocess
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[1];out=root/'out/aero_pulsed'
with (out/'pulse_telemetry.csv').open() as f:rows=list(csv.DictReader(f))
d={k:np.array([float(r[k]) for r in rows]) for k in rows[0]}
fig,axes=plt.subplots(3,1,figsize=(12.8,7.2),dpi=100,facecolor='#0b0d10',gridspec_kw={'height_ratios':[1.4,1,1]})
fig.subplots_adjust(left=.09,right=.97,bottom=.10,top=.85,hspace=.4)
t=d['t'];start=5.0;end=5.5;mask=(t>=start)&(t<=end)
for ax in axes:
 ax.set_facecolor('#14181d');ax.tick_params(colors='#d0d5dc');ax.grid(color='#2a3038',alpha=.7)
 for spine in ax.spines.values():spine.set_color('#46505b')
 ax.set_xlim(start,end);ax.set_ylim(-.08,1.12)
 ax.set_ylabel('Thrust fraction',color='white')
for ax,name in zip(axes,['BL_push','FL_push','FR_push']):
 ax.step(t[mask],d['valve_'+name][mask],where='post',color='#ffcc80',lw=1.7,label='Valve: fixed thrust ON / OFF')
 ax.plot(t[mask],d['delivered_'+name][mask],color='#81c784',lw=1.7,label='Delivered thrust after lag')
 ax.set_title(name.replace('_',' ')+' — measured model commands',color='white',loc='left',fontsize=11)
axes[0].legend(facecolor='#14181d',edgecolor='none',labelcolor='white',fontsize=9,loc='upper right')
axes[-1].set_xlabel('Simulation time (s)',color='white')
cursors=[a.axvline(start,color='#4fc3f7',lw=2) for a in axes]
title=fig.text(.09,.945,'PULSED RCS | 8× SLOW MOTION',color='white',fontsize=18,weight='bold')
clock=fig.text(.09,.89,'',color='#4fc3f7',fontsize=12)
fig.text(.09,.025,'20 ms minimum firing; 10 ms minimum off; 5 ms command latency; 30 ms thrust lag (assumed). Chemical RCS model.',color='#b5bdc9',fontsize=9)
fps=24;N=round((end-start)*8*fps)
cmd=['ffmpeg','-y','-v','error','-f','rawvideo','-vcodec','rawvideo','-pix_fmt','rgb24','-s','1280x720','-r',str(fps),'-i','-','-an','-c:v','libx264','-pix_fmt','yuv420p','-crf','18',str(out/'pulse_slow_motion.mp4')]
with subprocess.Popen(cmd,stdin=subprocess.PIPE) as enc:
 for k in range(N):
  now=start+(end-start)*k/(N-1)
  for c in cursors:c.set_xdata([now,now])
  clock.set_text(f'Simulation T+{now:.3f} s   |   cyan cursor follows actual 1 kHz log')
  fig.canvas.draw();pixels=np.asarray(fig.canvas.buffer_rgba())[:,:,:3].copy();enc.stdin.write(pixels.tobytes())
  if k==N//2:fig.savefig(out/'pulse_closeup.png',facecolor=fig.get_facecolor())
 enc.stdin.close();assert enc.wait()==0
plt.close(fig)
print('Wrote pulse_slow_motion.mp4',N,'frames')
