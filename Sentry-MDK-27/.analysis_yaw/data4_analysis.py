# -*- coding: utf-8 -*-
"""Data4: sine-sweep follow. Error/rate/spectrum, remaining HF content."""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PATH = r"G:\RM\00Sentry\采集数据\26.10.23 yaw跟随数据4.csv"
df = pd.read_csv(PATH)
y = df["I0"].to_numpy(float); u = df["I1"].to_numpy(float)
n=len(y); fs=100.0; t=np.arange(n)/fs
err = u - y
du = np.diff(u)
print("rows",n,"duration",n/fs)
print("y range",y.min(),y.max(),"u range",u.min(),u.max())
print("err mean/std/max",err.mean(),err.std(),np.max(np.abs(err)))
print("du p50/p90/p99/max",np.percentile(np.abs(du),[50,90,99,100]))

# command rate per 10ms; feedforward pulse estimate f_kp=20 * du (10ms diff; actual 5ms would be half)
print("f_out=20*du estimate: p90/max",20*np.percentile(np.abs(du),90),20*np.abs(du).max())

# constant-command holds (du==0) longer than 0.2s
mask=np.abs(du)<1e-9
# find runs
runs=[]; start=None
for i,v in enumerate(mask):
    if v and start is None: start=i
    if not v and start is not None:
        if i-start>20: runs.append((start,i))
        start=None
if start is not None and len(mask)-start>20: runs.append((start,len(mask)))
print("holds:",[(round(t[a],2),round(t[b],2),round(u[a],2)) for a,b in runs])
for a,b in runs:
    if b-a<20: continue
    s=slice(a+5,b-2)  # skip 50ms settle
    print(f"  hold {t[a]:5.2f}-{t[b]:5.2f} u={u[a]:7.2f} err_mean={err[s].mean():6.3f} err_std={err[s].std():6.3f}")

# spectrum of error during the whole active run (1.0-16.3)
s=(t>1.0)&(t<16.3)
ee=err[s]-err[s].mean()
f=np.fft.rfftfreq(len(ee),1/fs); A=np.abs(np.fft.rfft(ee))*2/len(ee)
top=np.argsort(A[1:])[::-1][:12]+1
print("error spectrum peaks:",[(round(f[i],2),round(A[i],2)) for i in top])
# HF energy >10Hz from amplitude spectrum
hf=np.sqrt(np.sum(A[f>10]**2)/2) if np.any(f>10) else 0
print("err rms total",ee.std(),"HF>10Hz rms",hf)

# rate error and voltage-step proxy: speed target = 9*err + 20*du
wc_est = 9*err[1:] + 20*du
print("wc_est p90/max",np.percentile(np.abs(wc_est),90),np.abs(wc_est).max())

fig,axs=plt.subplots(4,1,figsize=(14,12),sharex=True)
axs[0].plot(t,y,lw=0.8,label='I0'); axs[0].plot(t,u,lw=0.8,label='I1')
axs[0].legend(); axs[0].grid(True); axs[0].set_ylabel('deg')
axs[1].plot(t,err,lw=0.6); axs[1].grid(True); axs[1].set_ylabel('err deg')
axs[2].plot(t[1:],du,lw=0.6); axs[2].grid(True); axs[2].set_ylabel('du deg/10ms')
# spectrogram of error
win=200; step=10; freqs=np.fft.rfftfreq(win,1/fs); amp=[]
for i in range(0,n-win,step):
    seg=err[i:i+win]-np.mean(err[i:i+win]); seg=seg*np.hanning(win)
    amp.append(np.abs(np.fft.rfft(seg))/win*2)
amp=np.array(amp)
im=axs[3].pcolormesh(np.arange(len(amp))*step/fs,freqs,amp.T,shading='auto',cmap='inferno')
axs[3].set_ylim(0,50); axs[3].set_xlabel('s'); axs[3].set_ylabel('Hz'); fig.colorbar(im,ax=axs[3])
plt.tight_layout(); plt.savefig(r"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\d4_overview.png",dpi=120)

# zooms of likely noise: 0-1.4 hold and a reversal
for a,b,name in [(0.2,1.4,'d4_z0'),(1.3,2.0,'d4_z1'),(8.0,9.0,'d4_z8'),(10.5,11.0,'d4_z10')]:
    ia,ib=int(a*fs),min(int(b*fs),n)
    fig,axs=plt.subplots(3,1,figsize=(12,8),sharex=True)
    axs[0].plot(t[ia:ib],y[ia:ib],'.-',ms=3,lw=0.8,label='I0'); axs[0].plot(t[ia:ib],u[ia:ib],'.-',ms=3,lw=0.8,label='I1')
    axs[0].legend(); axs[0].grid(True); axs[0].set_ylabel('deg')
    axs[1].plot(t[ia:ib],err[ia:ib],'.-',ms=3,lw=0.8); axs[1].grid(True); axs[1].set_ylabel('err deg')
    axs[2].plot(t[1:][ia:ib],du[ia:ib],'.-',ms=3,lw=0.8); axs[2].grid(True); axs[2].set_ylabel('du deg/10ms'); axs[2].set_xlabel('s')
    plt.tight_layout(); plt.savefig(rf"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\{name}.png",dpi=120)
