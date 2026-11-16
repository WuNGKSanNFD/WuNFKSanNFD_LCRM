# -*- coding: utf-8 -*-
"""Data3: rollback + f_kp=40. Find vibration frequency/amplitude and source."""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PATH = r"G:\RM\00Sentry\采集数据\26.10.23 yaw跟随数据3.csv"
df = pd.read_csv(PATH)
y = df["I0"].to_numpy(float)
u = df["I1"].to_numpy(float)
n = len(y); fs = 100.0; t = np.arange(n)/fs
print("rows", n, "duration", n/fs)
print("u unique first/last", u[0], u[-1], "n unique", len(np.unique(u)))
print("y range", y.min(), y.max())
# find command changes
du = np.diff(u)
idx = np.where(np.abs(du) > 1e-9)[0]
print("command changes:", [(round(t[i],2), u[i], u[i+1]) for i in idx[:20]], "total", len(idx))

# segments
seg_static = (t > 1) & (t < t[idx[0]]-0.1) if len(idx) else (t>1)&(t<n/fs-1)
seg_move = t > (t[idx[0]] if len(idx) else 0)
print("static duration", t[seg_static].min() if seg_static.any() else None, t[seg_static].max() if seg_static.any() else None)

# residual after 1s MA, spectrum for static and whole
def ma(a,k): return np.convolve(a,np.ones(k)/k,mode="same")
def spec(a, fs):
    a = a - a.mean()
    f = np.fft.rfftfreq(len(a),1/fs)
    A = np.abs(np.fft.rfft(a))/len(a)*2
    return f,A
for name, s in [("static",seg_static),("all",np.ones(n,bool)),("move",seg_move)]:
    yy=y[s]
    if len(yy)<50: continue
    res=yy-ma(yy,101)
    f,A=spec(res,fs)
    top=np.argsort(A[1:])[::-1][:10]+1
    print(name, "n",len(yy),"res std",res.std(),"peaks",[(round(f[i],2),round(A[i],3)) for i in top])

fig,axs=plt.subplots(4,1,figsize=(14,12),sharex=True)
axs[0].plot(t,y,lw=0.8,label='I0'); axs[0].plot(t,u,lw=0.8,label='I1')
axs[0].legend(); axs[0].grid(True); axs[0].set_ylabel('deg')
res=y-ma(y,101)
axs[1].plot(t,res,lw=0.6); axs[1].grid(True); axs[1].set_ylabel('I0 - 1s MA')
dy=np.diff(y)
axs[2].plot(t[1:],dy,lw=0.5); axs[2].grid(True); axs[2].set_ylabel('delta deg/10ms')
# spectrogram-ish: sliding 1s windows, dominant 5-50Hz amplitude
win=100; step=10
freqs=np.fft.rfftfreq(win,1/fs)
amp=np.zeros((len(range(0,n-win,step)),len(freqs)))
for j,i in enumerate(range(0,n-win,step)):
    seg=res[i:i+win]*np.hanning(win)
    amp[j]=np.abs(np.fft.rfft(seg))/win*2
im=axs[3].pcolormesh(np.arange(len(amp))*step/fs, freqs, amp.T, shading='auto', cmap='inferno')
axs[3].set_ylim(0,50); axs[3].set_xlabel('s'); axs[3].set_ylabel('Hz')
fig.colorbar(im,ax=axs[3])
plt.tight_layout(); plt.savefig(r"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\d3_overview.png",dpi=120)

# zoom static and move
for a,b,name in [(2,3,'d3_z2'),(10,11,'d3_z10'),(16,18,'d3_z16'),(20,22,'d3_z20')]:
    ia,ib=int(a*fs),min(int(b*fs),n)
    fig,axs=plt.subplots(2,1,figsize=(12,6),sharex=True)
    axs[0].plot(t[ia:ib],y[ia:ib],'.-',ms=3,lw=0.8,label='I0')
    axs[0].plot(t[ia:ib],u[ia:ib],'.-',ms=3,lw=0.8,label='I1')
    axs[0].legend(); axs[0].grid(True); axs[0].set_ylabel('deg')
    axs[1].plot(t[1:][ia:ib],dy[ia:ib],'.-',ms=3,lw=0.8)
    axs[1].grid(True); axs[1].set_ylabel('delta deg/10ms'); axs[1].set_xlabel('s')
    plt.tight_layout(); plt.savefig(rf"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\{name}.png",dpi=120)
