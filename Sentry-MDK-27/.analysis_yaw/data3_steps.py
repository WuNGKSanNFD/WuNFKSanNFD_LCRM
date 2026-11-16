# -*- coding: utf-8 -*-
"""Data3: quantify step-induced ringing and feedforward kick."""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PATH = r"G:\RM\00Sentry\采集数据\26.10.23 yaw跟随数据3.csv"
df = pd.read_csv(PATH)
y = df["I0"].to_numpy(float)
u = df["I1"].to_numpy(float)
n = len(y); fs=100.0; t=np.arange(n)/fs
du = np.diff(u)

# largest command steps
idx = np.argsort(np.abs(du))[::-1][:15]
print("largest steps (t, du, u_before, u_after):")
for i in sorted(idx):
    print(f"  {t[i]:6.2f}  {du[i]:7.2f}  {u[i]:8.2f} -> {u[i+1]:8.2f}")

# command step-rate distribution
print("du p50/p90/p99/max", np.percentile(np.abs(du),[50,90,99,100]))

# holds: find segments where u constant for >=0.3s, then measure ring after the preceding step
# simple: look at residual (y - 0.5s MA) after big steps
def ma(a,k): return np.convolve(a,np.ones(k)/k,mode='same')
res = y - ma(y, 201)

# choose a clean hold after a big step, e.g. around 7.4-9.4 at -55
for a,b,name in [(7.3,9.5,'hold_55'),(13.0,15.0,'hold_60'),(17.5,19.5,'hold_m8')]:
    ia,ib=int(a*fs),int(b*fs)
    seg=res[ia:ib]
    f=np.fft.rfftfreq(len(seg),1/fs); A=np.abs(np.fft.rfft(seg-seg.mean()))*2/len(seg)
    top=np.argsort(A[1:])[::-1][:6]+1
    print(name,"res std",seg.std(),"peaks",[(round(f[i],2),round(A[i],3)) for i in top])

# ring metrics after the step at ~5.0? find step near 5.0 where command holds
# print command around 4.8-5.4
for a,b in [(4.7,5.6),(7.6,8.4),(9.0,9.8),(12.0,13.0)]:
    ia,ib=int(a*fs),int(b*fs)
    print(f"--- {a}-{b} ---")
    for i in range(ia,ib,5):
        print(f"  {t[i]:5.2f} u={u[i]:8.2f} y={y[i]:8.2f} e={u[i]-y[i]:7.2f}")

# plot 4.5-6.0 and 7.5-9.5
for a,b,name in [(4.4,6.2,'d3_step1'),(7.4,9.4,'d3_hold1'),(9.0,11.0,'d3_hold2')]:
    ia,ib=int(a*fs),int(b*fs)
    fig,axs=plt.subplots(2,1,figsize=(13,6),sharex=True)
    axs[0].plot(t[ia:ib],u[ia:ib],'.-',ms=3,lw=1,label='cmd')
    axs[0].plot(t[ia:ib],y[ia:ib],'.-',ms=3,lw=1,label='meas')
    axs[0].legend(); axs[0].grid(True); axs[0].set_ylabel('deg')
    axs[1].plot(t[ia:ib],res[ia:ib],'.-',ms=3,lw=1)
    axs[1].grid(True); axs[1].set_ylabel('I0 - 1s MA'); axs[1].set_xlabel('s')
    plt.tight_layout(); plt.savefig(rf"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\{name}.png",dpi=120)

# Estimate feedforward pulses: assume cmd updates 5ms, gimbal 2ms; f_out = 40*du per update
print("feedforward pulse estimate: 40*du [rpm], for largest steps:")
for i in sorted(idx)[:8]:
    print(f"  du={du[i]:6.2f} deg -> f_out={40*du[i]:7.1f} rpm")
