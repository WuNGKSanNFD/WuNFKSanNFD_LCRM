# -*- coding: utf-8 -*-
"""Explore data2: static command 0, suspected high-frequency jitter/drift."""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PATH = r"G:\RM\00Sentry\采集数据\26.10.23 yaw跟随数据2.csv"
df = pd.read_csv(PATH)
y = df["I0"].to_numpy(float)
u = df["I1"].to_numpy(float)
n = len(y)
fs = 100.0
t = np.arange(n)/fs
print("rows", n, "duration @100Hz", n/fs)
print("u unique", np.unique(u)[:10], "count", len(np.unique(u)))
print("y min/max/mean/std", y.min(), y.max(), y.mean(), y.std())
print("first 10 y", y[:10])
print("first 10 u", u[:10])
dy = np.diff(y)
print("dy std", dy.std(), "max abs dy", np.abs(dy).max(), "median abs dy", np.median(np.abs(dy)))
# remove slow trend with moving average 1s
k = 101
trend = np.convolve(y, np.ones(k)/k, mode="same")
res = y - trend
print("residual std after 1s MA", res.std())
# spectrum of residual (skip edges)
seg = res[200:-200]
f = np.fft.rfftfreq(len(seg), 1/fs)
Y = np.abs(np.fft.rfft(seg))/len(seg)*2
top = np.argsort(Y[1:])[::-1][:15]+1
print("top freqs residual:", [(round(f[i],2), round(Y[i],4)) for i in top])

fig, axs = plt.subplots(3,1,figsize=(14,10),sharex=True)
axs[0].plot(t,y,lw=0.8,label='I0 (IMU yaw)')
axs[0].plot(t,u,lw=0.8,label='I1 (cmd)')
axs[0].legend(); axs[0].grid(True); axs[0].set_ylabel('deg')
axs[1].plot(t,res,lw=0.6,label='I0 - 1s MA')
axs[1].legend(); axs[1].grid(True); axs[1].set_ylabel('deg')
axs[2].semilogy(f, Y+1e-12)
axs[2].set_xlim(0,50); axs[2].grid(True); axs[2].set_xlabel('Hz'); axs[2].set_ylabel('mag')
plt.tight_layout(); plt.savefig(r"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\data2_overview.png", dpi=120)

# zooms 0-1s, 10-11s, 30-31s
for a,b,name in [(0,1,'d2_z0'),(10,11,'d2_z10'),(20,21,'d2_z20'),(30,31,'d2_z30'),(36,38,'d2_z36')]:
    ia,ib=int(a*fs),min(int(b*fs),n)
    fig, axs = plt.subplots(2,1,figsize=(12,6),sharex=True)
    axs[0].plot(t[ia:ib],y[ia:ib],'.-',ms=3,lw=0.8,label='I0')
    axs[0].plot(t[ia:ib],u[ia:ib],'.-',ms=3,lw=0.8,label='I1')
    axs[0].legend(); axs[0].grid(True); axs[0].set_ylabel('deg')
    axs[1].plot(t[ia:ib],np.diff(y[ia-1:ib]) if ia>0 else np.diff(y[ia:ib]),'.-',ms=3,lw=0.8)
    axs[1].grid(True); axs[1].set_ylabel('sample delta deg'); axs[1].set_xlabel('s')
    plt.tight_layout(); plt.savefig(rf"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\{name}.png", dpi=120)
