# -*- coding: utf-8 -*-
"""Data2: sustained oscillation analysis, alias check, drift."""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PATH = r"G:\RM\00Sentry\采集数据\26.10.23 yaw跟随数据2.csv"
df = pd.read_csv(PATH)
y = df["I0"].to_numpy(float)
u = df["I1"].to_numpy(float)
n = len(y); fs = 100.0; t = np.arange(n)/fs

# drop first sample (reference jump)
y = y[1:]; u = u[1:]; n = len(y); t = np.arange(n)/fs

# slow trend (2 s MA) and residual
def ma(a, k):
    return np.convolve(a, np.ones(k)/k, mode="same")
trend = ma(y, 201)
res = y - trend
print("slow trend start/end", trend[0], trend[-1], "delta", trend[-1]-trend[0])
print("residual std", res.std(), "p2p", res.max()-res.min())

# bandpass 6-9 Hz via FFT
def bandpass(a, f1, f2, fs):
    A = np.fft.rfft(a)
    f = np.fft.rfftfreq(len(a), 1/fs)
    A[(f < f1) | (f > f2)] = 0
    return np.fft.irfft(A, len(a))
osc = bandpass(res, 6.0, 9.0, fs)
env = np.abs(osc)
# envelope via Hilbert-ish: use peak per 0.5s window
w = 50
env2 = np.array([np.max(np.abs(osc[i:i+w])) for i in range(0, n-w, w)])
print("osc std", osc.std(), "max", np.abs(osc).max())
print("envelope max per 0.5s (first 20):", np.round(env2[:20],3))
print("envelope mean last half:", env2[len(env2)//2:].mean(), "first half:", env2[:len(env2)//2].mean())

# zero crossings of osc to estimate frequency
zc = np.where(np.diff(np.signbit(osc)))[0]
if len(zc) > 10:
    periods = np.diff(zc)/fs*2  # half-cycle*2
    print("osc freq from zero crossings median:", 1/np.median(periods), "mean:", 1/periods.mean())

# sample delta stats (excluding first)
dy = np.diff(y)
print("dy abs median", np.median(np.abs(dy)), "p95", np.percentile(np.abs(dy),95), "max", np.abs(dy).max())

fig, axs = plt.subplots(4,1,figsize=(14,12),sharex=True)
axs[0].plot(t, y, lw=0.8, label='I0')
axs[0].plot(t, trend, lw=1.2, label='2s MA')
axs[0].legend(); axs[0].grid(True); axs[0].set_ylabel('deg')
axs[1].plot(t, res, lw=0.6, label='I0 - 2s MA')
axs[1].plot(t, osc, lw=0.8, label='6-9 Hz band')
axs[1].legend(); axs[1].grid(True); axs[1].set_ylabel('deg')
axs[2].plot(np.arange(len(env2))*0.5, env2, '.-')
axs[2].grid(True); axs[2].set_ylabel('6-9Hz peak per 0.5s')
axs[3].plot(t[1:], dy, lw=0.5)
axs[3].grid(True); axs[3].set_ylabel('sample delta deg'); axs[3].set_xlabel('s')
plt.tight_layout(); plt.savefig(r"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\d2_osc.png", dpi=120)

# zooms
for a,b,name in [(2,3,'d2_z2'),(10,11,'d2_z10'),(20,21,'d2_z20'),(30,31,'d2_z30'),(36,38,'d2_z36')]:
    ia,ib=int(a*fs),min(int(b*fs),n)
    fig, axs = plt.subplots(2,1,figsize=(12,6),sharex=True)
    axs[0].plot(t[ia:ib],y[ia:ib],'.-',ms=3,lw=0.8,label='I0')
    axs[0].plot(t[ia:ib],u[ia:ib],'.-',ms=3,lw=0.8,label='I1')
    axs[0].legend(); axs[0].grid(True); axs[0].set_ylabel('deg')
    dd = np.diff(y)[ia:ib]
    axs[1].plot(t[ia:ib],dd,'.-',ms=3,lw=0.8)
    axs[1].grid(True); axs[1].set_ylabel('delta deg per 10ms'); axs[1].set_xlabel('s')
    plt.tight_layout(); plt.savefig(rf"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\{name}.png", dpi=120)
