# -*- coding: utf-8 -*-
"""Zoom and quantify the 26.10.23 yaw follow run."""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PATH = r"G:\RM\00Sentry\采集数据\26.10.23 yaw跟随数据1.csv"
df = pd.read_csv(PATH)
y = df["I0"].to_numpy(float)  # measurement (IMU yaw)
u = df["I1"].to_numpy(float)  # command
fs = 100.0
t = np.arange(len(y)) / fs

def zoom(a, b, name, fs_plot=100):
    ia, ib = int(a * fs), min(int(b * fs), len(y))
    fig, axs = plt.subplots(3, 1, figsize=(13, 7.5), sharex=True,
                            gridspec_kw={"height_ratios": [2, 1.2, 0.8]})
    axs[0].plot(t[ia:ib], u[ia:ib], "-o", ms=2, lw=1.0, label="command I1")
    axs[0].plot(t[ia:ib], y[ia:ib], "-o", ms=2, lw=1.0, label="measure I0")
    axs[0].set_ylabel("deg"); axs[0].legend(); axs[0].grid(True)
    axs[1].plot(t[ia:ib], u[ia:ib] - y[ia:ib], lw=0.9, color="crimson")
    axs[1].set_ylabel("error deg"); axs[1].grid(True)
    # command rate and measured rate (deg/s)
    du = np.gradient(u, 1 / fs)
    dy = np.gradient(y, 1 / fs)
    axs[2].plot(t[ia:ib], du[ia:ib], lw=0.9, label="d(cmd)/dt")
    axs[2].plot(t[ia:ib], dy[ia:ib], lw=0.9, label="d(meas)/dt")
    axs[2].set_ylabel("deg/s"); axs[2].legend(); axs[2].grid(True)
    axs[2].set_xlabel("time s (10 ms grid)")
    plt.tight_layout(); plt.savefig(rf"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\{name}.png", dpi=120)
    plt.close()

zoom(11.8, 15.0, "step_125")
zoom(29.5, 33.5, "seg_30")
zoom(33.0, 45.0, "sine_seg")
zoom(44.0, 46.45, "tail")

# The step at ~12.5 s
i0 = np.argmax(np.abs(np.diff(u)) > 10)
print("largest jump index", i0, "t", t[i0], "u", u[i0], "->", u[i0+1])
for k in range(i0-2, i0+4):
    print(k, t[k], u[k], y[k], u[k]-y[k])
# settle behavior after step
after = slice(i0+1, i0+1+int(1.0*fs))
print("step: min/max meas after", y[after].min(), y[after].max())
# dominant frequency of last segment (33-44)
seg = slice(int(33*fs), int(44*fs))
yy = y[seg] - y[seg].mean()
n = len(yy)
f = np.fft.rfftfreq(n, 1/fs)
Y = np.abs(np.fft.rfft(yy))
idx = np.argsort(Y[1:])[::-1][:8] + 1
print("top freq (meas 33-44s):", [(round(f[i],3), round(Y[i],1)) for i in idx])

# cross-correlation lag between command and measurement in the sine segment
uu = u[seg] - u[seg].mean()
xc = np.correlate(yy, uu, mode="full")
lags = np.arange(-len(uu)+1, len(uu)) / fs
# restrict +-0.5s
m = np.abs(lags) <= 0.5
lag = lags[m][np.argmax(xc[m])]
print("xcorr lag meas vs cmd (s):", lag, "deg:", lag*360*0)  # placeholder

# fit ARX models y[k] = sum a_i y[k-i] + sum b_j u[k-d-j]
def fit_arx(y, u, na, nb, d):
    N = len(y)
    rows = N - max(na, nb + d - 1)
    Phi = np.zeros((rows, na + nb))
    Yv = np.zeros(rows)
    for r in range(rows):
        k = max(na, nb + d - 1) + r
        Phi[r, :na] = y[k-np.arange(1, na+1)]
        Phi[r, na:] = u[k-d-np.arange(0, nb)]
        Yv[r] = y[k]
    theta, *_ = np.linalg.lstsq(Phi, Yv, rcond=None)
    yh = Phi @ theta
    e = Yv - yh
    print(f"ARX na={na} nb={nb} d={d}: fit={1-np.sum(e**2)/np.sum((Yv-Yv.mean())**2):.4f} rms={np.std(e):.4f}")
    return theta, e

for d in [0, 1, 2, 3, 4, 5]:
    fit_arx(y, u, 2, 2, d)
