# -*- coding: utf-8 -*-
"""Zoom ramp starts / small-signal transient, fit 2nd-order ring."""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PATH = r"G:\RM\00Sentry\采集数据\26.10.23 yaw跟随数据1.csv"
df = pd.read_csv(PATH)
y = df["I0"].to_numpy(float)
u = df["I1"].to_numpy(float)
fs = 100.0
t = np.arange(len(y)) / fs
err = u - y

def plot(a, b, name):
    m = (t >= a) & (t <= b)
    fig, axs = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
    axs[0].plot(t[m], u[m], ".-", ms=4, label="cmd")
    axs[0].plot(t[m], y[m], ".-", ms=4, label="meas")
    axs[0].legend(); axs[0].grid(True); axs[0].set_ylabel("deg")
    axs[1].plot(t[m], err[m], ".-", ms=4, color="crimson")
    axs[1].grid(True); axs[1].set_ylabel("err deg")
    ru = np.gradient(u, 1/fs); ry = np.gradient(y, 1/fs)
    axs[2].plot(t[m], ru[m], ".-", ms=4, label="cmd rate")
    axs[2].plot(t[m], ry[m], ".-", ms=4, label="meas rate")
    axs[2].legend(); axs[2].grid(True); axs[2].set_ylabel("deg/s")
    plt.tight_layout(); plt.savefig(rf"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\{name}.png", dpi=120)
    plt.close()

plot(33.9, 34.8, "rampstart_341")
plot(35.9, 36.7, "reversal_3605")
plot(31.05, 32.2, "impulse_3115")

# fit decay of impulse: find peaks of y after 31.1
m = (t >= 31.08) & (t <= 32.2)
ts, ys = t[m], y[m]
# local maxima/minima
from numpy import diff, sign
ext = []
for i in range(1, len(ys)-1):
    if (ys[i] > ys[i-1] and ys[i] >= ys[i+1]) or (ys[i] < ys[i-1] and ys[i] <= ys[i+1]):
        ext.append((ts[i], ys[i]))
print("extrema:")
for e in ext[:14]:
    print(f"  {e[0]:.3f}  {e[1]:.3f}")
# extrema period
peaks = [e for e in ext if e[1] > 0]
for i in range(1, len(peaks)):
    print("period", peaks[i][0]-peaks[i-1][0], "amp ratio", peaks[i][1]/peaks[i-1][1])
