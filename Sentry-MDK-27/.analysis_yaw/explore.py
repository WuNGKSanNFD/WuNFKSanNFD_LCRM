# -*- coding: utf-8 -*-
"""Explore 26.10.23 yaw follow data (I0,I1)."""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PATH = r"G:\RM\00Sentry\采集数据\26.10.23 yaw跟随数据1.csv"
df = pd.read_csv(PATH)
x = df["I0"].to_numpy(dtype=float)   # candidate measurement
y = df["I1"].to_numpy(dtype=float)   # candidate command
n = len(df)
print("rows", n)
print("I0 stats", np.nanmin(x), np.nanmax(x), np.nanmean(x), np.nanstd(x))
print("I1 stats", np.nanmin(y), np.nanmax(y), np.nanmean(y), np.nanstd(y))
print("corr", np.corrcoef(x, y)[0, 1])
print("mean abs diff I1-I0", np.nanmean(np.abs(y - x)))
print("first 10 I0", x[:10])
print("first 10 I1", y[:10])
print("I1 unique count", len(np.unique(y)), "I0 unique", len(np.unique(x)))

# command increments
dy = np.diff(y)
print("I1 diff nonzero count", np.count_nonzero(dy), "max", dy.max(), "min", dy.min())
print("I1 diff values", pd.Series(dy[dy != 0]).value_counts().head(20))
# time index
t = np.arange(n) * 0.01
fig, axs = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
axs[0].plot(t, x, lw=0.7, label="I0")
axs[0].plot(t, y, lw=1.0, label="I1")
axs[0].legend(); axs[0].set_ylabel("deg"); axs[0].grid(True)
axs[1].plot(t, y - x, lw=0.7, color="crimson")
axs[1].set_ylabel("I1-I0 (deg)"); axs[1].grid(True)
axs[2].plot(t, x, lw=0.7)
axs[2].set_xlabel("time (s, assumed 10 ms)"); axs[2].grid(True)
plt.tight_layout(); plt.savefig(r"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\overview.png", dpi=110)

# zoom 0-2 s, 10-12 s, 40-42s
for a, b, name in [(0, 2, "zoom0"), (10, 12, "zoom10"), (30, 32, "zoom30"), (44, 46.45, "zoom44")]:
    ia, ib = int(a / 0.01), min(int(b / 0.01), n)
    fig, axs = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    axs[0].plot(t[ia:ib], x[ia:ib], ".-", ms=2, lw=0.8, label="I0")
    axs[0].plot(t[ia:ib], y[ia:ib], ".-", ms=2, lw=0.8, label="I1")
    axs[0].legend(); axs[0].grid(True); axs[0].set_ylabel("deg")
    axs[1].plot(t[ia:ib], y[ia:ib] - x[ia:ib], ".-", ms=2, lw=0.8)
    axs[1].grid(True); axs[1].set_ylabel("err"); axs[1].set_xlabel("s")
    plt.tight_layout(); plt.savefig(rf"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\{name}.png", dpi=120)

# segments: command nearly constant vs changing
d = np.abs(dy)
quiet = d == 0
print("quiet fraction", quiet.mean())
# noise of I0 in quiet segments
# investigate I0 noise: use diff
dx = np.diff(x)
print("I0 diff std", np.std(dx), "I1 diff std", np.std(dy))
