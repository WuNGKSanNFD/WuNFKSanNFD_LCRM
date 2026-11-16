# -*- coding: utf-8 -*-
"""Quantify ramp following error, velocity-loop tracking and ripple."""
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
# analysis window: control active 31.5 .. 44.5
m = (t >= 31.5) & (t <= 44.5)
tt, yy, uu = t[m], y[m], u[m]
rate_u = np.gradient(uu, 1/fs)
rate_y = np.gradient(yy, 1/fs)
err = uu - yy

# rolling mean to remove quantization/noise
def roll(a, n):
    k = np.ones(n) / n
    return np.convolve(a, k, mode="same")
n = 10  # 100 ms
ru = roll(rate_u, n)
ry = roll(rate_y, n)
re = roll(err, n)

# ramp plateaus: |ru|>150 and |d ru| small
acc = np.gradient(ru, 1/fs)
ramp = (np.abs(ru) > 150) & (np.abs(acc) < 1500)
print("ramp samples", ramp.sum())
# bin by rate
bins = np.linspace(-700, 700, 29)
idx = np.digitize(ru, bins)
print("rate_bin_center  mean_err  n")
for b in range(1, len(bins)):
    sel = ramp & (idx == b)
    if sel.sum() > 10:
        print(f"{(bins[b-1]+bins[b])/2:8.1f}  {np.mean(err[sel]):8.2f}  {sel.sum()}")

# linear fit e = rate/Kv + c on ramp plateaus
A = np.vstack([ru[ramp], np.ones(ramp.sum())]).T
coef, *_ = np.linalg.lstsq(A, err[ramp], rcond=None)
print("fit err = a*rate + c: a=", coef[0], "Kv=", 1/coef[0] if coef[0] else None, "c=", coef[1])
pred = A @ coef
print("rms resid", np.std(err[ramp] - pred))

# velocity loop: target speed = 9*err (rpm), measured speed = rate_y/6
target_rpm = 9 * re
meas_rpm = ry / 6.0
sel = (np.abs(target_rpm) > 10) & (np.abs(target_rpm) < 60)  # avoid saturation region
A2 = np.vstack([target_rpm[sel], np.ones(sel.sum())]).T
coef2, *_ = np.linalg.lstsq(A2, meas_rpm[sel], rcond=None)
print("velocity tracking: meas = a*target + b", coef2, "n", sel.sum())

# spectrum of error during holds (31.5-34) and ramps (34-44)
for name, sl in [("hold_31.5-33.5", (tt >= 31.5) & (tt <= 33.5)),
                 ("active_34-44", (tt >= 34) & (tt <= 44)),
                 ("hold_44-46", (tt >= 44) & (tt <= 46.4))]:
    e = err[sl] - np.mean(err[sl])
    f = np.fft.rfftfreq(len(e), 1/fs)
    E = np.abs(np.fft.rfft(e)) / len(e) * 2
    top = np.argsort(E[1:])[::-1][:6] + 1
    print(name, "std", np.std(e), "peaks", [(round(f[i], 2), round(E[i], 3)) for i in top])

# zoom on the reversal 35.8-36.6
zoom = (t >= 35.6) & (t <= 36.6)
fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
axs[0].plot(t[zoom], u[zoom], ".-", ms=3, label="cmd")
axs[0].plot(t[zoom], y[zoom], ".-", ms=3, label="meas")
axs[0].legend(); axs[0].grid(True); axs[0].set_ylabel("deg")
axs[1].plot(t[zoom], err[zoom], ".-", ms=3, color="crimson")
axs[1].grid(True); axs[1].set_ylabel("err deg")
axs[2].plot(t[zoom], rate_u[zoom], label="cmd rate")
axs[2].plot(t[zoom], rate_y[zoom], label="meas rate")
axs[2].plot(t[zoom], 6*9*err[zoom], label="9*err*6 (target rate)")
axs[2].legend(); axs[2].grid(True); axs[2].set_ylabel("deg/s")
plt.tight_layout(); plt.savefig(r"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\reversal.png", dpi=120)

# zoom on a hold to see ripple
zoom = (t >= 37.6) & (t <= 38.4)
fig, axs = plt.subplots(2, 1, figsize=(12, 5), sharex=True)
axs[0].plot(t[zoom], u[zoom], ".-", ms=3, label="cmd")
axs[0].plot(t[zoom], y[zoom], ".-", ms=3, label="meas")
axs[0].legend(); axs[0].grid(True); axs[0].set_ylabel("deg")
axs[1].plot(t[zoom], err[zoom], ".-", ms=3, color="crimson")
axs[1].grid(True); axs[1].set_ylabel("err deg")
plt.tight_layout(); plt.savefig(r"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\hold_ripple.png", dpi=120)
