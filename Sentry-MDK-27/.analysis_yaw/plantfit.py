# -*- coding: utf-8 -*-
"""Reconstruct the speed-loop voltage from measured trajectory and fit the plant."""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PATH = r"G:\RM\00Sentry\采集数据\26.10.23 yaw跟随数据1.csv"
df = pd.read_csv(PATH)
y_log = df["I0"].to_numpy(float)
u_log = df["I1"].to_numpy(float)
fs_log = 100.0
t_log = np.arange(len(y_log)) / fs_log

H = 0.002
t = np.arange(33.0, 44.5, H)
u = u_log[np.minimum((t * fs_log).astype(int), len(u_log) - 1)]
y = np.interp(t, t_log, y_log)

# smooth yaw and differentiate (deg/s)
def savgol(a, window=11, poly=2):
    # simple local polynomial (window odd), returns smoothed
    half = window // 2
    out = np.empty_like(a)
    x = np.arange(-half, half + 1)
    for i in range(len(a)):
        lo, hi = max(0, i-half), min(len(a), i+half+1)
        xx = x[lo-(i-half):hi-(i-half)]
        coef = np.polyfit(xx, a[lo:hi], poly)
        out[i] = np.polyval(coef, 0)
    return out

ys = savgol(y, 11, 2)
w_deg = np.gradient(ys, H)          # deg/s
w = w_deg / 6.0                     # rpm (same as IMU gyro_z)
dw = np.gradient(w, H)

# reconstruct controller from measured trajectory
e = u - y
target = 9.0 * e
ev = target - w
# integrate i (2 ms), clamp i_out
i = np.cumsum(ev) * H  # since code i_term += error each 2ms, i_term = sum(error)
iout = np.clip(2.0 * i, -1600, 1600)
v = np.clip(400.0 * ev + iout, -20000, 20000)
print("v saturated fraction:", np.mean(np.abs(v) >= 20000*0.999), "min/max", v.min(), v.max())
print("ev rms", np.sqrt(np.mean(ev**2)), "target rms", np.sqrt(np.mean(target**2)), "w rms", np.sqrt(np.mean(w**2)))

# low-pass everything at ~15 Hz to reduce derivative noise
def lp(a, fc=15.0):
    tau = 1/(2*np.pi*fc)
    a2 = np.empty_like(a); a2[0] = a[0]
    alpha = H/(tau+H)
    for k in range(1, len(a)):
        a2[k] = a2[k-1] + alpha*(a[k]-a2[k-1])
    return a2

wl, vl, dwl = lp(w, 15), lp(v, 15), lp(dw, 15)
# only use non-saturated, moderate dynamics
sel = (np.abs(v) < 19000) & (np.abs(w) < 200)
A = np.vstack([vl[sel], wl[sel]]).T
b = dwl[sel]
coef, *_ = np.linalg.lstsq(A, b, rcond=None)
a, bb = coef
tau_m = -1/bb
K = a*tau_m
print(f"plant fit: tau_m={tau_m:.4f} s, K={K:.6f} rpm/count, (K*30000={K*30000:.1f} rpm @30000)")
pred = A @ coef
print("domega fit rms", np.std(b-pred), "domega std", np.std(b))
# R2
print("R2", 1-np.sum((b-pred)**2)/np.sum((b-np.mean(b))**2))

# Fit with fixed K=320/30000, only tau
Kfix = 320/30000
A2 = (Kfix*vl[sel] - wl[sel])[:, None]
tau2, *_ = np.linalg.lstsq(A2, dwl[sel], rcond=None)
print("fixed K, tau_m =", 1/tau2[0])

fig, axs = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
axs[0].plot(t, u, lw=0.8); axs[0].plot(t, y, lw=0.8); axs[0].grid(True); axs[0].set_ylabel("deg")
axs[1].plot(t, w, lw=0.6); axs[1].plot(t, wl, lw=0.8); axs[1].grid(True); axs[1].set_ylabel("rpm")
axs[2].plot(t, v, lw=0.6); axs[2].axhline(20000, ls="--", c="k", lw=0.6); axs[2].axhline(-20000, ls="--", c="k", lw=0.6)
axs[2].grid(True); axs[2].set_ylabel("counts")
plt.tight_layout(); plt.savefig(r"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\reconstruct.png", dpi=120)
