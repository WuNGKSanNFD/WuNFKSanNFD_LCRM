# -*- coding: utf-8 -*-
"""Estimate velocity-loop closed-loop dynamics from target speed vs measured speed."""
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
m = (t >= 33.5) & (t <= 44.5)
tt = t[m]
uu = u[m]
yy = y[m]
err = uu - yy

# measured angular rate from IMU yaw (deg/s) and target speed from position loop (rpm)
# Use central difference; smooth with Savitzky-Golay-like moving average
def smooth(a, w):
    k = np.ones(w) / w
    return np.convolve(a, k, mode="same")
w = 5  # 50 ms
ys = smooth(yy, w)
omega_deg = np.gradient(ys, 1/fs)
omega_rpm = omega_deg / 6.0
target_rpm = 9.0 * err

# Fit tau: target = omega + tau*domega/dt  ->  omega = target - tau*domega/dt
# Use band-limited data (during moderate speeds, avoid saturation)
domega = np.gradient(omega_rpm, 1/fs)
sel = np.abs(target_rpm) < 60
A = np.vstack([domega[sel]]).T
b = (target_rpm - omega_rpm)[sel]
tau, *_ = np.linalg.lstsq(A, b, rcond=None)
print("fit tau (s):", tau[0])
res = b - A @ tau
print("rms residual rpm:", np.std(res), "target rms:", np.std(target_rpm[sel]))

# grid search tau minimizing simulation residual of first-order response
best = None
for tau_g in np.arange(0.001, 0.080, 0.001):
    # simulate discrete first-order: omega[k] = a*omega[k-1] + (1-a)*target[k-d]
    for d in [0, 1]:
        a = np.exp(-0.01 / tau_g)
        om = np.zeros_like(target_rpm)
        for k in range(1, len(om)):
            om[k] = a * om[k-1] + (1-a) * target_rpm[max(0, k-d)]
        # compare low-frequency component (moving average 100ms) to avoid derivative noise
        e = smooth(om, 10) - smooth(omega_rpm, 10)
        rms = np.sqrt(np.mean(e[sel]**2))
        if best is None or rms < best[0]:
            best = (rms, tau_g, d)
print("best first-order fit rms,tau,delay:", best)

# Plot target vs measured speed and speed error for a ramp section
fig, axs = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
axs[0].plot(tt, target_rpm, label="target speed = 9*(u-y)")
axs[0].plot(tt, omega_rpm, label="measured speed (dy/dt/6)")
axs[0].legend(); axs[0].grid(True); axs[0].set_ylabel("rpm")
axs[1].plot(tt, target_rpm - omega_rpm, label="speed error")
axs[1].grid(True); axs[1].set_ylabel("rpm"); axs[1].set_xlabel("s")
plt.tight_layout(); plt.savefig(r"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\vel_loop.png", dpi=120)

# Zoom 34.1-34.6
m2 = (t >= 34.05) & (t <= 34.7)
fig, axs = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
axs[0].plot(t[m2], u[m2], ".-", ms=4, label="cmd")
axs[0].plot(t[m2], y[m2], ".-", ms=4, label="meas")
axs[0].legend(); axs[0].grid(True)
axs[1].plot(t[m2], 9*(u[m2]-y[m2]), ".-", ms=4, label="target speed rpm")
axs[1].plot(t[m2], np.gradient(smooth(y[m2],5), 1/fs)/6, ".-", ms=4, label="meas speed rpm")
axs[1].legend(); axs[1].grid(True)
plt.tight_layout(); plt.savefig(r"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\vel_zoom.png", dpi=120)
