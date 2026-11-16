# -*- coding: utf-8 -*-
"""Check speed-loop error and integral saturation signature."""
import numpy as np
import pandas as pd

PATH = r"G:\RM\00Sentry\采集数据\26.10.23 yaw跟随数据1.csv"
df = pd.read_csv(PATH)
y = df["I0"].to_numpy(float)
u = df["I1"].to_numpy(float)
fs = 100.0
t = np.arange(len(y)) / fs

def roll(a, n):
    return np.convolve(a, np.ones(n)/n, mode="same")

# smooth over 200 ms to remove 5/10 ms sampling ripple
ys = roll(y, 21)
us = roll(u, 21)
err = us - ys
rate = np.gradient(ys, 1/fs)      # deg/s
w = rate / 6.0                    # rpm
target = 9.0 * err                # rpm
ev = target - w

sel = (t > 33.5) & (t < 44.4)
sel &= np.abs(rate) > 100
print("ramp: mean target rpm %.2f, mean w rpm %.2f, mean ev rpm %.2f, rms ev %.2f" %
      (target[sel].mean(), w[sel].mean(), ev[sel].mean(), ev[sel].std()))
for lo, hi in [(100,200),(200,300),(300,400),(400,500),(500,700)]:
    s = sel & (np.abs(rate) >= lo) & (np.abs(rate) < hi)
    if s.sum():
        print(f"rate {lo}-{hi}: n={s.sum()} err={err[s].mean():.2f} target={target[s].mean():.2f} w={w[s].mean():.2f} ev={ev[s].mean():.2f}")
# hold segments
for a,b in [(33.8,34.1),(36.6,37.2),(38.2,38.7),(43.6,44.2),(44.5,46.3)]:
    s=(t>a)&(t<b)
    print(f"hold {a}-{b}: err={err[s].mean():.3f} std={err[s].std():.3f} w={w[s].mean():.2f} ev={ev[s].mean():.2f}")

# integral needed: i_out = w/K - 400*ev, K=320/30000
K=320/30000
i_needed = w/K - 400*ev
print("needed i_out during ramps: mean %.0f median %.0f max %.0f (limit 1600)" %
      (i_needed[sel].mean(), np.median(i_needed[sel]), i_needed[sel].max()))
