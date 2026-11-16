# -*- coding: utf-8 -*-
"""Linear cascade analysis for candidate small-yaw gains.

Model identified from the run:
  outer loop Kv = 6*kp_p  (measured 55.3 with kp_p=9)
  velocity loop: first-order closed loop, tau_v ~ 0.015-0.019 s with kp_s=400
  => plant tau_m = tau_v*(1+kp_s*K), K=320/30000 rpm/count ~ 0.079-0.10 s
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

K = 320.0 / 30000.0     # rpm per count
H = 0.002

def sim(kp_p=9.0, kp_s=400.0, ki_s=2.0, maxout_s=20000.0, max_i=1600.0,
        tau_m=0.09, tau_f=1/(2*np.pi*47), delay=1, T=3.0,
        cmd_rate=500.0, ff=0.0):
    """Simulate response to a 500 deg/s ramp starting at t=1s (deg)."""
    n = int(T/H)
    t = np.arange(n)*H
    y = np.zeros(n); w = np.zeros(n); v = np.zeros(n)
    i = 0.0; wf = 0.0; last_u = 0.0
    # command: 0 until 1s, then ramp at cmd_rate deg/s for 1s, then hold
    u = np.where(t < 1.0, 0.0, np.where(t < 2.0, cmd_rate*(t-1.0), cmd_rate))
    vq = np.zeros(n)
    delaybuf = np.zeros(delay+1)
    for k in range(1, n):
        e = u[k] - y[k-1]
        p = kp_p*e + ff*(u[k]-last_u)
        last_u = u[k]
        wcmd = np.clip(p, -150, 150)
        ev = wcmd - wf
        i += ev
        iout = np.clip(ki_s*i, -max_i, max_i)
        vv = np.clip(kp_s*ev + iout, -maxout_s, maxout_s)
        vq[k] = vv
        # delay
        delaybuf = np.roll(delaybuf, 1); delaybuf[0] = vv
        vd = delaybuf[delay]
        w[k] = w[k-1] + H/tau_m*(-w[k-1] + K*vd)
        wf = wf + H/tau_f*(w[k]-wf)
        y[k] = y[k-1] + 6*w[k]*H
    err = u - y
    # metrics on the ramp plateau (1.5-2.0s) and after
    m = (t > 1.9) & (t < 2.0)
    e_ramp = err[m].mean()
    m2 = t > 2.0
    overshoot = (y[m2].max()-y[-1]) if len(y[m2]) else 0
    return t, u, y, err, vq, e_ramp, overshoot

if __name__ == "__main__":
    cases = [
        ("current",        dict(kp_p=9,  kp_s=400, ki_s=2, maxout_s=20000)),
        ("kp_p=12",        dict(kp_p=12, kp_s=400, ki_s=2, maxout_s=20000)),
        ("kp_p=15",        dict(kp_p=15, kp_s=400, ki_s=2, maxout_s=20000)),
        ("kp_s=800",       dict(kp_p=9,  kp_s=800, ki_s=3, maxout_s=30000)),
        ("12+800",         dict(kp_p=12, kp_s=800, ki_s=3, maxout_s=30000)),
        ("15+800",         dict(kp_p=15, kp_s=800, ki_s=3, maxout_s=30000)),
        ("12+800+ff60",    dict(kp_p=12, kp_s=800, ki_s=3, maxout_s=30000, ff=60)),
        ("9+800+ff60",     dict(kp_p=9,  kp_s=800, ki_s=3, maxout_s=30000, ff=60)),
    ]
    fig, axs = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    print(f"{'case':18s} {'e_ramp@500':>10s} {'overshoot':>10s} {'vmax':>8s}")
    for name, kw in cases:
        t,u,y,err,v,e_ramp,ov = sim(**kw)
        print(f"{name:18s} {e_ramp:10.2f} {ov:10.2f} {np.max(np.abs(v)):8.0f}")
        axs[0].plot(t, err, lw=1.0, label=name)
        axs[1].plot(t, y, lw=1.0, label=name)
    axs[0].set_ylabel("tracking error (deg)"); axs[0].grid(True); axs[0].legend(fontsize=7)
    axs[1].set_ylabel("yaw (deg)"); axs[1].grid(True); axs[1].set_xlabel("s")
    plt.tight_layout(); plt.savefig(r"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\candidates.png", dpi=120)
