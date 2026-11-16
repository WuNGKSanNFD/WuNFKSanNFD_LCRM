# -*- coding: utf-8 -*-
"""Grey-box simulation of the small-yaw cascade, fit to the 26.10.23 data.

Controller (from code):
  gimbal 2 ms:  w_cmd = clamp(kp_p*(u-y), +-maxout_p)          [rpm]
  motor  2 ms:  v     = clamp(kp_s*(w_cmd-w) + clamp(ki*i,+-max_i), +-maxout_s) [counts]
                i    += (w_cmd - w)
  plant:        tau_m*dw/dt = -w + K*v,  K = 320 rpm / 30000 counts
                dy/dt = 6*w  [deg/s]
"""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PATH = r"G:\RM\00Sentry\采集数据\26.10.23 yaw跟随数据1.csv"
df = pd.read_csv(PATH)
y_meas = df["I0"].to_numpy(float)
u_meas = df["I1"].to_numpy(float)
fs_log = 100.0
t_log = np.arange(len(y_meas)) / fs_log

H = 0.002
t = np.arange(31.5, 44.6, H)
# zero-order hold command at 2 ms from the 10 ms log (assume log is a sample of 5 ms cmd)
u = u_meas[np.minimum((t * fs_log).astype(int), len(u_meas) - 1)]

def simulate(kp_p=9.0, ki_p=0.0, kd_p=0.0, maxout_p=150.0, f_kp=0.0,
             kp_s=400.0, ki_s=2.0, max_i=1600.0, maxout_s=20000.0,
             tau_m=0.079, K=320/30000, tau_f=1/(2*np.pi*47), delay_steps=1,
             y0=0.0, w0=0.0, i0=0.0, ip0=0.0):
    n = len(t)
    y = np.zeros(n); w = np.zeros(n); y[0] = y0; w[0] = w0
    i = i0          # speed integral (error sum)
    ip = ip0        # position integral
    last_e = 0.0
    wf = w0         # filtered speed feedback
    u_last = u[0]
    vhist = np.zeros(n)
    for k in range(1, n):
        # measurement (y is logged/output)
        # position loop
        e = u[k] - y[k-1]
        ip += e
        p = kp_p * e + ki_p * ip + kd_p * (e - last_e)
        last_e = e
        p += f_kp * (u[k] - u_last)
        u_last = u[k]
        w_cmd = np.clip(p, -maxout_p, maxout_p)
        # speed loop
        ev = w_cmd - wf
        i += ev
        iout = np.clip(ki_s * i, -max_i, max_i)
        v = np.clip(kp_s * ev + iout, -maxout_s, maxout_s)
        vhist[k] = v
        # plant (first-order speed), delayed command
        vv = vhist[k - delay_steps] if k >= delay_steps else 0.0
        w[k] = w[k-1] + H / tau_m * (-w[k-1] + K * vv)
        # gyro filter
        wf = wf + H / tau_f * (w[k] - wf)
        # integrate yaw
        y[k] = y[k-1] + 6.0 * w[k] * H
    return y, w, vhist

if __name__ == "__main__":
    # fit tau_m and initial conditions to the data with current gains
    y0 = np.interp(t[0], t_log, y_meas)
    w0 = 0.0
    best = None
    for tau_m in np.arange(0.02, 0.16, 0.002):
        y, w, v = simulate(tau_m=tau_m, y0=y0, w0=w0)
        ym = np.interp(t, t_log, y_meas)
        # compare after 0.5 s
        sel = t > t[0] + 0.5
        rms = np.sqrt(np.mean((y[sel] - ym[sel])**2))
        if best is None or rms < best[0]:
            best = (rms, tau_m)
    print("best tau_m, rms (current gains):", best)
    tau_m = best[1]
    y, w, v = simulate(tau_m=tau_m, y0=y0, w0=w0)
    ym = np.interp(t, t_log, y_meas)
    um = np.interp(t, t_log, u_meas)
    err_sim = um - y
    err_meas = um - ym
    sel = (t > 34) & (t < 44.4)
    print("sim err rms", np.sqrt(np.mean(err_sim[sel]**2)), "meas err rms", np.sqrt(np.mean(err_meas[sel]**2)))
    print("sim max |err|", np.max(np.abs(err_sim[sel])), "meas max", np.max(np.abs(err_meas[sel])))
    print("v saturation fraction", np.mean(np.abs(v) >= 20000*0.999))

    fig, axs = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    axs[0].plot(t, um, lw=0.8, label="cmd")
    axs[0].plot(t, ym, lw=0.8, label="meas")
    axs[0].plot(t, y, lw=0.8, label="sim")
    axs[0].legend(); axs[0].grid(True)
    axs[1].plot(t, err_meas, lw=0.8, label="meas err")
    axs[1].plot(t, err_sim, lw=0.8, label="sim err")
    axs[1].legend(); axs[1].grid(True)
    axs[2].plot(t, v, lw=0.8, label="sim voltage cmd")
    axs[2].axhline(20000, color="k", ls="--", lw=0.6); axs[2].axhline(-20000, color="k", ls="--", lw=0.6)
    axs[2].legend(); axs[2].grid(True)
    plt.tight_layout(); plt.savefig(r"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\sim_fit.png", dpi=120)
