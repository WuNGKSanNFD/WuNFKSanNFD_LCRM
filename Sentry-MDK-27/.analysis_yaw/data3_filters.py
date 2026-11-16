# -*- coding: utf-8 -*-
"""Data3 replay: can output filters tame f_kp=40 kick without losing tracking?"""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PATH = r"G:\RM\00Sentry\采集数据\26.10.23 yaw跟随数据3.csv"
df = pd.read_csv(PATH)
y_meas = df["I0"].to_numpy(float); u_meas = df["I1"].to_numpy(float)
fs=100.0; t_log=np.arange(len(y_meas))/fs
H=0.002; t=np.arange(3.8, 19.5, H)
u=u_meas[np.minimum((t*fs).astype(int), len(u_meas)-1)]

def sim(f_kp=0.0, kp_p=9.0, kp_s=400.0, ki_s=2.0, max_i=1600.0, maxout_s=20000.0,
        tau_m=0.09, K=320/30000, tau_f=1/(2*np.pi*47), delay=1, y0=0.0,
        ap=0.0, as_=0.0):
    """ap: position output filter alpha; as_: speed output filter alpha."""
    n=len(t); y=np.zeros(n); w=np.zeros(n); v=np.zeros(n); wc=np.zeros(n)
    y[0]=y0; w[0]=0.0; i=0.0; wf=0.0; last_u=u[0]; buf=np.zeros(delay+1)
    last_wc=0.0; last_v=0.0
    for k in range(1,n):
        e=u[k]-y[k-1]
        p=kp_p*e + f_kp*(u[k]-last_u); last_u=u[k]
        wcmd=np.clip(p,-150,150)
        if ap>0: wcmd = ap*last_wc + (1-ap)*wcmd
        last_wc=wcmd; wc[k]=wcmd
        ev=wcmd-wf; i+=ev
        iout=np.clip(ki_s*i,-max_i,max_i)
        vv=np.clip(kp_s*ev+iout,-maxout_s,maxout_s)
        if as_>0: vv = as_*last_v + (1-as_)*vv
        last_v=vv; v[k]=vv
        buf=np.roll(buf,1); buf[0]=vv
        w[k]=w[k-1]+H/tau_m*(-w[k-1]+K*buf[delay])
        wf=wf+H/tau_f*(w[k]-wf)
        y[k]=y[k-1]+6*w[k]*H
    return y,w,v,wc

ym=np.interp(t,t_log,y_meas)
cases=[("f=0",dict(f_kp=0)),("f=40",dict(f_kp=40)),
       ("f=40 ap=.3",dict(f_kp=40,ap=0.3)),("f=40 ap=.5",dict(f_kp=40,ap=0.5)),
       ("f=40 as=.3",dict(f_kp=40,as_=0.3)),("f=40 as=.5",dict(f_kp=40,as_=0.5)),
       ("f=20 ap=.3",dict(f_kp=20,ap=0.3)),("f=0 ap=.3",dict(f_kp=0,ap=0.3))]
print(f"{'case':14s} {'err_rms':>8s} {'err_max':>8s} {'wc_max':>8s} {'v_sat%':>7s} {'v_step_max':>10s}")
for name,kw in cases:
    y,w,v,wc=sim(**kw)
    e=u-y
    print(f"{name:14s} {np.sqrt(np.mean(e**2)):8.2f} {np.max(np.abs(e)):8.2f} {np.max(np.abs(wc)):8.1f} {100*np.mean(np.abs(v)>=0.999*20000):7.1f} {np.max(np.abs(np.diff(v))):10.0f}")

fig,axs=plt.subplots(3,1,figsize=(13,9),sharex=True)
axs[0].plot(t,u,lw=1,label='cmd'); axs[0].legend(); axs[0].grid(True); axs[0].set_ylabel('deg')
for name,kw in [("f=0",dict(f_kp=0)),("f=40",dict(f_kp=40)),("f=40 ap=.5",dict(f_kp=40,ap=0.5))]:
    y,w,v,wc=sim(**kw)
    axs[1].plot(t,v,lw=0.8,label='V '+name)
    axs[2].plot(t,u-y,lw=0.8,label='err '+name)
axs[1].legend(); axs[1].grid(True); axs[1].set_ylabel('counts')
axs[2].legend(); axs[2].grid(True); axs[2].set_ylabel('deg'); axs[2].set_xlabel('s')
plt.tight_layout(); plt.savefig(r"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\d3_filter_compare.png",dpi=120)
