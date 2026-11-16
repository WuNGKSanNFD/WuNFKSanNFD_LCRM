# -*- coding: utf-8 -*-
"""Validate the grey-box model against the measured run, then predict candidates."""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PATH = r"G:\RM\00Sentry\采集数据\26.10.23 yaw跟随数据1.csv"
df = pd.read_csv(PATH)
y_meas = df["I0"].to_numpy(float)
u_meas = df["I1"].to_numpy(float)
fs = 100.0
t_log = np.arange(len(y_meas))/fs

H = 0.002
t = np.arange(31.5, 44.6, H)
u = u_meas[np.minimum((t*fs).astype(int), len(u_meas)-1)]

def simulate(kp_p=9.0, kp_s=400.0, ki_s=2.0, maxout_s=20000.0, max_i=1600.0,
             tau_m=0.09, K=320/30000, tau_f=1/(2*np.pi*47), delay=1,
             y0=-1.8, w0=0.0, i0=0.0, ff=0.0, dead=0.0):
    n=len(t); y=np.zeros(n); w=np.zeros(n); v=np.zeros(n)
    y[0]=y0; w[0]=w0
    i=i0; wf=w0; last_u=u[0]
    vbuf=np.zeros(delay+1)
    for k in range(1,n):
        e=u[k]-y[k-1]
        p=kp_p*e + ff*(u[k]-last_u)
        last_u=u[k]
        wcmd=np.clip(p,-150,150)
        ev=wcmd-wf
        i+=ev
        iout=np.clip(ki_s*i,-max_i,max_i)
        vv=np.clip(kp_s*ev+iout,-maxout_s,maxout_s)
        v[k]=vv
        vbuf=np.roll(vbuf,1); vbuf[0]=vv
        vd=vbuf[delay]
        w[k]=w[k-1]+H/tau_m*(-w[k-1]+K*vd)
        wf=wf+H/tau_f*(w[k]-wf)
        y[k]=y[k-1]+6*w[k]*H
    return y,w,v

ym=np.interp(t,t_log,y_meas)
um=np.interp(t,t_log,u_meas)
cases=[("current",dict()),
       ("12+400",dict(kp_p=12)),
       ("12+800",dict(kp_p=12,kp_s=800,ki_s=3,maxout_s=30000)),
       ("15+800",dict(kp_p=15,kp_s=800,ki_s=3,maxout_s=30000)),
       ("12+800+ff40",dict(kp_p=12,kp_s=800,ki_s=3,maxout_s=30000,ff=40)),
       ("9+800+ff60",dict(kp_p=9,kp_s=800,ki_s=3,maxout_s=30000,ff=60))]
print(f"{'case':16s} {'rms(34-44)':>11s} {'max|e|':>8s} {'err_rms':>8s} {'v_sat%':>7s}")
for name,kw in cases:
    y,w,v=simulate(**kw)
    e=um-y
    sel=(t>34)&(t<44.4)
    print(f"{name:16s} {np.sqrt(np.mean(e[sel]**2)):11.2f} {np.max(np.abs(e[sel])):8.2f} {np.sqrt(np.mean((um[sel]-ym[sel])**2)):8.2f} {100*np.mean(np.abs(v[sel])>=0.999*kw.get('maxout_s',20000)):7.1f}")

# plot measured vs current and 12+800
fig,axs=plt.subplots(2,1,figsize=(14,8),sharex=True)
axs[0].plot(t,um,lw=1.0,label='cmd')
axs[0].plot(t,ym,lw=1.0,label='measured')
for name,kw in [("current",dict()),("12+800",dict(kp_p=12,kp_s=800,ki_s=3,maxout_s=30000))]:
    y,w,v=simulate(**kw); axs[0].plot(t,y,lw=1.0,label='sim '+name)
axs[0].legend(); axs[0].grid(True); axs[0].set_ylabel('deg')
for name,kw in [("current",dict()),("12+800",dict(kp_p=12,kp_s=800,ki_s=3,maxout_s=30000))]:
    y,w,v=simulate(**kw); axs[1].plot(t,um-y,lw=1.0,label='err '+name)
axs[1].plot(t,um-ym,lw=0.8,color='k',label='err measured')
axs[1].legend(); axs[1].grid(True); axs[1].set_ylabel('deg')
plt.tight_layout(); plt.savefig(r"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\validate.png",dpi=120)
