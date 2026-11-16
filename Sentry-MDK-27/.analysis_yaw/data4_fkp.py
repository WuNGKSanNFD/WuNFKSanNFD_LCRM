# -*- coding: utf-8 -*-
"""Data4 replay: compare f_kp / output filter options."""
import numpy as np
import pandas as pd

PATH = r"G:\RM\00Sentry\采集数据\26.10.23 yaw跟随数据4.csv"
df = pd.read_csv(PATH)
y_meas = df["I0"].to_numpy(float); u_meas = df["I1"].to_numpy(float)
fs=100.0; t_log=np.arange(len(y_meas))/fs
H=0.002; t=np.arange(0.0,16.3,H)
u=u_meas[np.minimum((t*fs).astype(int),len(u_meas)-1)]
ym=np.interp(t,t_log,y_meas)

def sim4(f_kp=0.0, ap=0.0, as_=0.0, kp_p=9.0, kp_s=400.0, ki_s=2.0,
         max_i=1600.0, maxout_s=20000.0, tau_m=0.09, K=320/30000,
         tau_f=1/(2*np.pi*47), delay=1, y0=0.0, ylim=None):
    n=len(t); y=np.zeros(n); w=np.zeros(n); v=np.zeros(n); wc=np.zeros(n); y[0]=y0
    i=0.0; wf=0.0; last_u=u[0]; buf=np.zeros(delay+1); last_wc=0.0; last_v=0.0
    for k in range(1,n):
        e=u[k]-y[k-1]
        p=kp_p*e + f_kp*(u[k]-last_u); last_u=u[k]
        wcmd=np.clip(p,-150,150)
        if ap>0: wcmd=ap*last_wc+(1-ap)*wcmd
        last_wc=wcmd; wc[k]=wcmd
        ev=wcmd-wf; i+=ev
        iout=np.clip(ki_s*i,-max_i,max_i)
        vv=np.clip(kp_s*ev+iout,-maxout_s,maxout_s)
        if as_>0: vv=as_*last_v+(1-as_)*vv
        last_v=vv; v[k]=vv
        buf=np.roll(buf,1); buf[0]=vv
        w[k]=w[k-1]+H/tau_m*(-w[k-1]+K*buf[delay])
        wf=wf+H/tau_f*(w[k]-wf)
        y[k]=y[k-1]+6*w[k]*H
        if ylim is not None:
            if y[k]>ylim[1]: y[k]=ylim[1]; w[k]=0; wf=0
            elif y[k]<ylim[0]: y[k]=ylim[0]; w[k]=0; wf=0
    return y,w,v,wc

print(f"{'case':22s} {'err_rms':>8s} {'err_max':>8s} {'vsat%':>6s} {'vstep':>7s}")
for name,kw in [
    ("f20 a0.3 (current)", dict(f_kp=20,as_=0.3)),
    ("f10 a0.5", dict(f_kp=10,as_=0.5)),
    ("f0 a0.5", dict(f_kp=0,as_=0.5)),
    ("f20 a0.5", dict(f_kp=20,as_=0.5)),
    ("f10 a0.3", dict(f_kp=10,as_=0.3)),
]:
    y,w,v,wc=sim4(**kw); e=u-y
    print(f"{name:22s} {np.sqrt(np.mean(e**2)):8.2f} {np.max(np.abs(e)):8.2f} {100*np.mean(np.abs(v)>=0.999*20000):6.1f} {np.max(np.abs(np.diff(v))):7.0f}")

# with travel limit 116.6 to model the stall
print("\nwith y clamp +116.6:")
for name,kw in [("f20 a0.3",dict(f_kp=20,as_=0.3)),("f10 a0.5",dict(f_kp=10,as_=0.5)),("f0 a0.5",dict(f_kp=0,as_=0.5))]:
    y,w,v,wc=sim4(ylim=(-85.8,116.6),**kw); e=u-y
    print(f"{name:22s} {np.sqrt(np.mean(e**2)):8.2f} {np.max(np.abs(e)):8.2f} {100*np.mean(np.abs(v)>=0.999*20000):6.1f} {np.max(np.abs(np.diff(v))):7.0f}")
