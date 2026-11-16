# -*- coding: utf-8 -*-
"""Data4: f_kp=20, speed output alpha=0.3. Locate remaining noise."""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PATH = r"G:\RM\00Sentry\采集数据\26.10.23 yaw跟随数据4.csv"
df = pd.read_csv(PATH)
y = df["I0"].to_numpy(float); u = df["I1"].to_numpy(float)
n=len(y); fs=100.0; t=np.arange(n)/fs
print("rows",n,"duration",n/fs)
print("u unique",np.unique(u)[:10],"count",len(np.unique(u)))
print("y range",y.min(),y.max())
du=np.diff(u); idx=np.where(np.abs(du)>1e-9)[0]
print("command changes",[(round(t[i],2),u[i],u[i+1]) for i in idx], "count",len(idx))

def ma(a,k): return np.convolve(a,np.ones(k)/k,mode='same')
def spec(a,fs):
    a=a-a.mean(); f=np.fft.rfftfreq(len(a),1/fs); A=np.abs(np.fft.rfft(a))*2/len(a)
    return f,A

# segment boundaries from command changes
bounds=[0]+[t[i] for i in idx]+[n/fs]
for j in range(len(bounds)-1):
    a,b=bounds[j],bounds[j+1]
    if b-a<0.2: continue
    s=(t>=a+0.05)&(t<b-0.05)
    if s.sum()<20: continue
    yy=y[s]; res=yy-ma(yy,51)  # 0.5s MA
    f,A=spec(res,fs); top=np.argsort(A[1:])[::-1][:6]+1
    print(f"seg {a:6.2f}-{b:6.2f} u={np.mean(u[s]):8.2f} err={np.mean(u[s]-yy):6.2f} res_std={res.std():6.3f} peaks={[(round(f[i],2),round(A[i],3)) for i in top]}")

res=y-ma(y,101)
fig,axs=plt.subplots(4,1,figsize=(14,12),sharex=True)
axs[0].plot(t,y,lw=0.8,label='I0'); axs[0].plot(t,u,lw=0.8,label='I1')
axs[0].legend(); axs[0].grid(True); axs[0].set_ylabel('deg')
axs[1].plot(t,res,lw=0.6); axs[1].grid(True); axs[1].set_ylabel('I0 - 1s MA')
dy=np.diff(y); axs[2].plot(t[1:],dy,lw=0.5); axs[2].grid(True); axs[2].set_ylabel('delta deg/10ms')
win=100; step=10; freqs=np.fft.rfftfreq(win,1/fs)
amp=[]
for i in range(0,n-win,step):
    seg=res[i:i+win]*np.hanning(win); amp.append(np.abs(np.fft.rfft(seg))/win*2)
amp=np.array(amp)
im=axs[3].pcolormesh(np.arange(len(amp))*step/fs,freqs,amp.T,shading='auto',cmap='inferno')
axs[3].set_ylim(0,50); axs[3].set_xlabel('s'); axs[3].set_ylabel('Hz'); fig.colorbar(im,ax=axs[3])
plt.tight_layout(); plt.savefig(r"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\d4_overview.png",dpi=120)

for a,b,name in [(1,2,'d4_z1'),(5,6,'d4_z5'),(10,11,'d4_z10'),(14,15,'d4_z14')]:
    ia,ib=int(a*fs),min(int(b*fs),n)
    fig,axs=plt.subplots(2,1,figsize=(12,6),sharex=True)
    axs[0].plot(t[ia:ib],y[ia:ib],'.-',ms=3,lw=0.8,label='I0'); axs[0].plot(t[ia:ib],u[ia:ib],'.-',ms=3,lw=0.8,label='I1')
    axs[0].legend(); axs[0].grid(True); axs[0].set_ylabel('deg')
    axs[1].plot(t[1:][ia:ib],dy[ia:ib],'.-',ms=3,lw=0.8); axs[1].grid(True); axs[1].set_ylabel('delta deg/10ms'); axs[1].set_xlabel('s')
    plt.tight_layout(); plt.savefig(rf"G:\RM\00Sentry\Sentry-MDK\.analysis_yaw\{name}.png",dpi=120)
