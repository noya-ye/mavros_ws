import csv
import json
import math
import re
import sqlite3
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

OUT = Path(__file__).resolve().parent
BAG = OUT.parents[1] / 'rosbag2_2026_10_05-16_19_11/rosbag2_2026_10_05-16_19_11_0.db3'
db = sqlite3.connect(f'file:{BAG}?mode=ro', uri=True)
t0 = db.execute('SELECT MIN(timestamp) FROM messages').fetchone()[0] / 1e9
topics = {i: (n, k) for i, n, k in db.execute('SELECT id,name,type FROM topics')}
rows, goals = [], []
for tid, ns, blob in db.execute('SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp'):
    name, kind = topics[tid]
    if name not in ['/fastlio2/lio_odom', '/simple_2d_planner/goal']:
        continue
    m = deserialize_message(blob, get_message(kind))
    t = ns / 1e9 - t0
    h = m.header.stamp.sec + m.header.stamp.nanosec / 1e9 - t0
    p = m.pose.pose if name.endswith('odom') else m.pose
    q = p.orientation
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = math.asin(np.clip(2*(w*y-z*x), -1, 1))
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    if name.endswith('goal'):
        goals.append(dict(t=t, xyz=[p.position.x,p.position.y,p.position.z], yaw_deg=math.degrees(yaw)))
        continue
    rows.append([t,h,p.position.x,p.position.y,p.position.z,roll,pitch,yaw,norm,
                 q.x,q.y,q.z,q.w,*m.pose.covariance])
db.close()
a = np.array(rows)
t = a[:,0]
yaw = np.rad2deg(np.unwrap(a[:,7]))
dt = np.diff(a[:,1])
rate = np.diff(yaw)/dt
speed = np.linalg.norm(np.diff(a[:,2:5],axis=0),axis=1)/dt
q = a[:,9:13]
rot_rate = np.rad2deg(2*np.arccos(np.clip(np.abs(np.sum(q[1:]*q[:-1],axis=1)),0,1)))/dt
with (OUT/'lio.csv').open('w') as f:
    writer = csv.writer(f)
    writer.writerow(['bag_t','header_t','x','y','z','roll_rad','pitch_rad','yaw_rad','qnorm','qx','qy','qz','qw',*[f'cov_{i}' for i in range(36)],'yaw_unwrapped_deg'])
    writer.writerows(np.column_stack([a,yaw]))

log = Path('/home/jetson/.ros/log/align_drop_snake_ego_node_19474_1791188360494.log')
samples, events = [], []
for line in log.read_text().splitlines():
    match = re.search(r'\[(179\d+\.\d+)\]',line)
    if not match:
        continue
    lt = float(match[1])-t0
    match_y = re.search(r'(?<!locked_)yaw=([-\d.]+)',line)
    if match_y:
        samples.append([lt,math.degrees(float(match_y[1]))])
    if any(s in line for s in ['saving resume','phase=DOWN','drop completed','return completed','EGO target','signal_handler']):
        events.append(dict(t=lt,text=line))

def clock(t):
    return datetime.fromtimestamp(t0+t,ZoneInfo('Asia/Shanghai')).strftime('%H:%M:%S.%f')[:-3]

summary = dict(start=clock(0),end=clock(t[-1]),goals=goals,events=events,
               quaternion_norm_max_error=float(np.max(abs(a[:,8]-1))),
               header_dt_min=float(dt.min()),header_dt_max=float(dt.max()),
               header_age_min=float((a[:,0]-a[:,1]).min()),header_age_max=float((a[:,0]-a[:,1]).max()),
               covariance_nonzero=int(np.count_nonzero(a[:,13:])),windows=[])
for lo, hi in [(0,120),(120,140),(140,150),(150,160),(160,176.7)]:
    mask = (t>=lo)&(t<hi)
    rm = (t[1:]>=lo)&(t[1:]<hi)
    summary['windows'].append(dict(start=lo,end=hi,clock=clock(lo),
        yaw_min=float(yaw[mask].min()),yaw_max=float(yaw[mask].max()),
        yaw_rate_max=float(abs(rate[rm]).max()),rotation_rate_max=float(rot_rate[rm].max()),
        roll_min=float(np.rad2deg(a[mask,5]).min()),roll_max=float(np.rad2deg(a[mask,5]).max()),
        pitch_min=float(np.rad2deg(a[mask,6]).min()),pitch_max=float(np.rad2deg(a[mask,6]).max()),
        speed_max=float(speed[rm].max())))
summary['largest_yaw_steps'] = [dict(t=float(t[i+1]),clock=clock(t[i+1]),delta=float(yaw[i+1]-yaw[i]),rate=float(rate[i]),xyz=a[i+1,2:5].tolist()) for i in np.argsort(abs(rate))[-12:][::-1]]
summary['snapshots'] = [dict(t=float(t[i]),clock=clock(t[i]),yaw=float(yaw[i]),roll=float(np.rad2deg(a[i,5])),pitch=float(np.rad2deg(a[i,6])),xyz=a[i,2:5].tolist()) for target in range(115,177) for i in [np.argmin(abs(t-target))]]
summary['yaw_landmarks'] = [dict(threshold=threshold,t=float(t[i]),clock=clock(t[i]),yaw=float(yaw[i])) for threshold in [20,45,90,180,270,350] for i in [np.flatnonzero((t>140)&(yaw>threshold))[0]]]
summary['mavros_comparison'] = [dict(t=st,clock=clock(st),mavros_yaw=sy,lio_yaw_wrapped=float((np.interp(st,a[:,1],yaw)+180)%360-180)) for st,sy in samples if st>140]
(OUT/'metrics.json').write_text(json.dumps(summary,indent=2))
print(json.dumps({k:v for k,v in summary.items() if k not in ['events','snapshots','largest_yaw_steps']},indent=2))
for limits, filename in [((0,177),'overview.png'),((118,170),'final_segment.png')]:
    fig, axes = plt.subplots(4,1,figsize=(13,11),sharex=True)
    axes[0].plot(t,yaw,label='FAST-LIO yaw (unwrapped)')
    if samples:
        s = np.array(samples)
        s[:,1] += 360*np.round((np.interp(s[:,0],a[:,1],yaw)-s[:,1])/360)
        axes[0].scatter(s[:,0],s[:,1],s=28,color='red',label='MAVROS yaw from task log',zorder=5)
    axes[0].set_ylabel('Yaw (deg)')
    axes[0].legend()
    for stamp,label,color in [(1791188474.717,'Low battery','#b08000'),(1791188502.993,'Critical battery','#cc3333')]:
        bt = stamp-t0
        if limits[0]<bt<limits[1]:
            axes[0].axvline(bt,color=color,ls='--')
            axes[0].text(bt+.2,100,label,color=color,rotation=90,va='bottom',fontsize=9)
    axes[1].plot(t,np.rad2deg(a[:,5]),label='roll')
    axes[1].plot(t,np.rad2deg(a[:,6]),label='pitch')
    axes[1].set_ylabel('Tilt (deg)')
    axes[1].legend()
    for i,label in [(2,'x'),(3,'y'),(4,'z')]:
        axes[2].plot(t,a[:,i],label=label)
    axes[2].set_ylabel('Position (m)')
    axes[2].legend()
    axes[3].plot(t[1:],rate,label='yaw rate from LIO')
    axes[3].set_ylabel('Yaw rate (deg/s)')
    axes[3].set_xlabel('Seconds since '+clock(0)+' (Asia/Shanghai)')
    for ax in axes:
        ax.grid(alpha=.3)
        ax.set_xlim(*limits)
        for e in events:
            ax.axvline(e['t'],color='gray',alpha=.2)
    fig.tight_layout()
    fig.savefig(OUT/filename,dpi=150)
    plt.close(fig)
