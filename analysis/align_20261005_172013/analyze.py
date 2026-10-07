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
ROOT = OUT.parents[1]
BAG = ROOT / 'rosbag2_2026_10_05-17_20_13/rosbag2_2026_10_05-17_20_13_0.db3'
LOG = Path('/home/jetson/.ros/log/align_drop_snake_ego_node_11231_1791192026052.log')
db = sqlite3.connect(f'file:{BAG}?mode=ro', uri=True)
t0 = db.execute('SELECT MIN(timestamp) FROM messages').fetchone()[0] / 1e9
topics = {i: (n, get_message(k)) for i, n, k in db.execute('SELECT id,name,type FROM topics')}
odom, points, goals = [], {}, []
for tid, ns, blob in db.execute('SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp'):
    name, cls = topics[tid]
    if name not in ['/fastlio2/lio_odom', '/simple_2d_planner/goal'] and not name.startswith('/target/'):
        continue
    m = deserialize_message(blob, cls)
    t = ns / 1e9 - t0
    if name.startswith('/target/'):
        points.setdefault(name, []).append([t, m.x, m.y])
    elif name.endswith('goal'):
        goals.append([t, m.pose.position.x, m.pose.position.y, m.pose.position.z])
    else:
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        odom.append([t, p.x, p.y, p.z, yaw])
db.close()
a = np.array(odom)
points = {k: np.array(v) for k, v in points.items()}
def clock(t):
    return datetime.fromtimestamp(t0+t, ZoneInfo('Asia/Shanghai')).strftime('%H:%M:%S.%f')[:-3]

events = []
for line in LOG.read_text().splitlines():
    match = re.search(r'\[(179\d+\.\d+)\]', line)
    if match:
        t = float(match[1])-t0
        events.append(dict(t=t, clock=clock(t), text=line))
attempts = []
for event in events:
    if 'down alignment triggered' in event['text']:
        attempts.append(dict(start=event['t'], start_clock=event['clock']))
    elif attempts and 'end' not in attempts[-1] and any(s in event['text'] for s in ['alignment succeeded;', 'alignment timed out']):
        attempts[-1].update(end=event['t'], end_clock=event['clock'], result=event['text'])
    elif attempts and 'return_end' not in attempts[-1] and 'return completed;' in event['text']:
        attempts[-1].update(return_end=event['t'], return_clock=event['clock'])

for attempt in attempts:
    lo, hi = attempt['start'], attempt.get('end', a[-1,0])
    attempt['duration_s'] = hi-lo
    attempt['detection_counts'] = {}
    for name, p in points.items():
        mask = (p[:,0]>=lo)&(p[:,0]<=hi)
        attempt['detection_counts'][name] = int(mask.sum())
        if mask.any():
            attempt.setdefault('last_detection_clock', {})[name] = clock(p[mask][-1,0])
    mask = (a[:,0]>=lo+1)&(a[:,0]<=hi)
    if mask.any():
        attempt['lio_xy_range_after_1s_m'] = np.ptp(a[mask,1:3], axis=0).tolist()

with (OUT/'odom.csv').open('w') as f:
    w = csv.writer(f)
    w.writerow(['bag_seconds', 'x', 'y', 'z', 'yaw_rad'])
    w.writerows(a)
with (OUT/'detections.csv').open('w') as f:
    w = csv.writer(f)
    w.writerow(['topic', 'bag_seconds', 'clock', 'forward_px', 'left_px'])
    for name, p in points.items():
        w.writerows([name, t, clock(t), x, y] for t, x, y in p)
summary = dict(start=clock(0), end=clock(a[-1,0]), attempts=attempts, goals=goals,
               counts={k: len(v) for k,v in points.items()}, events=events,
               limitations=['No camera images, YOLO, MAVROS pose, setpoints or task-state topic in bag.',
                            'LIO coordinates differ from MAVROS ENU; event ordering and motion are comparable.',
                            'Point messages have no acquisition timestamp; camera delay cannot be measured.'])
(OUT/'metrics.json').write_text(json.dumps(summary, indent=2))
for filename, windows in [('stalls.png', [(42,86),(187,221)])]:
    fig, axes = plt.subplots(3,2,figsize=(15,9),sharex='col',constrained_layout=True)
    for col, (lo,hi) in enumerate(windows):
        for j,label in [(1,'x'),(2,'y'),(3,'z')]:
            axes[0,col].plot(a[:,0], a[:,j], label=label)
        axes[0,col].set_ylabel('LIO position (m)')
        axes[0,col].legend()
        for name, p in points.items():
            axes[1,col].scatter(p[:,0],p[:,2],s=9,label=name.split('/')[-1])
        axes[1,col].set_ylabel('Target left offset (px)')
        axes[1,col].legend(fontsize=8)
        speed = np.linalg.norm(np.diff(a[:,1:3],axis=0),axis=1)/np.diff(a[:,0])
        axes[2,col].plot(a[1:,0],speed,color='black')
        axes[2,col].set_ylabel('LIO horizontal speed (m/s)')
        axes[2,col].set_xlabel('Seconds since '+clock(0)+' (Asia/Shanghai)')
        for attempt in attempts:
            start, end = attempt['start'], attempt.get('end',hi)
            if lo<start<hi:
                for ax in axes[:,col]:
                    ax.axvspan(start,end,color='#e69f00',alpha=.18)
                axes[0,col].text(start,axes[0,col].get_ylim()[1],clock(start),va='top',fontsize=8)
        for ax in axes[:,col]:
            ax.set_xlim(lo,hi)
            ax.grid(alpha=.3)
    fig.savefig(OUT/filename,dpi=160)
    plt.close(fig)
print(json.dumps({k:v for k,v in summary.items() if k!='events'},indent=2))
