import csv
import argparse
import json
import sqlite3
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

parser = argparse.ArgumentParser()
parser.add_argument('--bag', type=Path)
parser.add_argument('--output', type=Path)
parser.add_argument('--kp', type=float, default=1.0)
parser.add_argument('--turn-windows', type=float, nargs=4, default=[36, 44, 46, 56])
args = parser.parse_args()
ROOT = args.output or Path(__file__).resolve().parent
ROOT.mkdir(parents=True, exist_ok=True)
BAG = args.bag or ROOT.parents[1] / 'ego_turn_test' / 'ego_turn_test_0.db3'
db = sqlite3.connect('file:' + str(BAG) + '?mode=ro', uri=True)
topics = {i: (name, get_message(kind)) for i, name, kind in db.execute('SELECT id,name,type FROM topics')}
t0 = db.execute('SELECT MIN(timestamp) FROM messages').fetchone()[0]
data = {name: [] for name, _ in topics.values()}
states, estimators = [], []
for tid, stamp, blob in db.execute('SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp'):
    name, cls = topics[tid]
    m = deserialize_message(blob, cls)
    t = (stamp - t0) / 1e9
    h = m.header.stamp.sec + m.header.stamp.nanosec / 1e9 - t0 / 1e9
    row = [t, h]
    if name == '/position_cmd':
        row += [m.position.x, m.position.y, m.position.z, m.velocity.x, m.velocity.y,
                m.velocity.z, m.acceleration.x, m.acceleration.y, m.acceleration.z,
                m.trajectory_id, m.yaw]
    elif name == '/mavros/setpoint_raw/local':
        row += [m.position.x, m.position.y, m.position.z, m.velocity.x, m.velocity.y,
                m.velocity.z, m.acceleration_or_force.x, m.acceleration_or_force.y,
                m.acceleration_or_force.z, m.type_mask, m.yaw]
    elif name.endswith('/pose') or name == '/mavros/setpoint_position/local':
        p = m.pose.position
        row += [p.x, p.y, p.z]
    elif name == '/fastlio2/lio_odom':
        p, v = m.pose.pose.position, m.twist.twist.linear
        row += [p.x, p.y, p.z, v.x, v.y, v.z]
    elif name.endswith('/velocity_local'):
        v = m.twist.linear
        row += [v.x, v.y, v.z]
    elif name == '/mavros/state':
        states.append(dict(t=t, armed=m.armed, mode=m.mode, connected=m.connected))
    elif name == '/mavros/estimator_status':
        estimators.append(dict(t=t, **{k: getattr(m, k) for k in m.get_fields_and_field_types() if k != 'header'}))
    data[name].append(row)
db.close()
data = {k: np.array(v) for k, v in data.items()}
cmd = data['/position_cmd']
raw = data['/mavros/setpoint_raw/local']
lio = data['/fastlio2/lio_odom']
pose = data['/mavros/local_position/pose']
vel = data['/mavros/local_position/velocity_local']
possp = data['/mavros/setpoint_position/local']

def interp(a, t, cols):
    return np.column_stack([np.interp(t, a[:, 0], a[:, c]) for c in cols])

def stats(a):
    return dict(median=float(np.median(a)), p95=float(np.percentile(a, 95)), max=float(np.max(a)))

summary = {'topics': {}, 'state_changes': [], 'estimator': estimators}
for name, a in data.items():
    dt = np.diff(a[:, 0])
    summary['topics'][name] = dict(count=len(a), start=float(a[0, 0]), end=float(a[-1, 0]),
                                  median_hz=float(1 / np.median(dt)), max_gap=float(np.max(dt)),
                                  header_age=stats(a[:, 0] - a[:, 1]))
    with (ROOT / (name.strip('/').replace('/', '_') + '.csv')).open('w') as f:
        csv.writer(f).writerows(a)
for s in states:
    if not summary['state_changes'] or any(s[k] != summary['state_changes'][-1][k] for k in ['armed', 'mode', 'connected']):
        summary['state_changes'].append(s)

tr = raw[:, 0]
pc = interp(cmd, tr, range(2, 11))
pl = interp(lio, tr, [2, 3, 4])
pp = interp(pose, tr, [2, 3, 4])
vv = interp(vel, tr, [2, 3, 4])
err = np.linalg.norm(pc[:, :2] - pl[:, :2], axis=1)
summary['tracking'] = dict(position_error=stats(err), fraction_error_above_030=float(np.mean(err > .30)),
                           raw_speed=stats(np.linalg.norm(raw[:, 5:7], axis=1)),
                           raw_acc=stats(np.linalg.norm(raw[:, 8:10], axis=1)),
                           actual_speed=stats(np.linalg.norm(vv[:, :2], axis=1)),
                           raw_masks=np.unique(raw[:, 11]).tolist(),
                           cmd_speed=stats(np.linalg.norm(pc[:, 3:5], axis=1)),
                           cmd_acc=stats(np.linalg.norm(pc[:, 6:8], axis=1)),
                           frame_offset_xy=stats(np.linalg.norm(pp[:, :2] - pl[:, :2], axis=1)))
summary['tracking']['velocity_ff_difference'] = stats(np.linalg.norm(raw[:, 5:7] - pc[:, 3:5], axis=1))
summary['tracking']['acc_ff_difference'] = stats(np.linalg.norm(raw[:, 8:10] - pc[:, 6:8], axis=1))

def latest(a, t):
    return a[np.clip(np.searchsorted(a[:, 0], t, side='right') - 1, 0, len(a)-1)]

cl = latest(cmd, tr)
ll = latest(lio, tr)
mp = latest(pose, tr)
ep = raw[:, 2:4] - mp[:, 2:4]
eold = cl[:, 2:4] - ll[:, 2:4]
limited = eold * np.minimum(1, .30 / np.maximum(1e-9, np.linalg.norm(eold, axis=1)))[:, None]
valid_gain = np.linalg.norm(limited, axis=1) > .05
summary['inferred_kp'] = stats(np.sum(ep[valid_gain]*limited[valid_gain], axis=1) / np.sum(limited[valid_gain]**2, axis=1))
limited *= args.kp
summary['controller'] = dict(position_mapping_residual=stats(np.linalg.norm(ep-limited, axis=1)),
                             command_age_at_raw=stats(tr-cl[:, 1]),
                             lio_age_at_raw=stats(tr-ll[:, 1]),
                             position_correction=stats(np.linalg.norm(ep, axis=1)))
summary['turns'] = []
for lo, hi in [args.turn_windows[:2], args.turn_windows[2:]]:
    valid = (tr >= lo) & (tr <= hi)
    inds = np.flatnonzero(valid)
    k = inds[np.argmax(abs(vv[valid, 1]))]
    q = inds[np.argmax(err[valid])]
    summary['turns'].append(dict(window=[lo, hi],
        peak_actual_y_velocity=dict(t=float(tr[k]), value=float(vv[k, 1]),
            cmd_y_velocity=float(pc[k, 4]), cmd_y_acceleration=float(pc[k, 7]),
            position_correction_y=float(ep[k, 1])),
        max_position_error=dict(t=float(tr[q]), value=float(err[q])),
        actual_y_range=[float(np.min(pl[valid, 1])), float(np.max(pl[valid, 1]))],
        cmd_y_range=[float(np.min(pc[valid, 1])), float(np.max(pc[valid, 1]))]))
after = possp[possp[:, 0] > tr[-1]]
if len(after):
    switch = after[0]
    land_times = [s['t'] for s in states if s['mode'] == 'AUTO.LAND' and s['t'] > switch[0]]
    end = (pose[:, 0] >= switch[0]) & (pose[:, 0] <= (land_times[0] if land_times else pose[-1, 0]))
    actual_end = pose[end]
    v_at_switch = interp(vel, [switch[0]], [2, 3])[0]
    summary['finish'] = dict(switch_time=float(switch[0]), target= switch[2:5].tolist(),
        velocity_at_switch=v_at_switch.tolist(),
        px4_x_overshoot=float(np.max(actual_end[:, 2])-switch[2]),
        px4_final_xy=actual_end[-1, 2:4].tolist(),
        last_raw=raw[-1].tolist())

summary['estimator_flag_counts'] = {k: sum(bool(e[k]) for e in estimators)
                                     for k in estimators[0] if k != 't'}
changes = np.flatnonzero(np.diff(cmd[:, 11]) != 0) + 1
summary['trajectory_changes'] = []
for i in changes:
    summary['trajectory_changes'].append(dict(t=float(cmd[i, 0]), id=int(cmd[i, 11]),
        position_step=float(np.linalg.norm(cmd[i, 2:4] - cmd[i-1, 2:4])),
        velocity_step=float(np.linalg.norm(cmd[i, 5:7] - cmd[i-1, 5:7])),
        acceleration_step=float(np.linalg.norm(cmd[i, 8:10] - cmd[i-1, 8:10]))))
summary['sampled_timeline'] = []
for t in np.arange(np.ceil(tr[0]), tr[-1], 1):
    i = np.argmin(abs(tr - t))
    summary['sampled_timeline'].append(dict(t=float(tr[i]), cmd_xy=pc[i, :2].tolist(),
        lio_xy=pl[i, :2].tolist(), error=float(err[i]), actual_v=vv[i, :2].tolist(),
        cmd_v=pc[i, 3:5].tolist(), cmd_a=pc[i, 6:8].tolist(), raw_p=raw[i, 2:4].tolist()))

fig, axes = plt.subplots(3, 2, figsize=(14, 13))
ax = axes[0, 0]
active = (lio[:, 0] >= tr[0]) & (lio[:, 0] <= tr[-1])
ax.plot(cmd[:, 2], cmd[:, 3], label='EGO command', alpha=.8)
ax.plot(lio[active, 2], lio[active, 3], label='FAST-LIO actual')
ax.scatter(lio[active, 2][::20], lio[active, 3][::20], c=lio[active, 0][::20], cmap='viridis', s=12)
ax.set_aspect('equal', adjustable='datalim')
ax.set(xlabel='X (m)', ylabel='Y (m)', title='Horizontal trajectory')
axes[0, 1].plot(tr, err, label='EGO position - LIO position')
axes[0, 1].axhline(.3, color='r', ls='--', label='Position correction cap 0.30 m')
axes[0, 1].set(ylabel='Error (m)', title='Tracking error at raw setpoint times')
for j in range(2):
    ax = axes[1, j]
    ax.plot(cmd[:, 0], cmd[:, 5+j], label='EGO velocity')
    ax.plot(tr, raw[:, 5+j], label='Raw velocity FF', ls='--')
    ax.plot(vel[:, 0], vel[:, 2+j], label='PX4 actual velocity')
    ax.set(ylabel='Velocity (m/s)', title=('X', 'Y')[j] + ' velocity')
ax = axes[2, 0]
ax.plot(cmd[:, 0], np.linalg.norm(cmd[:, 8:10], axis=1), label='EGO acceleration norm')
ax.plot(tr, np.linalg.norm(raw[:, 8:10], axis=1), label='Raw acceleration FF norm')
ax.set(ylabel='Acceleration (m/s2)', title='Acceleration feedforward')
ax = axes[2, 1]
ax.plot(cmd[:, 0], cmd[:, 11], label='Trajectory ID')
ax.set(ylabel='ID', title='Replanning')
for ax in axes.flat:
    ax.grid(alpha=.3)
    ax.legend(fontsize=8)
for ax in axes[1:, :].flat:
    ax.set_xlim(max(0, tr[0]-2), min(lio[-1, 0], tr[-1]+3))
    ax.set_xlabel('Bag time (s)')
fig.tight_layout()
fig.savefig(ROOT / 'tracking.png', dpi=150)
(ROOT / 'metrics.json').write_text(json.dumps(summary, indent=2))
print(json.dumps({k: v for k, v in summary.items() if k not in ['sampled_timeline', 'estimator']}, indent=2))
print('Estimator rows:', len(estimators))
