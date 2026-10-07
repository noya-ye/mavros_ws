import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
c = np.loadtxt(root/'position_cmd.csv', delimiter=',')
r = np.loadtxt(root/'mavros_setpoint_raw_local.csv', delimiter=',')
l = np.loadtxt(root/'fastlio2_lio_odom.csv', delimiter=',')
p = np.loadtxt(root/'mavros_local_position_pose.csv', delimiter=',')
v = np.loadtxt(root/'mavros_local_position_velocity_local.csv', delimiter=',')
def at(a, times, cols):
    return np.column_stack([np.interp(times, a[:, 0], a[:, j]) for j in cols])

jumps = np.flatnonzero(np.linalg.norm(np.diff(c[:, 2:4], axis=0), axis=1) > .03)+1
events = []
for i in jumps:
    events.append(dict(time=float(c[i, 0]), before=c[i-1, 2:11].tolist(),
                       after=c[i, 2:11].tolist(), step=float(np.linalg.norm(c[i, 2:4]-c[i-1, 2:4]))))
stop = c[jumps[-1], 0]
turn = l[(l[:, 0]>=59)&(l[:, 0]<stop)]
ref = c[(c[:, 0]>=54)&(c[:, 0]<stop), 2:4]
nearest = np.sqrt(np.min(np.sum((turn[:, None, 2:4]-ref[None, :, :])**2, axis=2), axis=1))
t = r[:, 0]
active = (t >= c[0, 0]) & (t < stop)
ce = at(c, t, [2, 3])-at(l, t, [2, 3])
moving_error = np.linalg.norm(ce[active], axis=1)
summary = dict(events=events, moving_tracking_error=dict(max=float(moving_error.max()),
    p95=float(np.percentile(moving_error,95))),
    second_turn_nearest_command_path_distance=dict(max=float(nearest.max()),
        p95=float(np.percentile(nearest,95))),
    actual_velocity_at_stop=at(v,[stop],[2,3])[0].tolist(),
    actual_lio_at_stop=at(l,[stop],[2,3])[0].tolist())
print(json.dumps(summary,indent=2))
(root/'events.json').write_text(json.dumps(summary,indent=2))

fig, axes = plt.subplots(3,1,figsize=(12,10),sharex=True)
axes[0].plot(c[:,0],c[:,3],label='EGO Y position')
axes[0].plot(l[:,0],l[:,3],label='FAST-LIO Y position')
axes[0].plot(r[:,0],r[:,3],label='Raw Y position target',ls='--')
axes[0].set_ylabel('Position (m)')
axes[1].plot(c[:,0],c[:,6],label='EGO Y velocity / feedforward')
axes[1].plot(v[:,0],v[:,3],label='PX4 Y velocity')
axes[1].plot(r[:,0],r[:,3]-at(p,t,[3])[:,0],label='Raw Y position correction (m)',ls='--')
axes[1].set_ylabel('Velocity (m/s), correction (m)')
axes[2].plot(c[:,0],c[:,9],label='EGO Y acceleration')
axes[2].plot(r[:,0],r[:,9],label='Raw Y acceleration',ls='--')
axes[2].set_ylabel('Acceleration (m/s2)')
for ax in axes:
    ax.axvline(stop,color='r',ls=':',label='Command switches to fixed hold')
    ax.grid(alpha=.3)
    ax.legend(fontsize=9)
axes[-1].set_xlim(62,69)
axes[-1].set_xlabel('Seconds since bag start')
fig.tight_layout()
fig.savefig(root/'second_turn.png',dpi=150)
