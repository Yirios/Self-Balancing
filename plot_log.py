"""Plot log CSV: theta_1, theta_2 (top) + wheel velocities (bottom).

Usage:
    uv run python plot_log.py log_100_2hz.csv
    uv run python plot_log.py log_100_2hz.csv -t 2 5    # zoom t=2-5s
    uv run python plot_log.py log_100_2hz.csv --ylim 18 # y-axis +/-18deg
"""
import argparse, csv
import numpy as np
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description="Plot theta_1, theta_2 + wheel velocities")
parser.add_argument("file", help="CSV log file")
parser.add_argument("-t", "--timerange", nargs=2, type=float, default=None,
                    help="Time range to plot, e.g. -t 2 5")
parser.add_argument("--ylim", type=float, default=None,
                    help="Y-axis limit for angle plot (symmetric +/-)")
args = parser.parse_args()

with open(args.file) as f:
    rows = list(csv.DictReader(f))
t = np.array([float(r['time_s']) for r in rows])
th1 = np.array([float(r['theta_1']) for r in rows])
th2 = np.array([float(r['theta_2']) for r in rows])
thdotL = np.array([float(r['theta_L_dot']) for r in rows])
thdotR = np.array([float(r['theta_R_dot']) for r in rows])

if args.timerange:
    mask = (t >= args.timerange[0]) & (t <= args.timerange[1])
    t, th1, th2, thdotL, thdotR = t[mask], th1[mask], th2[mask], thdotL[mask], thdotR[mask]

name = args.file.split("/")[-1]
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 8), sharex=True)

ax1.plot(t, np.rad2deg(th1), 'b', alpha=0.8, label='theta_1 (body)')
ax1.plot(t, np.rad2deg(th2), 'r', alpha=0.8, label='theta_2 (pendulum)')
ax1.axhline(0, color='gray', linestyle='-', linewidth=0.5)
if args.ylim:
    ax1.set_ylim(-args.ylim, args.ylim)
ax1.set_ylabel('Angle (deg)')
ax1.set_title(f'{name}  ({t[-1]-t[0]:.1f}s)')
ax1.legend(loc='upper left')
ax1.grid(True, alpha=0.3)

ax2.plot(t, thdotL, 'orange', alpha=0.7, label='theta_L_dot (left wheel)')
ax2.plot(t, thdotR, 'purple', alpha=0.7, label='theta_R_dot (right wheel)')
ax2.axhline(0, color='gray', linestyle='-', linewidth=0.5)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Wheel velocity (rad/s)')
ax2.legend(loc='upper left')
ax2.grid(True, alpha=0.3)

plt.tight_layout()
out = name.replace('.csv', '.png')
plt.savefig(out, dpi=150)
print(f'Saved {out}')
