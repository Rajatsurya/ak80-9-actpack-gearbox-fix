"""
Plot a torque staircase test using only the motor encoder (no Futek).

Usage:  python plot_encoder_ramp.py torque_ramp_XXXXXX_XXXXXX.csv
Saves:  a .png with the same name as the CSV (ending in _encoder.png).
"""

import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")            # no screen needed
import matplotlib.pyplot as plt

file_name = sys.argv[1]
d = np.genfromtxt(file_name, delimiter=",", names=True)
t = d["time_s"]
cmd = d["commanded_torque_Nm"]
travel = d["motor_travel_deg"]
vel = d["motor_velocity_deg_s"]
slip = d["slip"]

# ---- Per-step table: how far the motor turned during each torque level ----
print("\n cmd (Nm)   motor travel during step (deg)   max speed (deg/s)")
for level in np.unique(cmd[cmd > 0]):
    idx = np.where(cmd == level)[0]
    moved = travel[idx[-1]] - travel[idx[0]]
    vmax = np.max(np.abs(vel[idx]))
    print(" {:7.1f}   {:30.1f}   {:16.0f}".format(level, moved, vmax))

slip_i = np.argmax(slip > 0) if np.any(slip > 0) else None
if slip_i is not None:
    print("\nSLIP: input spinning at t = {:.3f} s (commanded {:.1f} Nm)".format(t[slip_i], cmd[slip_i]))
else:
    print("\nNo slip detected.")

# ---- Plot ----
fig, ax = plt.subplots(3, 1, sharex=True, figsize=(10, 9))

ax[0].plot(t, cmd, linewidth=2)
ax[0].set_ylabel("Commanded torque (Nm)")

ax[1].plot(t, travel, color="tab:gray")
ax[1].set_ylabel("Motor travel (deg)")

ax[2].plot(t, vel, color="tab:purple", linewidth=0.8)
ax[2].set_ylabel("Motor speed (deg/s)")
ax[2].set_xlabel("Time (s)")

for a in ax:
    a.grid(True)
    if slip_i is not None:
        a.axvline(t[slip_i], color="red", linestyle="--", label="Slip detected")
if slip_i is not None:
    ax[0].legend(loc="upper left")

out = file_name.rsplit(".", 1)[0] + "_encoder.png"
plt.tight_layout()
plt.savefig(out, dpi=150)
print("Saved", out)
