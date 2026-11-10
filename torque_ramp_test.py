"""
Torque staircase test: find where the transmission slips.

Commands output torque in 0.5 Nm steps (0.5, 1.0, ... 25 Nm) with the output
locked against the Futek. Steps up to 20 Nm are held 2 s; steps above 20 Nm
are held only 1 s. Slip is detected and logged but does NOT stop the test.
A battery-current guard stops the test to protect the board's 15 A fuse.

Model:  output torque = current (A) * kt * gear_ratio = I * 0.145 * 9

Two events are detected and printed:
  CREEP: the motor has turned more than CREEP_TRAVEL_DEG from its start
         (the slow stick-slip we saw before the big slip). Test keeps going.
  SLIP:  the motor spins fast (|mot_vel| > SLIP_VEL_DEG_S for SLIP_SAMPLES
         loops in a row) OR has turned more than SLIP_TRAVEL_DEG.
         Torque is cut to 0 and the test stops.

CSV columns (torques at the OUTPUT side):
    time_s, commanded_torque_Nm, actual_torque_Nm,
    commanded_current_A, measured_current_A,
    motor_travel_deg, motor_velocity_deg_s, temperature_C, slip
"""

import csv
import time
from time import strftime

import numpy as np
from flexsea.device import Device
from futek import Big100NmFutek
from SoftRealtimeLoop import SoftRealtimeLoop

# ---------------- Motor constants ----------------
kt = 0.145                       # Nm/A (motor side)
gear_ratio = 9
MOTOR_CLICKS_PER_REVOLUTION = 16384
DEG_PER_CLICK = 360.0 / MOTOR_CLICKS_PER_REVOLUTION

# ---------------- Hardware settings ----------------
PORT = "/dev/ttyACM0"
BAUD = 230400
FIRMWARE = "7.2.0"
DT = 0.004                       # loop period (s) -> 250 Hz
FUTEK_SIGN = -1                  # flip to +1 if the 5 Nm test now reads positive

# ---------------- Staircase (edit these) ----------------
TORQUE_START_NM = 0.5
TORQUE_END_NM = 25.0
TORQUE_STEP_NM = 0.5
STEP_HOLD_S = 2.0                # time at each torque level up to HIGH_TORQUE_NM (s)
HIGH_TORQUE_NM = 20.0            # levels above this are held for STEP_HOLD_HIGH_S
STEP_HOLD_HIGH_S = 1.0           # time at each level above 20 Nm (s)
REST_S = 2.0                     # 0 Nm before and after the staircase (s)

# ---------------- Safety and slip detection (edit these) ----------------
MAX_TORQUE_NM = 26.0             # hard clamp just above the 25 Nm target
TEMP_LIMIT_C = 60.0              # stop if ActPack temperature goes above this
STOP_ON_SLIP = False             # slip is printed and logged, but the test keeps going
BATT_CURRENT_LIMIT_A = 12.0      # stop if supply current exceeds this (board fuse is 15 A); set None to disable

CREEP_TRAVEL_DEG = 30.0          # creep warning (last run: ~3 deg at 6 Nm, ~50 deg by 7 Nm)
SLIP_VEL_DEG_S = 1000.0          # last run: creep < ~170 deg/s, full slip = thousands of deg/s
SLIP_SAMPLES = 5                 # fast for this many loops in a row (= 20 ms) to count as slip
SLIP_TRAVEL_DEG = 720.0          # backup: motor turned this far from start = slip


def torque_to_current_A(torque_out_Nm):
    """Output torque (Nm) -> motor current (A)."""
    return torque_out_Nm / (kt * gear_ratio)


def build_schedule():
    """List of (start_time, end_time, torque)."""
    schedule = []
    t = 0.0
    schedule.append((t, t + REST_S, 0.0))
    t += REST_S
    for tau in np.arange(TORQUE_START_NM, TORQUE_END_NM + 1e-9, TORQUE_STEP_NM):
        hold = STEP_HOLD_HIGH_S if tau > HIGH_TORQUE_NM + 1e-9 else STEP_HOLD_S
        schedule.append((t, t + hold, float(round(tau, 3))))
        t += hold
    schedule.append((t, t + REST_S, 0.0))
    t += REST_S
    return schedule, t


def torque_at(t, schedule):
    for start, end, tau in schedule:
        if start <= t < end:
            return tau
    return 0.0


def main():
    schedule, total_time = build_schedule()
    file_name = "torque_ramp_" + strftime("%y%m%d_%H%M%S") + ".csv"

    print("Staircase: {:.1f} to {:.1f} Nm in {:.1f} Nm steps ({:.0f} s each, {:.0f} s above {:.0f} Nm). "
          "Peak motor current {:.2f} A. Total {:.0f} s.".format(
              TORQUE_START_NM, TORQUE_END_NM, TORQUE_STEP_NM, STEP_HOLD_S, STEP_HOLD_HIGH_S,
              HIGH_TORQUE_NM, torque_to_current_A(TORQUE_END_NM), total_time))

    # ---- Start motor at zero current ----
    device = Device(port=PORT, firmwareVersion=FIRMWARE, baudRate=BAUD)
    device.open()
    device.start_streaming(frequency=500)
    time.sleep(1.0)
    device.set_gains(40, 400, 0, 0, 0, 128)   # kp, ki, kd, K, B, ff (current loop)
    device.command_motor_current(0)

    state = device.read()
    start_motor_angle_deg = state["mot_ang"] * DEG_PER_CLICK

    # ---- Zero the Futek with no load on it ----
    futek = Big100NmFutek()
    print("Zeroing Futek. Keep the output unloaded...")
    futek.calibrate_loadcell()

    f = open(file_name, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(["time_s", "commanded_torque_Nm", "actual_torque_Nm",
                     "commanded_current_A", "measured_current_A",
                     "motor_travel_deg", "motor_velocity_deg_s", "temperature_C", "slip"])

    slip = 0
    creep_reported = False
    fast_count = 0
    last_level = None
    print("Running for up to {:.0f} s. Ctrl+C to stop.".format(total_time))
    try:
        for t in SoftRealtimeLoop(dt=DT):
            if t > total_time:
                break

            # ---- Read sensors ----
            state = device.read()
            futek.update()
            futek.get_torque()
            actual_torque = FUTEK_SIGN * futek.torque

            motor_travel_deg = state["mot_ang"] * DEG_PER_CLICK - start_motor_angle_deg
            motor_vel_deg_s = state["mot_vel"]
            temperature = state["temperature"]
            level = torque_at(t, schedule)

            # ---- Safety: temperature ----
            if temperature > TEMP_LIMIT_C:
                print("\nActPack at {} C. Stopping.".format(temperature))
                break

            # ---- Safety: supply current (protects the board fuse) ----
            if BATT_CURRENT_LIMIT_A is not None and abs(state["batt_curr"]) / 1000.0 > BATT_CURRENT_LIMIT_A:
                print("\nSupply current {:.1f} A > {:.1f} A limit at {:.1f} Nm. Stopping.".format(
                    abs(state["batt_curr"]) / 1000.0, BATT_CURRENT_LIMIT_A, level))
                break

            # ---- Creep warning (keeps running) ----
            if not creep_reported and abs(motor_travel_deg) > CREEP_TRAVEL_DEG:
                creep_reported = True
                print("\nCREEP at t = {:.3f} s, commanded {:.1f} Nm, Futek {:.2f} Nm, travel {:.1f} deg".format(
                    t, level, actual_torque, motor_travel_deg))

            # ---- Slip check: input spinning while the output is locked ----
            fast_count = fast_count + 1 if abs(motor_vel_deg_s) > SLIP_VEL_DEG_S else 0
            if not slip and (fast_count >= SLIP_SAMPLES or abs(motor_travel_deg) > SLIP_TRAVEL_DEG):
                slip = 1
                print("\nSLIP at t = {:.3f} s, commanded {:.1f} Nm, Futek {:.2f} Nm, "
                      "travel {:.1f} deg, speed {:.0f} deg/s".format(
                          t, level, actual_torque, motor_travel_deg, motor_vel_deg_s))

            # ---- Command torque through current ----
            torque_cmd = float(np.clip(level, -MAX_TORQUE_NM, MAX_TORQUE_NM))
            if slip and STOP_ON_SLIP:
                torque_cmd = 0.0
            current_cmd_A = torque_to_current_A(torque_cmd)
            device.command_motor_current(int(current_cmd_A * 1000.0))  # mA

            writer.writerow([t, torque_cmd, actual_torque, current_cmd_A,
                             state["mot_cur"] / 1000.0, motor_travel_deg,
                             motor_vel_deg_s, temperature, slip])

            if slip and STOP_ON_SLIP:
                break

            # ---- Progress print once per level ----
            if torque_cmd != last_level:
                print("t = {:5.1f} s  ->  {:4.1f} Nm".format(t, torque_cmd))
                last_level = torque_cmd

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        device.command_motor_current(0)
        device.stop_motor()
        device.stop_streaming()
        device.close()
        f.close()
        print("Saved", file_name)


if __name__ == "__main__":
    main()
