"""
Command ONE Dephy ActPack at a fixed output torque (current control). No logging.
Prints measured current and motor angle twice a second.

Model:  output torque = current (A) * kt * gear_ratio = I * 0.145 * 9
"""

import time

from flexsea.device import Device
from SoftRealtimeLoop import SoftRealtimeLoop

# ---------------- Motor constants ----------------
kt = 0.145          # Nm/A (motor side)
gear_ratio = 9

# ---------------- Settings (edit these) ----------------
PORT = "/dev/ttyACM0"
BAUD = 230400
FIRMWARE = "7.2.0"
DT = 0.004              # loop period (s)

TORQUE_NM = 0.0         # output torque to command
HOLD_S = 15.0           # how long to hold it (s)
TEMP_LIMIT_C = 60.0     # stop if ActPack temperature goes above this


def main():
    current_mA = int(TORQUE_NM / (kt * gear_ratio) * 1000.0)

    device = Device(port=PORT, firmwareVersion=FIRMWARE, baudRate=BAUD)
    device.open()
    device.start_streaming(frequency=500)
    time.sleep(1.0)
    device.set_gains(40, 400, 0, 0, 0, 128)   # kp, ki, kd, K, B, ff (current loop)
    device.command_motor_current(0)

    print("Commanding {:.1f} Nm ({} mA) for {:.0f} s. Ctrl+C to stop.".format(
        TORQUE_NM, current_mA, HOLD_S))
    try:
        for t in SoftRealtimeLoop(dt=DT):
            if t > HOLD_S:
                break
            state = device.read()

            # Print current and motor angle every 0.5 s
            if int(t / 0.5) != int((t - DT) / 0.5):
                print("current {:.2f} A   motor angle {:.1f} deg".format(
                    state["mot_cur"] / 1000.0, state["mot_ang"] * 360.0 / 16384))

            if state["temperature"] > TEMP_LIMIT_C:
                print("ActPack at {} C. Stopping.".format(state["temperature"]))
                break
            device.command_motor_current(current_mA)

    except KeyboardInterrupt:
        print("Stopped by user.")

    finally:
        device.command_motor_current(0)
        device.stop_motor()
        device.stop_streaming()
        device.close()
        print("Done.")


if __name__ == "__main__":
    main()
