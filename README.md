# AK80-9 ActPack Gearbox Fix

Diagnosing and fixing gearbox slip in an AK80-9 Dephy ActPack.

Locomotor Control Systems Laboratory, University of Michigan.

## The problem

On the exoskeleton, the actuator's input (motor) started spinning on its own at high torque while the output stayed still. Bench tests showed the drivetrain slipping internally at about 11 Nm commanded output torque.

## Root cause

The **sun gear lost its press fit** on the motor rotor. The rotor kept turning while the sun gear stayed still against the loaded output. The ring gear was the first suspect (the Gen 2 ring has no anti-rotation key; Gen 1 had one), but witness marks showed the ring held and the sun moved.

**Fix:** the sun gear was replaced. After the fix, the actuator held up to 25 Nm with no slip.

## How we found it

1. **Grounded ramp test.** The output was locked to a fixed plate, and the commanded torque was stepped up while logging the motor encoder. With the output grounded, the motor should barely move. A sudden jump in motor angle and speed means the drivetrain is slipping.
2. **Teardown.** The planet gears showed no chipped teeth or tool damage, leaving two suspects: the sun gear on its rotor seat and the ring gear in its flange.
3. **Witness marks.** A Sharpie line was drawn across the sun gear and its seat, and across the ring gear and flange.
4. **Re-test and second teardown.** Slip recurred. The ring marks stayed aligned; the sun mark was offset.
5. **Verification.** After replacing the sun gear, the ramp test ran to 25 Nm with no slip.

| | Before fix | After fix |
|---|---|---|
| Slip | Full slip at 11 Nm | None up to 25 Nm |
| Motor speed | Up to ~7,000 deg/s | ~0 deg/s |
| Motor travel (max − min) | ~62,000° | ~26° (coupler wind-up) |

## Setup

- AK80-9 Dephy ActPack (firmware 7.2.0), 9:1 planetary gearbox
- Raspberry Pi running the Dephy `flexsea` Python library, current control
- Output grounded through a flexible coupler and a Futek torque sensor to a fixed plate
- Futek read over I2C through an ADS1115 ADC (address `0x48`). In the end, the motor encoder alone was enough to detect slip.

Torque model: `output torque = current (A) × 0.145 Nm/A × 9`

## Scripts

| Script | What it does |
|---|---|
| `torque_ramp_test.py` | Main test. Ramps output torque in 0.5 Nm steps (2 s each, 1 s each above 20 Nm) up to 25 Nm with the output grounded. Logs commanded torque, Futek torque, current, motor travel, and speed to CSV. Prints CREEP and SLIP events. Stops on temperature limit or supply current above 12 A. |
| `motor_not_grounded_test.py` | Commands a fixed torque with no logging and prints current and motor angle. Used with the output free to check that the motor spins and the encoder reads. |
| `find_poles.py` | Runs Dephy's `find_poles()` once from the Pi. The Dephy GUI can also do this. |
| `plot_encoder_ramp.py` | Plots a ramp CSV using only the encoder: commanded torque, motor travel, and motor speed, plus a per-step table. |

### Running the ramp test

On the Pi, with the output grounded:

```bash
python3 torque_ramp_test.py
```

Edit the settings at the top of the script (step size, end torque, hold times, `STOP_ON_SLIP`, safety limits) before running.

### Plotting

```bash
python plot_encoder_ramp.py torque_ramp_YYMMDD_HHMMSS.csv
```

Saves a PNG next to the CSV.

### Dependencies

On the Pi:

```bash
pip3 install --break-system-packages flexsea numpy adafruit-blinka adafruit-circuitpython-ads1x15
```

For plotting: `pip install numpy matplotlib`

`futek.py`, `AdcManager.py`, and `SoftRealtimeLoop.py` must be in the same folder as the test scripts. In `AdcManager.py`, use `AnalogIn(self.ads, 0)` instead of `ADS.P0` with newer versions of the Adafruit library.

## Safety

- Check the rotor's permanent magnets for metal shavings before reassembly. They hold ferrous debris that can rub the windings or get into the gears.
- If a screw is seized in the housing, do not force it out. Level it flush with the housing and keep debris out of the motor.
- The Gen 2 ring gear has no key. Avoid prying or impact loads on it, and re-mark the ring and sun after every rebuild.
- **Label the three motor phase leads before desoldering them** (see below).
- Keep the 12 A supply-current limit on for high-torque runs; the board has a 15 A fuse.

## Appendix: drive fault (`status_ex = 8`)

After the motor phase leads were resoldered in an unknown order, the motor held stiff instead of spinning. Running Find Poles then stored a bad calibration, which set `status_ex = 8` and blocked all motor current.

- Only **3 of the 6** lead orders work (cyclic rotations, e.g., A-B-C, B-C-A, C-A-B). Swapping any two leads reverses the phase sequence.
- A phase lead touching a battery-voltage component also kept Find Poles from completing.

**Fix:** clear the contact, swap two leads back, run Find Poles from the Dephy GUI (the motor ticks for about 30 s and the output turns ~10° or more), then power cycle.
