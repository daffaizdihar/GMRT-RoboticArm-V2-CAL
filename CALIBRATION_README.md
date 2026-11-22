# GMRT Robotic Arm V2 — Individual Servo Calibration Patch

## What changed
- Adds `CAL,<joint 1..5>,<servoDeg>` for a single servo only.
- Adds `CALSTOP` to detach the calibration servo.
- `STOP` also detaches the calibration servo.
- `DISARM` detaches calibration and all servo channels.
- Main ARM lock remains `CALIBRATION_VERIFIED = false`.
- Calibration mode rejects commands outside +/-3 degrees from the configured raw-servo baseline.
- This is a commissioning aid, not a declaration that the configured baseline is mechanically safe.

## Important safety notes
- Remove the arm's payload and keep hands clear.
- Secure the arm so gravity cannot cause a link to fall when servo torque is absent.
- Keep a physical way to disconnect the external servo supply immediately.
- Verify 5V supply current capacity and common ground; do not power five SG90s from ESP32 5V.
- Test one joint at a time. `CAL` detaches the previous calibration servo before attaching another.
- Raw `servoDeg` is the angle passed to `Servo.write()`, NOT the joint angle.
- Configured baselines are J1-J4 = 90 degrees and J5 = 10 degrees. J5 baseline may be near an endpoint; inspect the mechanism before attaching it.
- Do not run `ARM`, `J`, or IK commands during this stage.
- Calibration servo detach does not remove external power; it only stops the PWM signal.

## Serial commands
- `STATUS`
- `CAL,1,90` through `CAL,4,90` (baseline for J1-J4)
- `CAL,1,87` / `CAL,1,93` (small +/-3 degree checks)
- `CAL,5,10` (J5 configured baseline; only after verifying gripper linkage is unloaded and not at a hard stop)
- `CALSTOP`
- `STOP`
- `DISARM`

The allowed window is +/-3 degrees around each configured raw-servo baseline. If the baseline is not mechanically safe, do not issue CAL; first revise the calibration baseline only after physically measuring the servo/linkage alignment.

## Upload
1. Back up your current V2 project.
2. Replace `include/` and `src/` files with this archive's versions.
3. Keep `CALIBRATION_VERIFIED = false`.
4. Build, then Upload.
5. Open Monitor at 115200.
