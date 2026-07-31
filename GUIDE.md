# QUICK GUIDE — chi-Haptics Bridge

Every command runs **inside the Docker container** unless marked `[HOST]`.
Two separate workflows live in this repo:

- **Teleoperation** (§1–§6) — the production system: robot torque → pad pressure.
- **Pad characterisation** (§7–§10) — the test rig: measure and tune a new pad.

They use **different Arduino firmware**. Flash the right one before starting.

| Workflow | Sketch | Runs from |
|---|---|---|
| Teleoperation | `arduino/pressure_tracker` | container (ROS) |
| Characterisation | `arduino/pad_characterisation` | `[HOST]` (plain Python) |

---

## 0. Start the container

`[HOST]` — build and start (first time, or after changing the Dockerfile):
```bash
cd ~/kinova_haptic_ws/docker
docker compose up -d --build
```

`[HOST]` — just start it (normal case):
```bash
cd ~/kinova_haptic_ws/docker
docker compose up -d
```

`[HOST]` — open a shell inside:
```bash
docker exec -it kinova_haptic_humble bash
```

---

## 1. Build the ROS package

Required after every edit to any `.py` file, and after every `--force-recreate`:
```bash
cd /ros2_ws
colcon build
source install/setup.bash
```

Check the executables registered:
```bash
ros2 pkg executables kinova_haptic_teleop
```

Expect: `fake_torque_pub`, `kinova_haptic_bridge`, `kinova_haptic_bridge_sim`.

---

## 2. FAKE mode — bench test, no robot

**Terminal 1** — the bridge:
```bash
docker exec -it kinova_haptic_humble bash
source install/setup.bash
ros2 run kinova_haptic_teleop kinova_haptic_bridge_sim
```

**Terminal 2** — keyboard torque simulator (UP/DOWN arrows, `q` to quit):
```bash
docker exec -it kinova_haptic_humble bash
source install/setup.bash
ros2 run kinova_haptic_teleop fake_torque_pub
```

Publish a fixed value instead of using the keyboard:
```bash
ros2 topic pub /kinova/sim_torque std_msgs/msg/Float32 "{data: 4.0}" -r 10
```

Test deflation between two **non-zero** targets — a target of 0 triggers the full purge, not PFM:
```bash
ros2 topic pub /kinova/sim_torque std_msgs/msg/Float32 "{data: 1.0}" -r 10
```

---

## 3. REAL mode — with the robot

Preferred — use the params file:
```bash
docker exec -it kinova_haptic_humble bash
source install/setup.bash
ros2 run kinova_haptic_teleop kinova_haptic_bridge --ros-args \
  --params-file /config/bridge_real.yaml
```

Command line, for a one-off override:
```bash
docker exec -it kinova_haptic_humble bash
source install/setup.bash
ros2 run kinova_haptic_teleop kinova_haptic_bridge --ros-args \
  -p serial_port:=/dev/serial/by-id/usb-Arduino_UNO_WiFi_R4_CMSIS-DAP_F412FA654890-if01 \
  -p torque_axis:="'y'" \
  -p max_input:=6.0
```

The `"'y'"` quoting is required. YAML reads bare `y` as the boolean `true`, which fails the string type check. `x` and `z` do not have this problem.

---

## 4. The params file

`config/bridge_real.yaml` on the host, mounted read-only at `/config/` inside the container. Edit in VS Code, no rebuild needed — just restart the node.

```yaml
/kinova_haptic_bridge:
  ros__parameters:
    serial_port: "/dev/serial/by-id/usb-Arduino_UNO_WiFi_R4_CMSIS-DAP_F412FA654890-if01"
    torque_axis: "y"
    max_input: 6.0
    max_pressure: 60.0
    baseline_alpha: 0.995
    engage_threshold: 0.30
    release_threshold: 0.15
    auto_recover: true
```

| Parameter | Meaning | Tuning |
|---|---|---|
| `serial_port` | Arduino device. Use the `by-id` path — it survives re-enumeration, `ttyACM0` may not | — |
| `torque_axis` | wrench component to render: `"x"`, `"y"`, `"z"` | confirm empirically by pushing and watching the echo |
| `max_input` | interaction torque (Nm) mapping to full pressure | peak seen during a real push (~6 Nm at the valve stop) |
| `max_pressure` | kPa at full scale | 60 is the firmware clamp |
| `baseline_alpha` | how fast the gravity baseline adapts; closer to 1 = slower | lower if idle output drifts above 0 |
| `engage_threshold` | Nm above baseline counting as contact | raise if noise triggers phantom contact |
| `release_threshold` | Nm below which contact ends; must be < engage | the gap is the hysteresis band |
| `auto_recover` | force USB re-enumeration when telemetry stalls | leave `true` |

---

## 5. Monitoring

What the Arduino reports it is actually doing:
```bash
ros2 topic echo /pressure/actual_kpa
ros2 topic echo /pressure/target_kpa_echo
```

`target_kpa_echo` is ground truth — it is what the **Arduino** believes the target is, not what the bridge sent. If they disagree, commands are not landing.

Raw serial, bypassing ROS entirely (stop the bridge first):
```bash
python3 -c "
import serial,time
s=serial.Serial('/dev/ttyACM0',115200,timeout=2); time.sleep(3)
[print(s.readline().decode(errors='ignore').strip()) for _ in range(40)]
"
```
Shows the `State:` field — `IDLE`, `INFLATE`, `DEFLATE`, `HOLD`, `PURGE`.

---

## 6. Calibration procedure (real mode)

1. Start the bridge, do not touch the arm. `target_kpa_echo` should read **0.00**. If it drifts up, lower `baseline_alpha` to 0.99.
2. Move the arm through free space. Still ~0. If it blips on every move, lower `baseline_alpha` further.
3. Push the handle as in the real task. Note the peak on the echo.
4. Set `max_input` to that peak so a full push gives full pressure.
5. Watch one complete 90° turn: the value should rise, follow the stiffness, and **hold** at the stop without fading. If it fades mid-push, contact detection dropped out — raise `release_threshold`.

---

## 7. Pad characterisation — overview

Run this whenever a new pad design is built. Three phases:

1. **Safe limit** — step pressure manually until you judge the pad is at its limit.
2. **Inflation transient** — measure the rise time. This becomes `T_ref`.
3. **Deflation optimisation** — Bayesian optimisation of the reverse-PFM bounds so the descent approximates a linear ramp of duration `T_ref`.

Results go in `pads/<pad_name>/`, one folder per design, so successive pads stay comparable.

**Flash the characterisation firmware first.** The scripts speak a different serial protocol (`U`/`D`/`I`/`F` commands, `D,` telemetry) and will sit silent against `pressure_tracker.ino`.

`[HOST]`:
```bash
sudo lsof /dev/ttyACM0     # must print nothing — see §11 if the bridge holds it
cd ~/kinova_haptic_ws/arduino
./flash_arduino.sh         # choose 2) pad_characterisation
```

Scripts run on the **host**, not in the container — they need only pyserial, numpy and matplotlib, no ROS:
```bash
sudo apt install python3-numpy python3-matplotlib -y
cd ~/kinova_haptic_ws/scripts
```

---

## 8. Phase 1 & 2 — safe limit and inflation

```bash
python3 pad_logger.py --port /dev/ttyACM0 --out ../pads/pad_v2_55x30_shore10
```

Menu:
- `1` manual stepping — UP/DOWN in 2 kPa steps, `z` vents, `q` exits. Reports the max reached.
- `2` inflation test — vent, settle, full pump to target. Reports **RISE TIME**, writes `inflation.csv` and `inflation.png`.
- `3` single deflation test — one descent at chosen `T_max`/`T_min`. Useful for sanity-checking before a full optimisation run.

**Do phase 1 with the pad shielded, not on your arm.** The firmware refuses to exceed `HARD_CEILING_KPA = 80.0`, but you are looking for where the pad fails.

Note the rise time from phase 2 — that is `--tref` for the optimiser.

A closed-loop hold will *not* reveal a leak: bang-bang keeps topping up and hides it. The only open-loop window is the deflation test's settle phase, which is why the firmware tops the pad up immediately before each descent.

---

## 9. Phase 3 — deflation optimisation

```bash
python3 pad_optimize.py --port /dev/ttyACM0 \
  --start 30 --tref 2244 --trials 25 \
  --out ../pads/pad_v2_55x30_shore10
```

25 trials at roughly 15–25 s each, so about 10 minutes. First 6 are random seeds, the rest Bayesian (GP surrogate, Matern 5/2, Expected Improvement).

Watch the printed columns:

| Column | Meaning | What to check |
|---|---|---|
| `MSE` | error against the ideal linear descent | should drop sharply once BO starts |
| `dur` | actual descent duration | compare against `--tref` |
| `p0` | **measured** start pressure | must be consistent across trials, and close to `--start` |
| `n` | samples in the trace | too few means the descent terminated early |

`p0` is the one to watch. If it varies by more than ~1 kPa, trials are not comparable and the optimum is fitting noise rather than parameters.

Outputs: `optimum.json`, `optimisation_history.csv`, `deflation_best.csv`.

Combined figure:
```bash
python3 plot_characterisation.py --dir ../pads/pad_v2_55x30_shore10
```

Then paste `T_max_ms` / `T_min_ms` from `optimum.json` into `pressure_tracker.ino` as `settle_max` / `settle_min`.

**Anchor mismatch to watch for.** The production firmware interpolates between fixed anchors (10 and 50 kPa). The characterisation rig anchors on the test target and `DEFLATE_END_KPA`. If you characterise at a different start pressure, the bounds will not transfer directly — either characterise at the production anchor, or update the production anchors to match.

---

## 10. Reading the characterisation traces

**Sensor transients during bursts.** While the valve is open, air flows past the sensor and the reading dips sharply, recovering over roughly 60–80 ms after close. At fast cycling the off-period is shorter than that recovery, so the firmware latches one clean sample immediately *before* each burst rather than trying to find a quiet window afterwards. The logged curve should be monotonic; visible spikes mean the latch is not being applied.

**Steps per descent.** `RESULT,DEFLATE` reports the burst count. Around 30 bursts over a 20 kPa descent is ~0.6 kPa per burst — below the perceptual threshold for pressure, so the system is not volume-limited. If perceived roughness persists at that resolution, the cause is the valve's mechanical impulse travelling down the tubing, not the pressure steps. That is fixed by decoupling the valve (mount it off the pad, compliant tubing, foam under the body), not by control parameters.

**Shape versus duration.** The MSE objective weights profile shape more heavily than total duration, so the winning candidate may finish faster than `T_ref`. If matching the inflation duration matters more than linearity, the duration term needs explicit weighting in `score()`.

---

## 11. Flashing the Arduino

`[HOST]` — the port must be free. `arduino-cli` sends a 1200-baud touch to enter the bootloader; if anything holds the port it cannot, and the upload fails with `Device unsupported`.

```bash
sudo lsof /dev/ttyACM0
```

If the bridge appears, stop it:
```bash
sudo kill <PID>
# or, if it is inside the container:
cd ~/kinova_haptic_ws/docker && docker compose stop
```

Then:
```bash
cd ~/kinova_haptic_ws/arduino
./flash_arduino.sh
```

If it still reports `Device unsupported`: **double-tap the reset button** to force the bootloader, check the new port with `ls -l /dev/serial/by-id/`, and upload to that port explicitly.

---

## 12. When something breaks

**Arduino goes deaf, reset button does not help.** The USB CDC endpoint has stalled while still enumerated — only a bus-level re-enumeration clears it. `auto_recover: true` handles this automatically in the bridge. Manually, `[HOST]`:
```bash
sudo python3 ~/kinova_haptic_ws/scripts/usb_reset.py
```

**Nothing inflates but telemetry looks fine.** The external DC supply for the pump and valve is not connected. This has caused this exact symptom more than once.

**Log alternates between two values.** Two publishers on the topic — usually a ghost node from a closed terminal:
```bash
ps aux | grep -E "fake_torque|kinova_haptic" | grep -v grep
kill <PID>
```
`ros2 node list` collapses duplicate node names, so it will *not* show the duplicate. Use `ros2 topic info <topic> --verbose` and check the publisher count.

**Deflation test inflates and holds forever.** The settle phase is not completing. Check the `State:` field in telemetry — `DEF_SETTLE` that never advances to `DEF_RUN` means the timer is being reset each pass.

**Arrow keys do nothing.** Nested TTYs (tmux inside VS Code inside `docker exec`) can swallow escape sequences. Test in a plain SSH terminal.

**"No executable found".** You did not `colcon build`, or did not `source install/setup.bash`.

**Nodes on the same machine cannot see each other.** The CycloneDDS peer list needs `localhost` in it, otherwise disabling multicast also disables same-machine discovery.

**Valve gets very hot.** The production firmware caps continuous valve-on time and forces a cooldown, printing `VALVE:thermal cap`. If it heats at idle, the MOSFET gate is floating before `setup()` runs — fit a 10 kΩ pulldown from SIG to GND on both modules. Firmware cannot protect the pins before the firmware is running.

---

## 13. Daily commit

`[HOST]`:
```bash
cd ~/kinova_haptic_ws
git add -A
git commit -m "describe what changed"
git push
```

Tag a working milestone:
```bash
git tag -a v0.5-pad-v2-characterised -m "Pad v2 BO: T_max=85, T_min=37"
git push --tags
```