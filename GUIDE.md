# QUICK GUIDE — chi-Haptics Bridge

Every command below runs **inside the Docker container** unless marked `[HOST]`.

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

**Terminal 1** — the bridge (reads simulated torque, drives the Arduino):
```bash
docker exec -it kinova_haptic_humble bash
source install/setup.bash
ros2 run kinova_haptic_teleop kinova_haptic_bridge_sim
```

**Terminal 2** — the keyboard torque simulator (UP/DOWN arrows, `q` to quit):
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

Preferred: use the params file (avoids the YAML `y` → boolean trap):
```bash
docker exec -it kinova_haptic_humble bash
source install/setup.bash
ros2 run kinova_haptic_teleop kinova_haptic_bridge --ros-args \
  --params-file /config/bridge_real.yaml
```

Same thing on the command line, if you need a one-off override:
```bash
ros2 run kinova_haptic_teleop kinova_haptic_bridge --ros-args \
  -p serial_port:=/dev/serial/by-id/usb-Arduino_UNO_WiFi_R4_CMSIS-DAP_F412FA654890-if01 \
  -p torque_axis:="'y'" \
  -p max_input:=6.0
```

The `"'y'"` quoting is required. YAML reads bare `y` as the boolean `true`, which fails the string type check. `x` and `z` do not have this problem.

---

## 4. The params file

`config/bridge_real.yaml` on the host, mounted read-only at `/config/` inside the container. Edit it in VS Code, no rebuild needed — just restart the node.

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
| `torque_axis` | which wrench component to render: `"x"`, `"y"`, `"z"` | confirm empirically by pushing and watching the echo |
| `max_input` | interaction torque (Nm) that maps to full pressure | set to the peak seen during a real push (~6 Nm at the valve stop) |
| `max_pressure` | kPa at full scale | 60 is the firmware clamp |
| `baseline_alpha` | how fast the gravity baseline adapts; closer to 1 = slower | lower it if idle output drifts above 0 |
| `engage_threshold` | Nm above baseline that counts as contact | raise if noise triggers phantom contact |
| `release_threshold` | Nm below which contact ends; must be < engage | the gap between the two is the hysteresis band |
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
Shows the `State:` field too — `IDLE`, `INFLATE`, `DEFLATE`, `HOLD`, `PURGE`.

---

## 6. Calibration procedure (real mode)

1. Start the bridge, do not touch the arm. `target_kpa_echo` should read **0.00**. If it drifts up, lower `baseline_alpha` to 0.99.
2. Move the arm through free space. Still ~0. If it blips on every move, lower `baseline_alpha` further.
3. Push the handle as in the real task. Note the peak on the echo.
4. Set `max_input` to that peak so a full push gives full pressure.
5. Watch one complete 90° turn: the value should rise, follow the stiffness, and **hold** at the stop without fading. If it fades mid-push, contact detection dropped out — raise `release_threshold`.

---

## 7. Flashing the Arduino

`[HOST]` — stop the bridge first, then confirm the port is free:
```bash
sudo lsof /dev/ttyACM0     # must print nothing
```
Then flash from the Arduino IDE (board: **Arduino UNO R4 WiFi**) or:
```bash
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi arduino/pressure_tracker
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:renesas_uno:unor4wifi arduino/pressure_tracker
```

---

## 8. When something breaks

**Arduino goes deaf, reset button does not help.** The USB CDC endpoint has stalled while still enumerated. `auto_recover: true` handles this automatically. Manually, `[HOST]`:
```bash
sudo python3 ~/kinova_haptic_ws/scripts/usb_reset.py
```

**Nothing inflates but telemetry looks fine.** The external DC supply for the pump and valve is not connected. This has caused this exact symptom more than once.

**Log alternates between two values.** Two publishers on the topic — usually a ghost node from a closed terminal:
```bash
ps aux | grep -E "fake_torque|kinova_haptic" | grep -v grep
kill <PID>
```
`ros2 node list` collapses duplicate node names, so it will *not* show you the duplicate. Use `ros2 topic info <topic> --verbose` and check the publisher count.

**Arrow keys do nothing in `fake_torque_pub`.** Nested TTYs (tmux inside VS Code inside `docker exec`) can swallow escape sequences. Test in a plain SSH terminal.

**"No executable found".** You did not `colcon build`, or did not `source install/setup.bash`.

**Nodes on the same machine cannot see each other.** The CycloneDDS peer list needs `localhost` in it, otherwise disabling multicast also disables same-machine discovery.

---

## 9. Daily commit

`[HOST]`:
```bash
cd ~/kinova_haptic_ws
git add -A
git commit -m "describe what changed"
git push
```

Tag a working milestone so you can always come back to it:
```bash
git tag -a v0.4-real-tested -m "Gated tare + USB auto-recovery, tested on robot"
git push --tags
```