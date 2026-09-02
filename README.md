# χ-Haptics — Kinova Haptic Teleoperation Bridge

Wearable pneumatic haptic interface that renders end-effector torque from a
Kinova robotic arm as asymmetric skin-stretch on the forearm.

A ROS 2 Humble node subscribes to a torque signal, maps it to a pressure
setpoint, and streams that setpoint over USB serial to an Arduino UNO R4 WiFi.
The Arduino runs a closed-loop pressure controller driving a mini pump and a
3-way solenoid exhaust valve, with a Reverse Pulse-Frequency Modulated (PFM)
deflation strategy that approximates proportional control using binary valves.

---

## Table of Contents

1. [System Architecture](#1-system-architecture)
2. [Hardware & Circuit](#2-hardware--circuit)
3. [Arduino Firmware](#3-arduino-firmware)
4. [Raspberry Pi Setup](#4-raspberry-pi-setup)
5. [Docker Environment](#5-docker-environment)
6. [ROS 2 Package](#6-ros-2-package)
7. [Building & Running](#7-building--running)
8. [Connecting to a Real Robot](#8-connecting-to-a-real-robot)
9. [Troubleshooting](#9-troubleshooting)
10. [Measuring Command Latency](#10-measuring-command-latency)

---

## 1. System Architecture

```
┌────────────────────┐
│  Remote PC (LAN)   │  ROS 2 Humble
│  Kinova + F/T      │  publishes torque topic
└─────────┬──────────┘
          │  DDS over Ethernet/WiFi
          │
┌─────────▼──────────────────────────────┐
│  Raspberry Pi — Ubuntu 24.04 (arm64)   │
│  ┌──────────────────────────────────┐  │
│  │ Docker: kinova_haptic_humble     │  │
│  │   ROS 2 Humble                   │  │
│  │   ├─ fake_torque_pub  (testing)  │  │
│  │   └─ kinova_haptic_bridge        │  │
│  └───────────────┬──────────────────┘  │
└──────────────────┼─────────────────────┘
                   │  USB serial, 115200 baud
                   │
┌──────────────────▼─────────────────────┐
│  Arduino UNO R4 WiFi                   │
│    A0  ← MPX5100DP pressure sensor     │
│    D3  → MOSFET → mini pump            │
│    D5  → MOSFET → 3-way exhaust valve  │
└──────────────────┬─────────────────────┘
                   │  pneumatic line
              ┌────▼─────┐
              │ 2 pads   │  forearm, shared air line
              └──────────┘
```

### Data flow

| Stage | Signal | Range |
|-------|--------|-------|
| Robot F/T sensor | torque | ±5 Nm |
| ROS topic | `Float32` (or `WrenchStamped`) | ±5 Nm |
| Bridge mapping | `abs(τ)/5.0 × 50.0` | 0–50 kPa |
| Serial to Arduino | ASCII, e.g. `"32.50\n"` | 0–60 kPa (clamped) |
| Arduino telemetry | `"Target:32.50, Actual:31.88"` | 20 Hz |

Note the mapping uses `abs()`: both pads share one air line, so the system
renders torque **magnitude**, not direction. Directionality comes from pad
placement on the forearm, not differential pressure.

### ROS topics

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/kinova/sim_torque` | `std_msgs/Float32` | bridge subscribes | torque input |
| `/pressure/actual_kpa` | `std_msgs/Float32` | bridge publishes | measured pressure |
| `/pressure/target_kpa_echo` | `std_msgs/Float32` | bridge publishes | target as confirmed by the Arduino |

`target_kpa_echo` is the ground truth for debugging: it is what the **Arduino**
believes the target is, not what the bridge sent. If they disagree, commands are
not landing.

---

## 2. Hardware & Circuit

### Bill of materials

| Component | Notes |
|-----------|-------|
| Arduino UNO R4 WiFi | native USB CDC; pins 3, 5 are PWM-capable |
| MPX5100DP pressure sensor | 0–100 kPa differential, analog out |
| Mini diaphragm pump | 5–12 V DC |
| 3-way solenoid valve | normally closed |
| 2× MOSFET modules | 3-pin logic input: SIG / VCC / GND |
| External DC supply | for pump and valve — **not** the Arduino's 5 V rail |
| 2× inflatable pads | shared pneumatic line |
| Raspberry Pi 4/5 | Ubuntu 24.04 arm64 |

### Wiring

```
MPX5100DP
  Pin 1 (Vout) ──── A0
  Pin 2 (GND)  ──── GND
  Pin 3 (VS)   ──── 5V
  (low-pressure port vented to atmosphere → gauge pressure)

Pump MOSFET module
  SIG ──── D3
  VCC ──── 5V   (logic supply)
  GND ──── GND  (common with Arduino)
  V+ / V- ──── external DC supply
  OUT ──── pump

Valve MOSFET module
  SIG ──── D5
  VCC ──── 5V
  GND ──── GND  (common with Arduino)
  V+ / V- ──── external DC supply
  OUT ──── valve
```

**Critical points**

- The pump and valve **must** be powered from an external DC supply. The
  Arduino's regulator cannot source their current. Forgetting to connect this
  supply produces a system that looks correct in software and does nothing
  physically — telemetry streams, targets update, and no air moves.
- Arduino GND and the external supply GND must be **common**, or the MOSFET
  gates have no reference and switching is unreliable.
- Add a flyback diode (1N4007 or Schottky) across the solenoid coil if the
  MOSFET module does not include one. Inductive kickback destroys MOSFETs.
- Decouple the sensor supply: 100 nF + 10 µF close to the sensor pins. Pump
  inrush sags the 5 V rail and corrupts ADC readings otherwise.

### Sensor transfer function

The MPX5100DP datasheet gives, at supply `VS`:

```
Vout = VS × (0.009 × P + 0.04)
```

Solving for pressure:

```
P = (Vout/VS − 0.04) / 0.009
```

With the Arduino's ADC reference equal to `VS` (both 5 V), `Vout/VS` is exactly
`adc/1023`, so the firmware reduces to:

```cpp
float convertADCToKPa(int adcValue) {
  return ((adcValue / 1023.0) - 0.04) / 0.009;
}
```

This holds **only** while the sensor and the ADC reference share a rail. On a
3.3 V board, level-shift the sensor output and recompute.

---

## 3. Arduino Firmware

Located in `arduino/pressure_tracker/pressure_tracker.ino`.

### Serial protocol

| Direction | Format | Example |
|-----------|--------|---------|
| Host → Arduino | `<target_kPa>\n` | `32.50\n` |
| Host → Arduino | `<target>,<T_max>,<T_min>\n` | `32.50,164,95\n` |
| Arduino → Host | `Target:<t>, Actual:<a>\r\n` @ 20 Hz | `Target:32.50, Actual:31.88` |

The 3-value form overrides the PFM settle bounds at runtime — used by the
optimisation harness. The parser accepts only digits, `.` and `,`; anything
else is discarded.

### Startup tare

On reset the firmware opens the exhaust valve, waits 1500 ms, then averages 50
ADC samples over 500 ms to establish a zero offset. **Total ≈ 2 seconds during
which all serial input is ignored.**

Opening the serial port asserts DTR, which resets the R4. Any host software
must therefore wait ≥3 s after opening the port before sending commands. The
bridge does this.

### Control states

| Condition | Behaviour |
|-----------|-----------|
| `target ≤ 0.1` and `actual < 2.0` | valve held open — full purge |
| `error > +1.5 kPa` | pump on, valve closed — continuous fill |
| `error < −1.5 kPa` | pump off — Reverse-PFM deflation |
| otherwise | both off — hold |

The 1.5 kPa deadband prevents limit-cycle chatter around the setpoint.

### Reverse-PFM deflation

Binary solenoids cannot throttle. To approximate proportional exhaust, the
valve is pulsed with a **fixed** minimum actuation time and a **variable**
interval:

- `T_on = 6 ms` — the shortest pulse that reliably lifts the armature. Shorter
  pulses fail to actuate at all.
- `T_off` — linearly interpolated against current pressure:

```
T_off(P) = map(P, 10 kPa, 50 kPa, 95 ms, 164 ms)
```

Longer waits at high pressure, shorter at low. This is deliberate: flow through
the orifice scales with the pressure differential, so a fixed 6 ms pulse vents
more air at 50 kPa than at 10 kPa. Compensating with the interval holds dP/dt
roughly constant, producing a near-linear descent instead of the exponential
decay a fixed interval would give.

The bounds 164/95 ms were found by hardware-in-the-loop parameter search
against the MSE of the descent versus an ideal linear ramp mirroring the
measured ~2.5 s inflation transient.

### Non-blocking implementation

The PFM state machine is driven by `millis()`, not `delay()`. This matters: an
earlier blocking version froze the loop for up to 170 ms per burst, during
which serial input was not parsed and telemetry stalled. Because the R4 uses
native USB CDC, a stalled loop combined with a host that is not draining the
stream causes `Serial.print()` to block, which hangs the controller entirely.

**Consequence for any host software: you must continuously read the serial
stream.** A write-only client will hang the Arduino once the CDC buffer fills.

### Flashing

Stop anything holding the port first:

```bash
sudo lsof /dev/ttyACM0        # must print nothing
```

Then flash via the Arduino IDE (board: *Arduino UNO R4 WiFi*) or:

```bash
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi arduino/pressure_tracker
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:renesas_uno:unor4wifi arduino/pressure_tracker
```

---

## 4. Raspberry Pi Setup

Ubuntu 24.04 (Noble), arm64. ROS 2 is **not** installed on the host — it lives
entirely in Docker.

### SSH access

```bash
sudo apt install openssh-server -y
sudo systemctl enable ssh --now
```

From a client:

```bash
ssh <user>@<pi-ip>
```

Optionally add to `~/.ssh/config` on the client for a friendly alias:

```
Host pi-haptics
  HostName 10.169.156.98
  User haptic
```

VS Code's **Remote - SSH** extension uses the same config and gives a full
editor running against the Pi's filesystem.

### Docker

```bash
sudo apt install docker.io docker-compose-plugin -y
sudo usermod -aG docker $USER
newgrp docker
docker compose version
```

### Identify the Arduino

```bash
ls -l /dev/serial/by-id/
```

Expect something like:

```
usb-Arduino_UNO_WiFi_R4_CMSIS-DAP_XXXXXXXX-if01 -> ../../ttyACM0
```

The `by-id` path is stable across reboots; `/dev/ttyACM0` is not guaranteed to
be. If the device enumerates as `ttyACM1`, pass it explicitly (see §7).

---

## 5. Docker Environment

### `docker/Dockerfile`

```dockerfile
FROM arm64v8/ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /root/.bashrc

CMD ["bash"]
```

**Humble, not Jazzy** — the distro must match the machine publishing the robot
topics. Cross-distro DDS usually works for standard message types but is not
guaranteed, and breaks on custom messages.

### `docker/docker-compose.yml`

```yaml
services:
  kinova_haptic_humble:
    build:
      context: .
      dockerfile: Dockerfile
    container_name: kinova_haptic_humble
    network_mode: host
    privileged: true
    volumes:
      - ../src:/ros2_ws/src
      - /dev:/dev
    stdin_open: true
    tty: true
```

| Setting | Why |
|---------|-----|
| `network_mode: host` | DDS discovery needs the real network interface; Docker's bridge NAT breaks multicast |
| `privileged` + `/dev:/dev` | access to `/dev/ttyACM0` |
| `../src:/ros2_ws/src` | source lives on the host — edit in VS Code, no rebuild needed |
| `stdin_open` + `tty` | keeps the container alive and allows interactive shells |

---

## 6. ROS 2 Package

```
src/kinova_haptic_teleop/
├── package.xml                          # deps: rclpy, std_msgs, python3-serial
├── setup.py                             # entry_points → executables
├── resource/kinova_haptic_teleop        # ament index marker (empty file)
└── kinova_haptic_teleop/
    ├── __init__.py
    ├── fake_torque_pub.py
    └── kinova_haptic_bridge.py
```

`ament_python` build type. The `entry_points` block in `setup.py` is what makes
`ros2 run` work:

```python
entry_points={
    'console_scripts': [
        'fake_torque_pub = kinova_haptic_teleop.fake_torque_pub:main',
        'kinova_haptic_bridge = kinova_haptic_teleop.kinova_haptic_bridge:main',
    ],
},
```

Omitting these produces "No executable found" even when the package builds.

### `fake_torque_pub`

Keyboard-driven torque simulator for bench testing without the robot.

- Publishes `Float32` on `/kinova/sim_torque` at 10 Hz
- UP / DOWN arrows adjust by ±0.25 Nm, clamped to ±5.0 Nm
- `q` or Ctrl+C exits

Reads the terminal via `os.read()` on the raw file descriptor rather than
`sys.stdin.read()`. This is deliberate: `sys.stdin` is a buffered
`TextIOWrapper`, so a single-character read pulls a whole chunk into Python's
internal buffer, after which `select()` reports no data pending. Arrow keys are
3-byte escape sequences (`ESC [ A`) and get desynchronised, so keypresses are
silently lost.

### `kinova_haptic_bridge`

- Subscribes `/kinova/sim_torque` (`Float32`)
- Maps `min(abs(τ)/5.0 × 50.0, 50.0)` → kPa
- Writes `"<value>\n"` over serial **only when the value changes**
- Runs a reader thread that continuously drains Arduino telemetry and
  republishes it as `/pressure/actual_kpa` and `/pressure/target_kpa_echo`

The reader thread is **mandatory**, not a convenience. See §3.

Deduplicating writes reduces serial traffic from 10 Hz of identical commands to
only genuine changes, and makes a stray second publisher immediately visible —
the log alternates between two values instead of looking like normal operation.

Parameters:

| Parameter | Default |
|-----------|---------|
| `serial_port` | `/dev/ttyACM0` |
| `baud_rate` | `115200` |

---

## 7. Building & Running

### First-time build

```bash
cd ~/kinova_haptic_ws/docker
docker compose up -d --build
docker exec -it kinova_haptic_humble bash
```

Inside the container:

```bash
cd /ros2_ws
colcon build
source install/setup.bash
ros2 pkg executables kinova_haptic_teleop
```

Expected:

```
kinova_haptic_teleop fake_torque_pub
kinova_haptic_teleop kinova_haptic_bridge
```

Rebuild after editing Python sources (`colcon build` again). The bind-mount
means files sync instantly, but `ros2 run` executes the installed copy.

### Normal operation

**Terminal 1 — bridge**

```bash
docker exec -it kinova_haptic_humble bash
source install/setup.bash
ros2 run kinova_haptic_teleop kinova_haptic_bridge
```

With a non-default port:

```bash
ros2 run kinova_haptic_teleop kinova_haptic_bridge --ros-args -p serial_port:=/dev/ttyACM1
```

**Terminal 2 — torque input**

```bash
docker exec -it kinova_haptic_humble bash
source install/setup.bash
ros2 run kinova_haptic_teleop fake_torque_pub
```

**Terminal 3 — monitoring (optional)**

```bash
docker exec -it kinova_haptic_humble bash
source install/setup.bash
ros2 topic echo /pressure/actual_kpa
```

### Testing without the keyboard node

```bash
ros2 topic pub /kinova/sim_torque std_msgs/msg/Float32 "{data: 4.0}" -r 10   # → 40 kPa
# Ctrl+C, then:
ros2 topic pub /kinova/sim_torque std_msgs/msg/Float32 "{data: 1.0}" -r 10   # → 10 kPa, PFM descent
```

Test deflation between two **non-zero** targets. A target of 0 triggers the
full-purge branch, not PFM.

### Direct serial test (bypasses ROS entirely)

Useful for isolating hardware from software faults:

```bash
python3 -c "import serial,time; s=serial.Serial('/dev/ttyACM0',115200,timeout=2); time.sleep(3); s.write(b'25.00\n'); [print(s.readline()) for _ in range(20)]"
```

`Target:` should change to `25.00` and pressure should rise. The 3 s sleep
covers the reset-and-tare window.

---

## 8. Connecting to a Real Robot

Two changes: **network discovery** and **message type**.

### 8.1 Network discovery

Both machines must share a `ROS_DOMAIN_ID` (default `0`) and be able to
discover each other. On a plain home LAN with multicast enabled, `network_mode:
host` is sufficient and nothing further is needed.

Many institutional networks block multicast. In that case configure CycloneDDS
for **unicast peer discovery**. Create `config/cyclonedds.xml`:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
    <Domain id="any">
        <General>
            <AllowMulticast>false</AllowMulticast>
        </General>
        <Discovery>
            <Peers>
                <Peer address="10.186.85.98"/>   <!-- robot PC -->
                <Peer address="10.186.85.30"/>   <!-- this Pi -->
            </Peers>
        </Discovery>
    </Domain>
</CycloneDDS>
```

Replace the addresses with the actual IPs. List **every** participating machine,
including this one.

Mount it and point the runtime at it in `docker-compose.yml`:

```yaml
services:
  kinova_haptic_humble:
    # ... existing settings ...
    volumes:
      - ../src:/ros2_ws/src
      - ../config:/config:ro
      - /dev:/dev
    environment:
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      - CYCLONEDDS_URI=file:///config/cyclonedds.xml
      - ROS_DOMAIN_ID=0
```

Recreate the container:

```bash
cd ~/kinova_haptic_ws/docker
docker compose up -d --force-recreate
```

Verify discovery — with the robot publishing:

```bash
docker exec -it kinova_haptic_humble bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic hz <topic_name>
```

If topics appear but `hz` reports nothing, discovery succeeded and data flow
did not — check firewall rules on UDP 7400–7500.

### 8.2 Message type

`fake_torque_pub` publishes `std_msgs/Float32`. A real F/T sensor almost
certainly publishes `geometry_msgs/WrenchStamped`.

Confirm before editing:

```bash
ros2 topic info /<their_topic> --verbose
ros2 interface show geometry_msgs/msg/WrenchStamped
ros2 topic echo /<their_topic> --once
```

Then in `kinova_haptic_bridge.py`:

```python
# Replace the Float32 import
from geometry_msgs.msg import WrenchStamped

# Replace the subscription
self.create_subscription(
    WrenchStamped,
    '/your/actual/topic',        # ← their topic name
    self.on_torque_received,
    10)

# Replace the callback signature and first line
def on_torque_received(self, msg: WrenchStamped):
    tau = msg.wrench.torque.z     # ← choose the relevant axis
    pressure = min(abs(tau) / self.max_torque * self.max_pressure,
                   self.max_pressure)
    # ... rest unchanged
```

Add the dependency to `package.xml`:

```xml
<exec_depend>geometry_msgs</exec_depend>
```

Then `colcon build` and rerun.

**Choosing the axis.** `msg.wrench.torque` has x, y and z. Which one corresponds
to the motion you want to render depends on the sensor's mounting frame. Echo
the topic while moving the arm along each axis and watch which component
responds.

**Scaling.** `max_torque = 5.0` assumes ±5 Nm saturates the display. Measure the
real signal's range first — if the sensor typically reads ±0.5 Nm, the mapping
produces almost no pressure and the interface feels dead. Adjust `max_torque` to
the actual working range.

**QoS.** Sensor streams are frequently published with `BEST_EFFORT` reliability.
The default subscription is `RELIABLE`, and a reliable subscriber will **not**
match a best-effort publisher — the topic appears in `ros2 topic list` but no
callback ever fires. Check with `ros2 topic info <topic> --verbose` and if
needed:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10)

self.create_subscription(WrenchStamped, '/your/topic',
                         self.on_torque_received, qos)
```

**Safety.** A real sensor can produce spikes and noise. The firmware clamps to
`MAX_PRESSURE = 60.0` and the bridge clamps to 50 kPa, but consider adding a
rate limit or low-pass filter in the callback before deploying on a person.

---

## 9. Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Telemetry streams, targets update, nothing inflates | External DC supply not connected | Connect pump/valve supply |
| Arduino stops responding after a few seconds | Host not draining serial; CDC buffer full | Ensure the reader thread is running |
| Pressure only drops when the bridge is stopped | Closing the port resets the R4 → `target = 0` → purge. Board was not receiving updates | Above |
| Log alternates between two values | Two publishers on the topic | `ps aux \| grep fake_torque` inside the container; kill the stale PID |
| `ros2 node list` shows one node but behaviour suggests two | Identical node names are deduplicated in that output | Use `ros2 topic info <topic> --verbose` and check publisher count |
| Arrow keys do nothing | Nested TTYs (tmux inside VS Code inside `docker exec`) intercepting escape sequences | Test in a plain SSH terminal |
| "No executable found" | Missing `entry_points`, or `install/setup.bash` not sourced | Check `setup.py`; re-source |
| Serial port busy / upload fails | A node still holds the port | `sudo lsof /dev/ttyACM0`, kill the PID |
| First commands after startup ignored | 2 s reset-and-tare window | Wait ≥3 s after opening the port |
| Topics visible but no callbacks | QoS mismatch | See §8.2 |

### Ghost processes

A `docker exec -it` session that loses its terminal leaves the process running
detached. Because `ros2 node list` collapses duplicate node names, a stale node
is invisible there while actively publishing. When behaviour seems impossible:

```bash
docker exec kinova_haptic_humble bash -c "ps aux | grep ros2 | grep -v grep"
```

### Useful commands

```bash
# Who holds the serial port (host)
sudo lsof /dev/ttyACM0

# Confirm stable device path
ls -l /dev/serial/by-id/

# Publisher/subscriber counts
ros2 topic info /kinova/sim_torque --verbose

# What the Arduino thinks the target is
ros2 topic echo /pressure/target_kpa_echo

# Container logs
docker logs kinova_haptic_humble
```

---

## 10. Measuring Command Latency

How long is it from a torque reading landing on the Pi to the pump actually
switching on? `test/latency_probe.py` answers that, broken down per step.

It is **standalone**. It does not import, modify or wrap `kinova_haptic_bridge.py`,
and it needs no `colcon build` - `setup.py` excludes `test/` from the install, so
you run it by path. The only thing it shares with the bridge is `screw_torque.py`,
imported read-only so the mapping under test cannot drift from production.

### What it measures

| # | Stage | Meaning |
|---|-------|---------|
| 1 | `sensor_to_host` | FT `header.stamp` → wrench callback entry |
| 2 | `compute` | screw-torque + pad mapping arithmetic |
| 3 | `queue_wait` | wrench received → serial write begins **(the dominant term)** |
| 4 | `serial_write` | `ser.write()` + `flush()` duration |
| 5 | `host_total` | wrench received → command bytes flushed (3 + 4) |
| 6 | `link_ack_rtt` | bytes flushed → Arduino acknowledged the command |
| 7 | `link_oneway_est` | estimated one-way host→MCU (half of 6) |
| 8 | `cmd_to_inflate` | bytes flushed → Arduino entered `INFLATE` |
| 9 | `end_to_end` | wrench received → `INFLATE` (5 + 8) |
| 10 | `pneumatic_dead` | `INFLATE` → first measurable pressure rise |

Stages 1-5 are exact: both endpoints are timestamps on the probe's own monotonic
clock. Stage 10 is the pump, tubing and pad volume - it is reported but
deliberately **excluded** from the latency budget, because it is the actuator
doing its job rather than a control-path delay.

Stage 3 is the one worth looking at. `push_to_arduino` runs on its own 20 Hz
timer, unsynchronised with `on_wrench`, so a fresh reading waits a uniform
0-50 ms before it reaches the wire.

### Modes

**`--mode stimulus`** — no ROS, no robot, no wrench topic. Plain `python3` +
`pyserial`. Drives a square wave and measures the link, the MCU and the
pneumatic dead time in isolation. The cleanest, most repeatable answer to
"how long from command to the start of the action". **Stop the bridge first.**

```bash
python3 src/kinova_haptic_teleop/test/latency_probe.py --mode stimulus \
    --serial-port /dev/ttyACM0 --stim-high 30 --stim-period 4
```

**`--mode live`** (default) — an instrumented twin of the bridge: same 20 Hz
timer, same 2-decimal dedup, same serial framing, driving the real hardware
from the real wrench topic. Full chain under real conditions.
**Stop the bridge first** - two processes cannot own the serial port.

```bash
docker exec -it kinova_haptic_humble bash
source install/setup.bash
python3 /repo/src/kinova_haptic_teleop/test/latency_probe.py --mode live \
    --params /config/bridge_real.yaml
```

You must actually apply torque during the run. If nothing moves, `pA` never
changes at 2 decimals, the dedup suppresses every write, and there is nothing
to measure - the report will say so.

**`--mode passive`** — opens no serial port, publishes nothing, and observes a
**running** bridge over its own topics. Safe mid-experiment, but it only sees
stage 1 and a coarse end-to-end round trip; everything else happens inside the
bridge process where it cannot reach.

```bash
python3 /repo/src/kinova_haptic_teleop/test/latency_probe.py --mode passive
```

### Output

Stop with **Ctrl+C**. The probe prints a per-stage table (n / mean / p50 / p95 /
max / min, in milliseconds), a "where the time goes" budget with percentages,
and interpretation notes. Two files land in `--out-dir` (default `/repo/logs`):

- `latency_<mode>_<timestamp>_events.csv` — one row per command, every stage in ms
- `latency_<mode>_<timestamp>_report.txt` — a copy of the printed report

Numbers backed by fewer than 20 samples are printed but flagged rather than
interpreted.

### Firmware precision

With the **stock** `pressure_tracker.ino` the Arduino only speaks every
`LOG_INTERVAL_MS` (50 ms), so stages 6-9 are quantised to 50 ms and biased high
by ~25 ms on average. Usable as an upper bound, not a precise figure.

For millisecond accuracy, flash the optional
`arduino/pressure_tracker_latency/` build. It is a generated copy of the stock
sketch whose only changes are two immediate event lines - `EVT:ACK` when a
command parses, `EVT:INFLATE` on entry into `INFLATE` - plus the `prev_state`
variable that detects the transition. No control law, timing constant or safety
limit is altered. The probe auto-detects it; there is no flag to pass.

Reflash the stock sketch for real runs. The `EVT:` lines are harmless to the
bridge (`parse_telemetry()` rejects them), but keep one firmware in service.

### Caveats

- `link_oneway_est` halves the round trip, which assumes a symmetric USB CDC
  path. Good to a millisecond or two. For ground truth, toggle a spare Pi GPIO
  immediately before `ser.write` and probe it against Arduino pin 3 on a
  two-channel scope.
- `sensor_to_host` compares a ROS stamp against this host's wall clock. It is
  only meaningful if the FT publisher runs on this Pi, or if the clocks are
  disciplined (PTP/chrony). The probe warns when the difference goes negative.

---

## Repository Layout

```
kinova_haptic_ws/
├── arduino/
│   ├── pressure_tracker/
│   │   └── pressure_tracker.ino
│   └── pressure_tracker_latency/          # optional, measurement only
│       └── pressure_tracker_latency.ino
├── config/
│   └── cyclonedds.xml
├── docker/
│   ├── Dockerfile
│   └── docker-compose.yml
├── src/
│   └── kinova_haptic_teleop/
│       ├── package.xml
│       ├── setup.py
│       ├── resource/kinova_haptic_teleop
│       ├── kinova_haptic_teleop/
│       │   ├── __init__.py
│       │   ├── fake_torque_pub.py
│       │   └── kinova_haptic_bridge.py
│       └── test/
│           ├── test_screw_torque.py
│           └── latency_probe.py           # standalone, not installed
├── .gitignore
└── README.md
```

## License

MIT