/*
 * ===========================================================================
 *  LATENCY-INSTRUMENTED BUILD - MEASUREMENT ONLY, NOT FOR PRODUCTION
 * ===========================================================================
 *  Generated copy of ../pressure_tracker/pressure_tracker.ino. That file is
 *  the authority; this one exists solely so latency_probe.py can see exactly
 *  when the MCU acts, instead of guessing from the 50 ms telemetry tick.
 *
 *  The ONLY differences from the stock sketch, in full:
 *
 *    1. EVT:ACK,<target>,<millis>      emitted the instant a command parses
 *    2. EVT:INFLATE,<target>,<millis>  emitted on entry into INFLATE
 *    3. prev_state, to detect that entry as a transition
 *
 *  No control law, no timing constant, no safety limit is altered. The pump
 *  and valve behave identically. The extra output is two short lines per
 *  command change; the 20 Hz command rate leaves the CDC endpoint idle
 *  between them.
 *
 *  These lines are harmless to the production bridge (its parse_telemetry()
 *  rejects them and moves on), but reflash the stock sketch for real runs
 *  anyway - keep one firmware in service, not two.
 *
 *  Re-generate rather than hand-edit if the stock sketch ever changes.
 * ===========================================================================
 *
 * PURE DIGITAL PRESSURE TRACKER - NON-BLOCKING REVERSE-PFM
 * RL-optimised bounds: 6 ms burst, T_off 95 ms (low P) to 164 ms (high P).
 *
 * Safety features:
 *   - Command watchdog: no serial command -> vent, then idle.
 *   - Time-limited purge: the valve is never held open indefinitely.
 *   - Valve thermal cap: hard limit on continuous energised time.
 *   - Serial line timeout: a half-received command cannot wedge the parser.
 */

const int SENSOR_PIN = A0;
const int PUMP_PIN   = 3;
const int VALVE_PIN  = 5;

const float MAX_PRESSURE = 60.0;
const float DEADBAND_KPA = 1.5;

// ---- Reverse-PFM parameters ----
const unsigned long BURST_MS = 6;     // minimum mechanical actuation time
const int P_LOW  = 10;                // kPa, lower bound of the T_off map
const int P_HIGH = 50;                // kPa, upper bound of the T_off map
int settle_min = 40;                  // T_off at P_LOW
int settle_max = 80;                 // T_off at P_HIGH

// ---- Purge (full vent) ----
const unsigned long PURGE_MAX_MS   = 3000;  // never hold the valve open longer
const float PURGE_DONE_KPA         = 1.0;   // below this, consider vented
const float RE_PURGE_KPA           = 5.0;   // repressurised -> allow another purge

// ---- Valve thermal protection (hard safety net) ----
const unsigned long VALVE_MAX_ON_MS    = 5000;   // max continuous energised time
const unsigned long VALVE_COOLDOWN_MS  = 10000;  // forced off period after that

// ---- Command watchdog ----
const unsigned long COMMAND_TIMEOUT_MS = 15000;
unsigned long lastCommandTime = 0;
bool watchdog_tripped = false;

// ---- Serial line timeout ----
const unsigned long SERIAL_LINE_TIMEOUT_MS = 500;
unsigned long lastSerialByteTime = 0;

float target_kPa = 0.0;
float sensor_offset_kPa = 0.0;

// ---- PFM state ----
bool valve_open = false;
unsigned long pfm_timer = 0;
unsigned long current_settle = 0;

// ---- Purge state ----
bool purging = false;
bool purge_done = false;
unsigned long purge_start = 0;

// ---- Valve on-time tracking ----
bool valve_energised = false;
unsigned long valve_on_since = 0;
bool valve_cooling = false;
unsigned long cooldown_start = 0;

unsigned long lastLogTime = 0;
const int LOG_INTERVAL_MS = 50;

char serialBuffer[32];
int bufferIndex = 0;
const char* state_name = "INIT";
const char* prev_state = "INIT";   // [latency build] INFLATE-entry detection

void calibrateSensorOffset();
float convertADCToKPa(int adcValue);
void setValve(bool on, unsigned long now);
void setPump(bool on);
void emitAck(unsigned long now);   // [latency build]

void setup() {
  Serial.begin(115200);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(VALVE_PIN, LOW);
  calibrateSensorOffset();
  lastCommandTime = millis();
  lastSerialByteTime = millis();
}

void loop() {
  unsigned long now = millis();

  // ---- 1. SERIAL PARSING (non-blocking) ----
  while (Serial.available() > 0) {
    char c = Serial.read();
    lastSerialByteTime = now;
    if (c == '\n' || c == '\r') {
      if (bufferIndex > 0) {
        serialBuffer[bufferIndex] = '\0';
        int commaCount = 0;
        for (int i = 0; i < bufferIndex; i++) {
          if (serialBuffer[i] == ',') commaCount++;
        }
        if (commaCount == 2) {
          char* t_str   = strtok(serialBuffer, ",");
          char* max_str = strtok(NULL, ",");
          char* min_str = strtok(NULL, ",");
          if (t_str && max_str && min_str) {
            target_kPa = constrain(atof(t_str), 0.0, MAX_PRESSURE);
            settle_max = constrain(atoi(max_str), 50, 300);
            settle_min = constrain(atoi(min_str), 10, 150);
            lastCommandTime = now;
            watchdog_tripped = false;
            emitAck(now);            // [latency build]
          }
        } else {
          target_kPa = constrain(atof(serialBuffer), 0.0, MAX_PRESSURE);
          lastCommandTime = now;
          watchdog_tripped = false;
          emitAck(now);              // [latency build]
        }
        bufferIndex = 0;
      }
    } else if ((c >= '0' && c <= '9') || c == '.' || c == ',') {
      if (bufferIndex < 31) {
        serialBuffer[bufferIndex++] = c;
      }
    }
  }

  // Discard a partial line left behind by a killed host process.
  if (bufferIndex > 0 && (now - lastSerialByteTime) > SERIAL_LINE_TIMEOUT_MS) {
    bufferIndex = 0;
  }

  // ---- 2. WATCHDOG ----
  if (now - lastCommandTime > COMMAND_TIMEOUT_MS) {
    if (!watchdog_tripped) {
      Serial.println("WATCHDOG:timeout, venting");
      watchdog_tripped = true;
      purge_done = false;      // allow one purge to vent safely
      purging = false;
    }
    target_kPa = 0.0;
  }

  // ---- 3. READ & TARE ----
  float actual_kPa = convertADCToKPa(analogRead(SENSOR_PIN)) - sensor_offset_kPa;
  if (actual_kPa < 0.0) actual_kPa = 0.0;
  float error = target_kPa - actual_kPa;

  // ---- 4. CONTROL ----
  if (target_kPa <= 0.1) {
    // Commanded to zero: vent once, then sit idle with nothing energised.
    setPump(false);
    valve_open = false;

    if (actual_kPa > RE_PURGE_KPA) purge_done = false;  // repressurised

    if (!purge_done && actual_kPa > PURGE_DONE_KPA) {
      if (!purging) { purging = true; purge_start = now; }
      if (now - purge_start < PURGE_MAX_MS) {
        setValve(true, now);
        state_name = "PURGE";
      } else {
        purging = false;
        purge_done = true;
        setValve(false, now);
        state_name = "IDLE";
      }
    } else {
      purging = false;
      purge_done = true;
      setValve(false, now);        // IDLE: coil de-energised, valve closed
      state_name = "IDLE";
    }
  }
  else {
    purging = false;
    purge_done = false;

    if (error > DEADBAND_KPA) {
      // INFLATE
      setValve(false, now);
      setPump(true);
      valve_open = false;
      pfm_timer = now;
      state_name = "INFLATE";
    }
    else if (error < -DEADBAND_KPA) {
      // DEFLATE: reverse-PFM (low duty cycle, thermally safe)
      setPump(false);
      state_name = "DEFLATE";

      if (valve_open) {
        if (now - pfm_timer >= BURST_MS) {
          setValve(false, now);
          valve_open = false;
          pfm_timer = now;
          current_settle = constrain(
              map((int)actual_kPa, P_LOW, P_HIGH, settle_min, settle_max),
              settle_min, settle_max);
        }
      } else {
        if (now - pfm_timer >= current_settle) {
          setValve(true, now);
          valve_open = true;
          pfm_timer = now;
        }
      }
    }
    else {
      // HOLD
      setPump(false);
      setValve(false, now);
      valve_open = false;
      pfm_timer = now;
      state_name = "HOLD";
    }
  }

  // ---- 4b. [latency build] STATE-TRANSITION EVENT ----
  // Emitted immediately, NOT on the LOG_INTERVAL_MS tick: the whole point is
  // to timestamp the pump turning on without 50 ms of sampling delay folded in.
  if (state_name != prev_state) {
    if (strcmp(state_name, "INFLATE") == 0) {
      Serial.print("EVT:INFLATE,");
      Serial.print(target_kPa);
      Serial.print(",");
      Serial.println(now);
    }
    prev_state = state_name;
  }

  // ---- 5. TELEMETRY ----
  if (now - lastLogTime >= LOG_INTERVAL_MS) {
    Serial.print("Target:");
    Serial.print(target_kPa);
    Serial.print(", Actual:");
    Serial.print(actual_kPa);
    Serial.print(", State:");
    Serial.println(state_name);
    lastLogTime = now;
  }
}

// Valve driver with a hard cap on continuous energised time.
// Even if the control logic misbehaves, the coil cannot be held on
// longer than VALVE_MAX_ON_MS.
void setValve(bool on, unsigned long now) {
  if (valve_cooling) {
    if (now - cooldown_start < VALVE_COOLDOWN_MS) {
      digitalWrite(VALVE_PIN, LOW);
      valve_energised = false;
      return;
    }
    valve_cooling = false;
  }

  if (on) {
    if (!valve_energised) {
      valve_energised = true;
      valve_on_since = now;
    } else if (now - valve_on_since > VALVE_MAX_ON_MS) {
      Serial.println("VALVE:thermal cap, forcing cooldown");
      valve_cooling = true;
      cooldown_start = now;
      digitalWrite(VALVE_PIN, LOW);
      valve_energised = false;
      return;
    }
    digitalWrite(VALVE_PIN, HIGH);
  } else {
    digitalWrite(VALVE_PIN, LOW);
    valve_energised = false;
  }
}

// [latency build] Acknowledge a parsed command the moment it is parsed, so the
// host can time the link and the MCU separately from the pneumatics.
void emitAck(unsigned long now) {
  Serial.print("EVT:ACK,");
  Serial.print(target_kPa);
  Serial.print(",");
  Serial.println(now);
}

void setPump(bool on) {
  digitalWrite(PUMP_PIN, on ? HIGH : LOW);
}

void calibrateSensorOffset() {
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(VALVE_PIN, HIGH);
  delay(1500);
  float running_sum = 0.0;
  for (int i = 0; i < 50; i++) {
    running_sum += convertADCToKPa(analogRead(SENSOR_PIN));
    delay(10);
  }
  sensor_offset_kPa = running_sum / 50.0;
  digitalWrite(VALVE_PIN, LOW);
}

float convertADCToKPa(int adcValue) {
  return ((adcValue / 1023.0) - 0.04) / 0.009;
}