/*
 * PAD CHARACTERISATION RIG
 *
 * Standalone test firmware for characterising a new pneumatic pad.
 * Not for wearing during phase 1 - find the safe limit with the pad
 * behind a shield first.
 *
 * Serial protocol, line terminated, 115200 baud.
 *
 *   U                       step target up   by STEP_KPA
 *   D                       step target down by STEP_KPA
 *   Z                       vent to zero, leave any test mode
 *   R                       reset the max-pressure recorder
 *   S                       report status
 *   I,<target>              inflation test: vent, settle, full pump to target
 *   F,<target>,<Tmax>,<Tmin>  deflation test: inflate to target, settle,
 *                             then reverse-PFM descent to DEFLATE_END_KPA
 *
 * Output lines:
 *   D,<ms>,<target>,<actual>,<mode>     telemetry
 *   EVT,<text>                          events
 *   MAX,<kPa>                           max pressure seen since reset
 *   RESULT,INFLATE,<rise_ms>,<final>
 *   RESULT,DEFLATE,<fall_ms>,<final>
 */

const int SENSOR_PIN = A0;
const int PUMP_PIN   = 3;
const int VALVE_PIN  = 5;

// ---- Safety ----
const float HARD_CEILING_KPA = 80.0;   // absolute refusal, never exceeded
const float STEP_KPA         = 2.0;    // manual increment
const float DEADBAND_KPA     = 1.5;

// ---- Reverse-PFM ----
const unsigned long BURST_MS = 6;      // minimum mechanical actuation time
int settle_min = 95;                   // T_off at the low anchor
int settle_max = 164;                  // T_off at the high anchor
const float DEFLATE_END_KPA = 2.0;    // descent terminates here
float map_anchor_high = 50.0;          // set to the test target

// ---- Valve thermal protection ----
const unsigned long VALVE_MAX_ON_MS   = 5000;
const unsigned long VALVE_COOLDOWN_MS = 10000;
bool valve_energised = false;
unsigned long valve_on_since = 0;
bool valve_cooling = false;
unsigned long cooldown_start = 0;
unsigned long valve_close_time = 0;
float settled_kPa = 0.0;
const unsigned long BLANK_MS = 0;   // ignore readings this soon after a burst

// ---- Modes ----
enum Mode { M_MANUAL, M_INFLATE_TEST, M_DEFLATE_TEST };
Mode mode = M_MANUAL;

// test sub-phases
enum Phase { PH_IDLE, PH_VENT, PH_SETTLE, PH_RUN, PH_DONE };
Phase phase = PH_IDLE;

float target_kPa = 0.0;
float sensor_offset_kPa = 0.0;
float max_seen_kPa = 0.0;
float test_target = 0.0;

unsigned long phase_start = 0;
unsigned long test_start = 0;
bool reached_target = false;
unsigned long settle_start = 0;

// PFM state
bool valve_open = false;
unsigned long pfm_timer = 0;
unsigned long current_settle = 0;

// logging
unsigned long lastLogTime = 0;
int log_interval_ms = 100;             // 10 Hz idle, 20 ms during tests

// serial
char buf[48];
int bufIdx = 0;

float convertADCToKPa(int adcValue) {
  return ((adcValue / 1023.0) - 0.04) / 0.009;
}

void setPump(bool on) { digitalWrite(PUMP_PIN, on ? HIGH : LOW); }

void setValve(bool on, unsigned long now) {
  if (valve_cooling) {
    if (now - cooldown_start < VALVE_COOLDOWN_MS) {
      digitalWrite(VALVE_PIN, LOW); valve_energised = false; return;
    }
    valve_cooling = false;
  }
  if (on) {
    if (!valve_energised) { valve_energised = true; valve_on_since = now; }
    else if (now - valve_on_since > VALVE_MAX_ON_MS) {
      Serial.println("EVT,valve thermal cap - cooldown");
      valve_cooling = true; cooldown_start = now;
      digitalWrite(VALVE_PIN, LOW); valve_energised = false; return;
    }
    digitalWrite(VALVE_PIN, HIGH);
  } else {
    if (valve_energised) valve_close_time = now;
    digitalWrite(VALVE_PIN, LOW); valve_energised = false;
  }
}

void calibrateSensorOffset() {
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(VALVE_PIN, HIGH);
  delay(1500);
  float s = 0.0;
  for (int i = 0; i < 50; i++) { s += convertADCToKPa(analogRead(SENSOR_PIN)); delay(10); }
  sensor_offset_kPa = s / 50.0;
  digitalWrite(VALVE_PIN, LOW);
}

const char* modeName() {
  if (mode == M_INFLATE_TEST) {
    if (phase == PH_VENT)   return "INF_VENT";
    if (phase == PH_SETTLE) return "INF_SETTLE";
    if (phase == PH_RUN)    return "INF_RUN";
    return "INFTEST";
  }
  if (mode == M_DEFLATE_TEST) {
    if (phase == PH_VENT)   return "DEF_VENT";
    if (phase == PH_SETTLE) return "DEF_SETTLE";
    if (phase == PH_RUN)    return "DEF_RUN";
    return "DEFTEST";
  }
  return "MANUAL";
}

void setup() {
  Serial.begin(115200);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(VALVE_PIN, LOW);
  calibrateSensorOffset();
  Serial.println("EVT,ready");
}

void handleCommand(unsigned long now) {
  char c = buf[0];

  if (c == 'U') {
    target_kPa = min(target_kPa + STEP_KPA, HARD_CEILING_KPA);
    mode = M_MANUAL; phase = PH_IDLE;
    Serial.print("EVT,target "); Serial.println(target_kPa);
  }
  else if (c == 'D') {
    target_kPa = max(target_kPa - STEP_KPA, 0.0);
    mode = M_MANUAL; phase = PH_IDLE;
    Serial.print("EVT,target "); Serial.println(target_kPa);
  }
  else if (c == 'Z') {
    target_kPa = 0.0; mode = M_MANUAL; phase = PH_IDLE;
    log_interval_ms = 100;
    Serial.println("EVT,vent");
  }
  else if (c == 'R') {
    max_seen_kPa = 0.0;
    Serial.println("EVT,max reset");
  }
  else if (c == 'S') {
    Serial.print("MAX,"); Serial.println(max_seen_kPa);
  }
  else if (c == 'I') {
    char* t = strtok(buf + 2, ",");
    if (t) {
      test_target = constrain(atof(t), 0.0, HARD_CEILING_KPA);
      mode = M_INFLATE_TEST; phase = PH_VENT;
      phase_start = now; log_interval_ms = 20;
      Serial.print("EVT,inflation test to "); Serial.println(test_target);
    }
  }
  else if (c == 'F') {
    char* t    = strtok(buf + 2, ",");
    char* tmax = strtok(NULL, ",");
    char* tmin = strtok(NULL, ",");
    if (t && tmax && tmin) {
      test_target = constrain(atof(t), 0.0, HARD_CEILING_KPA);
      settle_max  = constrain(atoi(tmax), 10, 600);
      settle_min  = constrain(atoi(tmin), 5, 400);
      map_anchor_high = test_target;
      mode = M_DEFLATE_TEST; phase = PH_VENT;
      reached_target = false;                     
      phase_start = now; log_interval_ms = 20;
      Serial.print("EVT,deflation test "); Serial.print(test_target);
      Serial.print(" Tmax="); Serial.print(settle_max);
      Serial.print(" Tmin="); Serial.println(settle_min);
    }
  }
}

void loop() {
  unsigned long now = millis();

  // ---- serial ----
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufIdx > 0) { buf[bufIdx] = '\0'; handleCommand(now); bufIdx = 0; }
    } else if (bufIdx < 47) {
      buf[bufIdx++] = c;
    }
  }

  // ---- read ----
  float a1 = convertADCToKPa(analogRead(SENSOR_PIN));
  float a2 = convertADCToKPa(analogRead(SENSOR_PIN));
  float a3 = convertADCToKPa(analogRead(SENSOR_PIN));
  float med = max(min(a1,a2), min(max(a1,a2), a3));
  float actual = med - sensor_offset_kPa;
  // Trust the sensor only when the valve has been closed long enough for
  // the line to equalise. Everything downstream uses settled_kPa.
  if (!valve_energised && (now - valve_close_time) >= BLANK_MS) {
    settled_kPa = actual;
  }
  if (actual < 0.0) actual = 0.0;
  if (actual > max_seen_kPa) max_seen_kPa = actual;

  // ---- hard safety: never exceed the ceiling regardless of mode ----
  if (actual > HARD_CEILING_KPA) {
    setPump(false); setValve(true, now);
    target_kPa = 0.0; mode = M_MANUAL; phase = PH_IDLE;
    Serial.println("EVT,CEILING EXCEEDED - emergency vent");
  }

  // ---- test sequencing ----
  if (mode == M_INFLATE_TEST) {
    if (phase == PH_VENT) {
      setPump(false); setValve(true, now);
      if (actual < 1.0 || now - phase_start > 8000) {
        phase = PH_SETTLE; phase_start = now; setValve(false, now);
      }
    }
    else if (phase == PH_SETTLE) {
      setValve(false, now);
      if (!reached_target) {
        if (actual >= test_target - DEADBAND_KPA) {
          reached_target = true;
          setPump(false);
          settle_start = now;
        } else {
          setPump(true);
        }
      } else {
        if (now - settle_start < 1000) {
          setPump(false);                 // let it settle
        } else if (actual < test_target - 0.5) {
          setPump(true);                  // top up any loss just before descent
        } else {
          setPump(false);
          phase = PH_RUN; test_start = now;
          pfm_timer = now; valve_open = false; current_settle = 0;
          Serial.print("EVT,deflate start at "); Serial.println(actual);
        }
      }
      if (now - phase_start > 30000) {
        setPump(false);
        Serial.println("EVT,deflate test aborted - target unreachable");
        mode = M_MANUAL; phase = PH_IDLE; target_kPa = 0.0;
        log_interval_ms = 100;
      }
    }
    else if (phase == PH_RUN) {
      setValve(false, now); setPump(true);
      if (actual >= test_target - DEADBAND_KPA || now - test_start > 30000) {
        setPump(false);
        Serial.print("RESULT,INFLATE,"); Serial.print(now - test_start);
        Serial.print(","); Serial.println(actual);
        phase = PH_DONE; target_kPa = test_target;
        mode = M_MANUAL; log_interval_ms = 100;
      }
    }
  }
  else if (mode == M_DEFLATE_TEST) {
    if (phase == PH_VENT) {
      setPump(false); setValve(true, now);
      if (actual < 1.0 || now - phase_start > 8000) {
        phase = PH_SETTLE; phase_start = now; setValve(false, now);
      }
    }
    else if (phase == PH_SETTLE) {
      setValve(false, now);
      if (!reached_target) {
        if (actual >= test_target - DEADBAND_KPA) {
          reached_target = true;
          setPump(false);
          settle_start = now;
        } else {
          setPump(true);
        }
      } else {
        // Hold with the pump off and let it settle; do NOT re-pump on a
        // small leak, or the settle timer never completes.
        setPump(false);
        if (now - settle_start > 1500) {
          phase = PH_RUN; test_start = now;
          pfm_timer = now; valve_open = false; current_settle = 0;
          Serial.println("EVT,deflate start");
        }
      }
      if (now - phase_start > 30000) {
        setPump(false);
        Serial.println("EVT,deflate test aborted - target unreachable");
        mode = M_MANUAL; phase = PH_IDLE; target_kPa = 0.0;
        log_interval_ms = 100;
      }
    }
    else if (phase == PH_RUN) {
      setPump(false);
      if (settled_kPa <= DEFLATE_END_KPA || now - test_start > 60000) {
        setValve(false, now);
        Serial.print("RESULT,DEFLATE,"); Serial.print(now - test_start);
        Serial.print(","); Serial.println(settled_kPa);
        phase = PH_DONE; target_kPa = 0.0;
        mode = M_MANUAL; log_interval_ms = 100;
      } else {
        // reverse-PFM descent
        if (valve_open) {
          if (now - pfm_timer >= BURST_MS) {
            setValve(false, now); valve_open = false; pfm_timer = now;
            current_settle = constrain(
              (long)(settle_min + (actual - DEFLATE_END_KPA) *
                     (settle_max - settle_min) /
                     max(1.0, (map_anchor_high - DEFLATE_END_KPA))),
              settle_min, settle_max);
          }
        } else {
          if (now - pfm_timer >= current_settle) {
            settled_kPa = actual;        // latch the quiet reading
            setValve(true, now);
            valve_open = true; pfm_timer = now;
            // burst_count++;
          }
        }
      }
    }
  }
  else {
    // ---- MANUAL closed loop ----
    float error = target_kPa - actual;
    if (target_kPa <= 0.1) {
      setPump(false);
      if (actual > 1.0) setValve(true, now); else setValve(false, now);
      valve_open = false;
    }
    else if (error > DEADBAND_KPA) {
      setValve(false, now); setPump(true); valve_open = false; pfm_timer = now;
    }
    else if (error < -DEADBAND_KPA) {
      setPump(false);
      if (valve_open) {
        if (now - pfm_timer >= BURST_MS) {
          setValve(false, now); valve_open = false; pfm_timer = now;
          current_settle = constrain(
            (long)(settle_min + (settled_kPa - DEFLATE_END_KPA) *
                   (settle_max - settle_min) /
                   max(1.0, (map_anchor_high - DEFLATE_END_KPA))),
            settle_min, settle_max);
        }
      } else {
        if (now - pfm_timer >= current_settle) {
          setValve(true, now); valve_open = true; pfm_timer = now;
        }
      }
    }
    else {
      setPump(false); setValve(false, now); valve_open = false; pfm_timer = now;
    }
  }

  // ---- telemetry ----
  if (now - lastLogTime >= (unsigned long)log_interval_ms) {
    Serial.print("D,"); Serial.print(now);
    Serial.print(","); Serial.print(mode == M_MANUAL ? target_kPa : test_target);
    Serial.print(","); Serial.print(actual);
    Serial.print(","); Serial.println(modeName());
    lastLogTime = now;
  }
}