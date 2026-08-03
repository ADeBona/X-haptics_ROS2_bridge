/*
 * PAD CHARACTERISATION RIG
 *
 * Standalone test firmware for characterising a new pneumatic pad.
 * Phase 1 (finding the safe limit) should be done with the pad shielded,
 * not worn.
 *
 * Serial protocol, line terminated, 115200 baud.
 *
 *   U                         step target up   by STEP_KPA
 *   D                         step target down by STEP_KPA
 *   Z                         vent to zero, leave any test mode
 *   R                         reset the max-pressure recorder
 *   S                         report max pressure seen
 *   I,<target>                inflation test: vent, settle, full pump to target
 *   F,<target>,<Tmax>,<Tmin>  deflation test: inflate to target, settle,
 *                             top up, then reverse-PFM descent to
 *                             DEFLATE_END_KPA
 *
 * Output lines:
 *   D,<ms>,<target>,<pressure>,<state>       telemetry
 *   EVT,<text>                               events
 *   MAX,<kPa>                                max pressure since reset
 *   RESULT,INFLATE,<rise_ms>,<final_kPa>
 *   RESULT,DEFLATE,<fall_ms>,<final_kPa>,<burst_count>
 *
 * Pressure reporting: the sensor reads low while the valve is open, and
 * takes 60-80 ms to re-equalise after it closes. All control decisions,
 * telemetry and results therefore use settled_kPa, which is only written
 * when the line is known to be quiet: either the valve has been shut for
 * BLANK_MS, or we are about to open it at the end of an off-period. The
 * raw reading is used only for the hard ceiling check.
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
int settle_min = 95;                   // T_off at DEFLATE_END_KPA
int settle_max = 164;                  // T_off at map_anchor_high
const float DEFLATE_END_KPA = 2.0;     // descent terminates here
float map_anchor_high = 50.0;          // set to the test target by 'F'

// ---- Valve state and thermal protection ----
const unsigned long VALVE_MAX_ON_MS   = 5000;
const unsigned long VALVE_COOLDOWN_MS = 10000;
const unsigned long BLANK_MS          = 80;   // line equalisation after a burst

bool valve_energised = false;
unsigned long valve_on_since = 0;
bool valve_cooling = false;
unsigned long cooldown_start = 0;
unsigned long valve_close_time = 0;

// ---- Free venting (PH_VENT, manual vent-to-zero) ----
// settled_kPa only updates while the valve is shut, so venting has to be
// duty-cycled rather than held open solid, or the reading freezes and the
// vent can only ever end via the thermal cap forcing the valve shut.
const unsigned long VENT_OPEN_MS   = 400;  // burst-open duration while purging
const unsigned long VENT_SAMPLE_MS = 120;  // closed window to get a clean read
bool vent_open = true;
unsigned long vent_timer = 0;

float settled_kPa = 0.0;               // the trusted pressure reading

// ---- Modes ----
enum Mode { M_MANUAL, M_INFLATE_TEST, M_DEFLATE_TEST };
Mode mode = M_MANUAL;

enum Phase { PH_IDLE, PH_VENT, PH_SETTLE, PH_RUN, PH_DONE };
Phase phase = PH_IDLE;

float target_kPa = 0.0;
float sensor_offset_kPa = 0.0;
float max_seen_kPa = 0.0;
float test_target = 0.0;

unsigned long phase_start = 0;
unsigned long test_start = 0;
unsigned long settle_start = 0;
bool reached_target = false;

// ---- PFM state ----
bool valve_open = false;
unsigned long pfm_timer = 0;
unsigned long current_settle = 0;
int burst_count = 0;

// ---- logging ----
unsigned long lastLogTime = 0;
int log_interval_ms = 100;             // 10 Hz idle, 20 ms during tests

// ---- serial ----
char buf[48];
int bufIdx = 0;


float convertADCToKPa(int adcValue) {
  return ((adcValue / 1023.0) - 0.04) / 0.009;
}

void setPump(bool on) { digitalWrite(PUMP_PIN, on ? HIGH : LOW); }

void setValve(bool on, unsigned long now) {
  if (valve_cooling) {
    if (now - cooldown_start < VALVE_COOLDOWN_MS) {
      digitalWrite(VALVE_PIN, LOW);
      if (valve_energised) valve_close_time = now;
      valve_energised = false;
      return;
    }
    valve_cooling = false;
  }
  if (on) {
    if (!valve_energised) { valve_energised = true; valve_on_since = now; }
    else if (now - valve_on_since > VALVE_MAX_ON_MS) {
      Serial.println("EVT,valve thermal cap - cooldown");
      valve_cooling = true; cooldown_start = now;
      digitalWrite(VALVE_PIN, LOW);
      valve_close_time = now;
      valve_energised = false;
      return;
    }
    digitalWrite(VALVE_PIN, HIGH);
  } else {
    if (valve_energised) valve_close_time = now;
    digitalWrite(VALVE_PIN, LOW);
    valve_energised = false;
  }
}

// Linear T_off schedule: long waits at high pressure, short at low, so that
// the larger volume vented per burst at high dP is compensated.
unsigned long settleFor(float p) {
  float span = map_anchor_high - DEFLATE_END_KPA;
  if (span < 1.0) span = 1.0;
  float frac = (p - DEFLATE_END_KPA) / span;
  if (frac < 0.0) frac = 0.0;
  if (frac > 1.0) frac = 1.0;
  long s = (long)(settle_min + frac * (settle_max - settle_min));
  if (settle_min < settle_max) {
    s = constrain(s, settle_min, settle_max);
  } else {
    s = constrain(s, settle_max, settle_min);
  }
  return (unsigned long)s;
}

void calibrateSensorOffset() {
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(VALVE_PIN, HIGH);
  delay(1500);
  float s = 0.0;
  for (int i = 0; i < 50; i++) {
    s += convertADCToKPa(analogRead(SENSOR_PIN));
    delay(10);
  }
  sensor_offset_kPa = s / 50.0;
  digitalWrite(VALVE_PIN, LOW);
}

const char* stateName() {
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
      vent_open = true; vent_timer = now;
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
      vent_open = true; vent_timer = now;
      Serial.print("EVT,deflation test "); Serial.print(test_target);
      Serial.print(" Tmax="); Serial.print(settle_max);
      Serial.print(" Tmin="); Serial.println(settle_min);
    }
  }
}

void loop() {
  unsigned long now = millis();

  // ---- 1. SERIAL ----
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufIdx > 0) { buf[bufIdx] = '\0'; handleCommand(now); bufIdx = 0; }
    } else if (bufIdx < 47) {
      buf[bufIdx++] = c;
    }
  }

  // ---- 2. READ ----
  float a1 = convertADCToKPa(analogRead(SENSOR_PIN));
  float a2 = convertADCToKPa(analogRead(SENSOR_PIN));
  float a3 = convertADCToKPa(analogRead(SENSOR_PIN));
  float raw = max(min(a1, a2), min(max(a1, a2), a3)) - sensor_offset_kPa;
  if (raw < 0.0) raw = 0.0;

  // Update the trusted reading only when the line is quiet. During fast PFM
  // cycling this window never opens, and the pre-burst latch in PH_RUN is
  // the only writer - exactly one clean sample per cycle.
  if (!valve_energised && (now - valve_close_time) >= BLANK_MS) {
    settled_kPa = raw;
  }
  if (settled_kPa > max_seen_kPa) max_seen_kPa = settled_kPa;

  // ---- 3. HARD SAFETY (uses the unfiltered reading) ----
  if (raw > HARD_CEILING_KPA) {
    setPump(false); setValve(true, now);
    target_kPa = 0.0; mode = M_MANUAL; phase = PH_IDLE;
    log_interval_ms = 100;
    Serial.println("EVT,CEILING EXCEEDED - emergency vent");
  }

  // ---- 4. SEQUENCING ----
  if (mode == M_INFLATE_TEST) {

    if (phase == PH_VENT) {
      setPump(false);
      if (vent_open) {
        setValve(true, now);
        if (now - vent_timer >= VENT_OPEN_MS) { vent_open = false; vent_timer = now; }
      } else {
        setValve(false, now);
        if (now - vent_timer >= VENT_SAMPLE_MS) { vent_open = true; vent_timer = now; }
      }
      if (settled_kPa < 1.0 || now - phase_start > 8000) {
        setValve(false, now);
        phase = PH_SETTLE; phase_start = now;
      }
    }
    else if (phase == PH_SETTLE) {
      setPump(false); setValve(false, now);
      if (now - phase_start > 1000) {
        phase = PH_RUN; test_start = now;
        Serial.println("EVT,inflate start");
      }
    }
    else if (phase == PH_RUN) {
      setValve(false, now); setPump(true);
      if (settled_kPa >= test_target - DEADBAND_KPA || now - test_start > 30000) {
        setPump(false);
        Serial.print("RESULT,INFLATE,"); Serial.print(now - test_start);
        Serial.print(","); Serial.println(settled_kPa);
        phase = PH_DONE; target_kPa = test_target;
        mode = M_MANUAL; log_interval_ms = 100;
      }
    }
  }
  else if (mode == M_DEFLATE_TEST) {

    if (phase == PH_VENT) {
      setPump(false);
      if (vent_open) {
        setValve(true, now);
        if (now - vent_timer >= VENT_OPEN_MS) { vent_open = false; vent_timer = now; }
      } else {
        setValve(false, now);
        if (now - vent_timer >= VENT_SAMPLE_MS) { vent_open = true; vent_timer = now; }
      }
      if (settled_kPa < 1.0 || now - phase_start > 8000) {
        setValve(false, now);
        phase = PH_SETTLE; phase_start = now;
      }
    }
    else if (phase == PH_SETTLE) {
      setValve(false, now);

      if (!reached_target) {
        if (settled_kPa >= test_target - DEADBAND_KPA) {
          reached_target = true;
          setPump(false);
          settle_start = now;
        } else {
          setPump(true);
        }
      } else {
        if (now - settle_start < 1000) {
          setPump(false);                        // let the pad settle
        } else if (settled_kPa < test_target - 0.5) {
          setPump(true);                         // top up the bleed
        } else {
          setPump(false);
          phase = PH_RUN; test_start = now;
          pfm_timer = now; valve_open = false;
          burst_count = 0;
          current_settle = 0;                    // fire the first burst at once
          Serial.print("EVT,deflate start at "); Serial.println(settled_kPa);
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
        Serial.print(","); Serial.print(settled_kPa);
        Serial.print(","); Serial.println(burst_count);
        phase = PH_DONE; target_kPa = 0.0;
        mode = M_MANUAL; log_interval_ms = 100;
      }
      else if (valve_open) {
        if (now - pfm_timer >= BURST_MS) {
          setValve(false, now);
          valve_open = false; pfm_timer = now;
          current_settle = settleFor(settled_kPa);
        }
      }
      else {
        if (now - pfm_timer >= current_settle) {
          settled_kPa = raw;                     // latch the quiet reading
          if (settled_kPa > max_seen_kPa) max_seen_kPa = settled_kPa;
          setValve(true, now);
          valve_open = true; pfm_timer = now;
          burst_count++;
        }
      }
    }
  }
  else {
    // ---- MANUAL closed loop ----
    float error = target_kPa - settled_kPa;

    if (target_kPa <= 0.1) {
      setPump(false);
      if (settled_kPa > 1.0) {
        if (vent_open) {
          setValve(true, now);
          if (now - vent_timer >= VENT_OPEN_MS) { vent_open = false; vent_timer = now; }
        } else {
          setValve(false, now);
          if (now - vent_timer >= VENT_SAMPLE_MS) { vent_open = true; vent_timer = now; }
        }
      } else {
        setValve(false, now);
        vent_open = true; vent_timer = now;   // start open again next time
      }
      valve_open = false; pfm_timer = now;
    }
    else if (error > DEADBAND_KPA) {
      setValve(false, now); setPump(true);
      valve_open = false; pfm_timer = now;
    }
    else if (error < -DEADBAND_KPA) {
      setPump(false);
      if (valve_open) {
        if (now - pfm_timer >= BURST_MS) {
          setValve(false, now);
          valve_open = false; pfm_timer = now;
          current_settle = settleFor(settled_kPa);
        }
      } else {
        if (now - pfm_timer >= current_settle) {
          settled_kPa = raw;                     // latch before opening
          setValve(true, now);
          valve_open = true; pfm_timer = now;
        }
      }
    }
    else {
      setPump(false); setValve(false, now);
      valve_open = false; pfm_timer = now;
    }
  }

  // ---- 5. TELEMETRY ----
  if (now - lastLogTime >= (unsigned long)log_interval_ms) {
    Serial.print("D,"); Serial.print(now);
    Serial.print(","); Serial.print(mode == M_MANUAL ? target_kPa : test_target);
    Serial.print(","); Serial.print(settled_kPa);
    Serial.print(","); Serial.println(stateName());
    lastLogTime = now;
  }
}