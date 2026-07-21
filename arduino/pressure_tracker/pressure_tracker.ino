/*
 * PURE DIGITAL PRESSURE TRACKER - NON-BLOCKING REVERSE-PFM
 * optimised bounds: 6 ms burst, T_off 95 ms (low P) to 164 ms (high P).
 */

const int SENSOR_PIN = A0;
const int PUMP_PIN   = 3;
const int VALVE_PIN  = 5;

const float MAX_PRESSURE = 60.0;
const float DEADBAND_KPA = 2.5;

// Reverse-PFM parameters
const unsigned long BURST_MS = 7;     // minimum mechanical actuation time
const int P_LOW  = 10;                // kPa, lower bound of the T_off map
const int P_HIGH = 50;                // kPa, upper bound of the T_off map
int settle_min = 95;                  // T_off at P_LOW
int settle_max = 164;                 // T_off at P_HIGH

float target_kPa = 0.0;
float sensor_offset_kPa = 0.0;

// PFM state machine
bool valve_open = false;
unsigned long pfm_timer = 0;
unsigned long current_settle = 0;

unsigned long lastLogTime = 0;
const int LOG_INTERVAL_MS = 50;

char serialBuffer[32];
int bufferIndex = 0;

void calibrateSensorOffset();
float convertADCToKPa(int adcValue);

void setup() {
  Serial.begin(115200);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(VALVE_PIN, LOW);
  calibrateSensorOffset();
}

void loop() {
  unsigned long now = millis();

  // ---- 1. SERIAL PARSING (non-blocking) ----
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufferIndex > 0) {
        serialBuffer[bufferIndex] = '\0';
        int commaCount = 0;
        for (int i = 0; i < bufferIndex; i++) {
          if (serialBuffer[i] == ',') commaCount++;
        }
        if (commaCount == 2) {
          // "target,settle_max,settle_min" for live RL tuning
          char* t_str   = strtok(serialBuffer, ",");
          char* max_str = strtok(NULL, ",");
          char* min_str = strtok(NULL, ",");
          if (t_str && max_str && min_str) {
            target_kPa = constrain(atof(t_str), 0.0, MAX_PRESSURE);
            settle_max = constrain(atoi(max_str), 50, 300);
            settle_min = constrain(atoi(min_str), 10, 150);
          }
        } else {
          target_kPa = constrain(atof(serialBuffer), 0.0, MAX_PRESSURE);
        }
        bufferIndex = 0;
      }
    } else if ((c >= '0' && c <= '9') || c == '.' || c == ',') {
      if (bufferIndex < 31) {
        serialBuffer[bufferIndex++] = c;
      }
    }
  }

  // ---- 2. READ & TARE ----
  float actual_kPa = convertADCToKPa(analogRead(SENSOR_PIN)) - sensor_offset_kPa;
  if (actual_kPa < 0.0) actual_kPa = 0.0;
  float error = target_kPa - actual_kPa;

  // ---- 3. CLOSED-LOOP CONTROL ----
  if (target_kPa <= 0.1 && actual_kPa < 2.0) {
    // FULL EXHAUST: purge the last residual air without chattering
    digitalWrite(PUMP_PIN, LOW);
    digitalWrite(VALVE_PIN, HIGH);
    valve_open = false;
  }
  else if (error > DEADBAND_KPA) {
    // INFLATE: continuous bang-bang
    digitalWrite(VALVE_PIN, LOW);
    digitalWrite(PUMP_PIN, HIGH);
    valve_open = false;
    pfm_timer = now;
  }
  else if (error < -DEADBAND_KPA) {
    // DEFLATE: reverse-PFM, non-blocking
    digitalWrite(PUMP_PIN, LOW);

    if (valve_open) {
      if (now - pfm_timer >= BURST_MS) {
        digitalWrite(VALVE_PIN, LOW);
        valve_open = false;
        pfm_timer = now;
        // Recompute T_off from the pressure at the end of this burst
        current_settle = constrain(
            map((int)actual_kPa, P_LOW, P_HIGH, settle_min, settle_max),
            settle_min, settle_max);
      }
    } else {
      if (now - pfm_timer >= current_settle) {
        digitalWrite(VALVE_PIN, HIGH);
        valve_open = true;
        pfm_timer = now;
      }
    }
  }
  else {
    // HOLD
    digitalWrite(PUMP_PIN, LOW);
    digitalWrite(VALVE_PIN, LOW);
    valve_open = false;
    pfm_timer = now;
  }

  // ---- 4. TELEMETRY ----
  if (now - lastLogTime >= LOG_INTERVAL_MS) {
    Serial.print("Target:");
    Serial.print(target_kPa);
    Serial.print(", Actual:");
    Serial.println(actual_kPa);
    lastLogTime = now;
  }
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