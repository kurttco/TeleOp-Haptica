/*
 * SLAVE FORCE SENSOR — ESP32 (5 directions)
 *
 * Graphite sponge sensors mounted on the slave end-effector.
 *
 * Directions:
 *   0: Right
 *   1: Left
 *   2: Forward
 *   3: Back
 *   4: Bottom
 *
 * Assumption:
 *   - End-effector always points DOWN
 *   - It does not rotate during motion
 *
 * Therefore:
 *   fx -> left/right contact
 *   fy -> forward/back contact
 *   fz -> bottom contact (cannot go further down)
 *
 * JSON output (20 Hz):
 *   {"r":[r0,r1,r2,r3,r4],
 *    "p":[p0,p1,p2,p3,p4],
 *    "fx":1.23,"fy":-0.45,"fz":3.10,"fm":3.37}
 *
 * CALIBRATION:
 *   1. Set CALIBRATION_MODE = true
 *   2. Note resting values -> BASELINE[]
 *   3. Press each sensor hard -> note minimums -> PRESSED[]
 *   4. Update arrays
 *   5. Set CALIBRATION_MODE = false
 */

#include <math.h>

const int NUM_SENSORS = 5;
const int PINS[NUM_SENSORS] = {34, 35, 32, 33, 25};
const char* NAMES[NUM_SENSORS] = {"Right","Left","Forward","Back","Bottom"};

// ===== CALIBRATION VALUES =====
int BASELINE[NUM_SENSORS] = {2200, 900, 1400, 2800, 1200};
int PRESSED[NUM_SENSORS]  = { 500, 200, 1600,  200,  450};

const bool CALIBRATION_MODE = false;
const int AVG_SAMPLES = 4;
const float ALPHA    = 0.30;
const float DEADZONE = 0.08;
const float FX_SCALE = 15.0;
const float FY_SCALE = 15.0;
const float FZ_SCALE = 15.0;

float smoothed[NUM_SENSORS] = {0,0,0,0,0};

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  delay(800);
  if (CALIBRATION_MODE) {
    Serial.println("=== SLAVE FORCE SENSOR CALIBRATION ===");
  } else {
    Serial.println("# Slave 5-dir force sensor ready");
  }
}

void loop() {
  int raw[NUM_SENSORS];
  float p[NUM_SENSORS];

  for (int i = 0; i < NUM_SENSORS; i++) {
    long sum = 0;
    for (int s = 0; s < AVG_SAMPLES; s++) sum += analogRead(PINS[i]);
    raw[i] = sum / AVG_SAMPLES;

    float range = (float)(BASELINE[i] - PRESSED[i]);
    if (range < 1.0) range = 1.0;
    float val = (float)(BASELINE[i] - raw[i]) / range;
    if (val < 0.0) val = 0.0;
    if (val > 1.0) val = 1.0;
    if (val < DEADZONE) val = 0.0;
    smoothed[i] = ALPHA * val + (1.0 - ALPHA) * smoothed[i];
    p[i] = smoothed[i];
  }

  if (CALIBRATION_MODE) {
    Serial.print("RAW: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(NAMES[i]); Serial.print("="); Serial.print(raw[i]);
      if (i < NUM_SENSORS - 1) Serial.print("  ");
    }
    Serial.println();
    delay(100);
    return;
  }

  float fx = (p[0] - p[1]) * FX_SCALE;
  float fy = (p[2] - p[3]) * FY_SCALE;
  float fz =  p[4]         * FZ_SCALE;
  float fm = sqrtf(fx*fx + fy*fy + fz*fz);

  Serial.print("{\"r\":[");
  for (int i=0;i<NUM_SENSORS;i++){Serial.print(raw[i]);if(i<NUM_SENSORS-1)Serial.print(",");}
  Serial.print("],\"p\":[");
  for (int i=0;i<NUM_SENSORS;i++){Serial.print(p[i],3);if(i<NUM_SENSORS-1)Serial.print(",");}
  Serial.print("],\"fx\":"); Serial.print(fx,3);
  Serial.print(",\"fy\":"); Serial.print(fy,3);
  Serial.print(",\"fz\":"); Serial.print(fz,3);
  Serial.print(",\"fm\":"); Serial.print(fm,3);
  Serial.println("}");

  delay(50);  // 20 Hz
}
