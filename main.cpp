#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Preferences.h>
#include <esp_system.h>

static const char *AP_SSID = "MicroQuad";
static const char *AP_PASS = "flyquad123";

static const int PINS[4] = {16, 17, 18, 19};

static const float ROLL_MIX[4]  = {-1, -1, +1, +1};
static const float PITCH_MIX[4] = {-1, +1, +1, -1};
static const float YAW_PATTERN[4] = {+1, -1, +1, -1};
static volatile float YAW_SIGN = +1.0f;

static const float ROLL_SIGN  = 1.0f;
static const float PITCH_SIGN = 1.0f;

static const int   PWM_FREQ = 20000;
static const int   PWM_RES  = 8;
static const float MAX_PCT  = 75.0f;
static const float IDLE_PCT = 6.0f;
static const int   LOOP_HZ  = 500;

static const float GYRO_LPF_HZ = 110.0f;
static const float DTERM_LPF_HZ = 70.0f;
static const float LEVEL_TAU_S = 0.8f;
static const float LEVEL_TAU_FLIGHT_S = 6.0f;
static const float LEVEL_ACC_TRUST_WINDOW = 0.30f;

static const float MAX_RATE_DPS  = 220.0f;
static const float MAX_CORR_PCT  = 40.0f;
static const float MAX_YAW_PCT   = 25.0f;

static const float TRIM_LIMIT_DEG = 15.0f;

static const int   BARO_EVERY      = 20;
static const float VZ_LPF_HZ       = 3.0f;
static const float BARO_P0_TRACK   = 0.05f;

static const float TILT_CUT_DEG  = 60.0f;
static const float ARM_MAX_TILT  = 20.0f;
static const unsigned long FAILSAFE_MS = 3000;
static volatile unsigned long HOP_LIMIT_MS = 0;

static const float I_ENABLE_PCT = 20.0f;
static volatile float KP_ANGLE = 4.0f;
static volatile float KP_RATE  = 0.15f;
static volatile float KI_RATE  = 0.10f;
static volatile float KD_RATE  = 0.00f;
static volatile float KP_YAW   = 0.25f;
static volatile float KI_YAW   = 0.20f;
static const   float I_RATE_LIMIT = 80.0f;

#define USE_BOOT_KILL 1

static uint8_t MPU = 0x68;
static const float GYRO_LSB = 65.5f;
static const float ACC_LSB  = 8192.0f;
static const float GLITCH_DPS   = 250.0f;

static WebServer server(80);
static Preferences prefs;

static volatile bool armed = false;
static volatile bool killLatch = false;
static volatile bool imuOK = false;
static volatile bool benchMode = false;
static volatile float throttlePct = 0;
static volatile unsigned long lastBeat = 0;
static volatile unsigned long armMillis = 0;

static volatile float tRoll = 0, tPitch = 0;
static volatile float tRollRate = 0, tPitchRate = 0, tYawRate = 0;
static volatile float tOutR = 0, tOutP = 0, tOutY = 0;
static volatile float tSat = 0;
static volatile int   tM[4] = {0, 0, 0, 0};
static volatile float tLoopHz = 0;
static volatile float tWebHz = 0;
static volatile int   tResetReason = 0;

static volatile unsigned long hbCount = 0;
static volatile unsigned long pageHits = 0;
static volatile unsigned long notFound = 0;
static volatile int  tStations = 0;
static volatile uint32_t tMinHeap = 0;

static volatile bool  baroOK = false;
static volatile float tAlt = 0, tVz = 0;
static volatile float tBaroP = 0, tBaroT = 0;

static volatile float trimRoll = 0, trimPitch = 0;
static float levelRollRef = 0, levelPitchRef = 0;
static volatile bool levelValid = true;
static volatile bool relevelReq = false;
static float bootTiltR = 0, bootTiltP = 0;
static float gBias[3] = {0, 0, 0};
static float aBias[3] = {0, 0, 0};

static volatile int  testMotor = -1;
static volatile float testMotorPct = 18.0f;
static volatile unsigned long testUntil = 0;

#define LOG_DEPTH 200
static float logBuf[LOG_DEPTH][6];
static volatile int logIdx = 0;

static void pwmInit() {
  for (int i = 0; i < 4; i++) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(PINS[i], PWM_FREQ, PWM_RES);
#else
    ledcSetup(i, PWM_FREQ, PWM_RES);
    ledcAttachPin(PINS[i], i);
#endif
  }
}

static inline void pwmWrite(int i, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PINS[i], duty);
#else
  ledcWrite(i, duty);
#endif
}

static void setAll(int duty) {
  for (int i = 0; i < 4; i++) pwmWrite(i, duty);
}

static bool imuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static uint8_t imuReadReg(uint8_t reg) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom((int)MPU, 1, (int)true) != 1) return 0xFF;
  return (uint8_t)Wire.read();
}

static bool imuRead(int16_t v[7]) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU, 14, (int)true) != 14) return false;
  for (int i = 0; i < 7; i++) {
    int hi = Wire.read(), lo = Wire.read();
    if (hi < 0 || lo < 0) return false;
    v[i] = (int16_t)((hi << 8) | lo);
  }
  return true;
}

static bool imuFind() {
  const uint8_t cand[2] = {0x68, 0x69};
  for (int c = 0; c < 2; c++) {
    MPU = cand[c];
    Wire.beginTransmission(MPU);
    if (Wire.endTransmission() != 0) continue;
    uint8_t who = imuReadReg(0x75);
    Serial.printf("MPU probe 0x%02X -> WHO_AM_I 0x%02X\n", MPU, who);
    if (who == 0x68 || who == 0x70 || who == 0x71 || who == 0x73 || who == 0x98) return true;
    if (who == 0x00 || who == 0xFF) {
      Serial.println("  (WHO_AM_I odd, trying it anyway)");
      return true;
    }
  }
  return false;
}

static bool imuInit() {
  if (!imuFind()) return false;
  if (!imuWrite(0x6B, 0x01)) return false;
  delay(100);
  imuWrite(0x1B, 0x08);
  imuWrite(0x1C, 0x08);
  imuWrite(0x1A, 0x03);
  imuWrite(0x19, 0x01);
  delay(50);
  int16_t v[7];
  return imuRead(v);
}

static uint8_t  BMP = 0x76;
static uint16_t bT1, bP1;
static int16_t  bT2, bT3, bP2, bP3, bP4, bP5, bP6, bP7, bP8, bP9;

static bool bmpWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BMP);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool bmpReadBytes(uint8_t reg, uint8_t *buf, int len) {
  Wire.beginTransmission(BMP);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)BMP, len, (int)true) != len) return false;
  for (int i = 0; i < len; i++) {
    int b = Wire.read();
    if (b < 0) return false;
    buf[i] = (uint8_t)b;
  }
  return true;
}

static bool baroRead(float &pPa, float &tC) {
  uint8_t d[6];
  if (!bmpReadBytes(0xF7, d, 6)) return false;
  int32_t adcP = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
  int32_t adcT = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);
  if (adcP == 0x80000 || adcT == 0x80000) return false;

  double v1 = (((double)adcT) / 16384.0 - ((double)bT1) / 1024.0) * (double)bT2;
  double v2 = (((double)adcT) / 131072.0 - ((double)bT1) / 8192.0);
  v2 = v2 * v2 * (double)bT3;
  double tFine = v1 + v2;
  tC = (float)(tFine / 5120.0);

  v1 = tFine / 2.0 - 64000.0;
  v2 = v1 * v1 * (double)bP6 / 32768.0;
  v2 = v2 + v1 * (double)bP5 * 2.0;
  v2 = v2 / 4.0 + (double)bP4 * 65536.0;
  v1 = ((double)bP3 * v1 * v1 / 524288.0 + (double)bP2 * v1) / 524288.0;
  v1 = (1.0 + v1 / 32768.0) * (double)bP1;
  if (v1 == 0.0) return false;
  double p = 1048576.0 - (double)adcP;
  p = (p - v2 / 4096.0) * 6250.0 / v1;
  v1 = (double)bP9 * p * p / 2147483648.0;
  v2 = p * (double)bP8 / 32768.0;
  p = p + (v1 + v2 + (double)bP7) / 16.0;

  if (p < 30000.0 || p > 110000.0) return false;
  pPa = (float)p;
  return true;
}

static bool baroInit() {
  const uint8_t cand[2] = {0x76, 0x77};
  bool found = false;
  for (int k = 0; k < 2 && !found; k++) {
    BMP = cand[k];
    uint8_t id = 0;
    if (!bmpReadBytes(0xD0, &id, 1)) continue;
    Serial.printf("BARO probe 0x%02X -> chip ID 0x%02X\n", BMP, id);
    if (id == 0x58 || id == 0x60) found = true;
  }
  if (!found) return false;

  bmpWrite(0xE0, 0xB6);
  delay(10);
  uint8_t c[24];
  if (!bmpReadBytes(0x88, c, 24)) return false;
#define LE16(i) ((uint16_t)(c[i] | (c[(i) + 1] << 8)))
  bT1 = LE16(0);  bT2 = (int16_t)LE16(2);  bT3 = (int16_t)LE16(4);
  bP1 = LE16(6);  bP2 = (int16_t)LE16(8);  bP3 = (int16_t)LE16(10);
  bP4 = (int16_t)LE16(12); bP5 = (int16_t)LE16(14); bP6 = (int16_t)LE16(16);
  bP7 = (int16_t)LE16(18); bP8 = (int16_t)LE16(20); bP9 = (int16_t)LE16(22);
#undef LE16
  bmpWrite(0xF5, 0x08);
  bmpWrite(0xF4, 0x53);
  delay(60);
  float p, t;
  if (!baroRead(p, t)) return false;
  Serial.printf("BARO %.1f Pa  %.1f C\n", (double)p, (double)t);
  return true;
}

static bool calibrateImu() {
  Serial.println("Calibrating: hold the quad STILL and LEVEL for 2 s...");
  const int N = 1000;
  long sa[3] = {0, 0, 0}, sg[3] = {0, 0, 0};
  int n = 0;
  int16_t v[7];
  for (int i = 0; i < N; i++) {
    if (imuRead(v)) {
      sa[0] += v[0]; sa[1] += v[1]; sa[2] += v[2];
      sg[0] += v[4]; sg[1] += v[5]; sg[2] += v[6];
      n++;
    }
    delay(2);
    if ((i & 0x3F) == 0) yield();
  }
  if (n < N / 3) { Serial.println("Calibration FAILED: not enough samples"); return false; }

  for (int k = 0; k < 3; k++) {
    gBias[k] = (float)sg[k] / n;
    aBias[k] = (float)sa[k] / n;
  }
  aBias[2] -= ACC_LSB;

  float mx = (float)sa[0] / n, my = (float)sa[1] / n, mz = (float)sa[2] / n;
  bootTiltR = atan2f(-mx, sqrtf(my * my + mz * mz)) * 57.29578f * ROLL_SIGN;
  bootTiltP = atan2f( my, sqrtf(mx * mx + mz * mz)) * 57.29578f * PITCH_SIGN;
  levelRollRef = levelPitchRef = 0.0f;
  levelValid = (fabsf(bootTiltR) <= 10.0f && fabsf(bootTiltP) <= 10.0f);

  Serial.printf("gyro bias %.1f %.1f %.1f dps | boot tilt R %.1f P %.1f\n",
                (double)(gBias[0] / GYRO_LSB), (double)(gBias[1] / GYRO_LSB),
                (double)(gBias[2] / GYRO_LSB),
                (double)bootTiltR, (double)bootTiltP);
  if (!levelValid)
    Serial.println("NOT LEVEL at boot: put it flat and press LEVEL on the page before arming");
  return true;
}

static void relevel() {
  const int N = 200;
  int16_t v[7];
  long sa[3] = {0, 0, 0}, sg[3] = {0, 0, 0};
  int n = 0;
  for (int i = 0; i < N; i++) {
    if (imuRead(v)) {
      sa[0] += v[0]; sa[1] += v[1]; sa[2] += v[2];
      sg[0] += v[4]; sg[1] += v[5]; sg[2] += v[6];
      n++;
    }
    delay(2);
  }
  if (!n) return;
  for (int k = 0; k < 3; k++) gBias[k] = (float)sg[k] / n;
  float ax = (float)sa[0] / n - aBias[0];
  float ay = (float)sa[1] / n - aBias[1];
  float az = (float)sa[2] / n - aBias[2];
  levelRollRef  = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f * ROLL_SIGN;
  levelPitchRef = atan2f( ay, sqrtf(ax * ax + az * az)) * 57.29578f * PITCH_SIGN;
  trimRoll = trimPitch = 0;
  levelValid = true;
  Serial.printf("Level re-zeroed: R %.2f P %.2f | gyro bias %.1f %.1f %.1f dps\n",
                (double)levelRollRef, (double)levelPitchRef,
                (double)(gBias[0] / GYRO_LSB), (double)(gBias[1] / GYRO_LSB), (double)(gBias[2] / GYRO_LSB));
}

static void mixAndWrite(float base, float outR, float outP, float outY, bool write) {
  float dev[4];
  for (int i = 0; i < 4; i++)
    dev[i] = ROLL_MIX[i] * outR + PITCH_MIX[i] * outP + YAW_PATTERN[i] * (YAW_SIGN * outY);

  float dmin = dev[0], dmax = dev[0];
  for (int i = 1; i < 4; i++) { dmin = fminf(dmin, dev[i]); dmax = fmaxf(dmax, dev[i]); }

  float bMin = IDLE_PCT - dmin;
  float bMax = 100.0f - dmax;
  float sat = 0.0f;

  if (bMin > bMax) {
    float span = dmax - dmin;
    float room = 100.0f - IDLE_PCT;
    float k = (span > 1e-4f) ? (room / span) : 1.0f;
    sat = 100.0f * (1.0f - k);
    for (int i = 0; i < 4; i++) dev[i] *= k;
    bMin = IDLE_PCT - dmin * k;
    bMax = 100.0f - dmax * k;
  }

  float b = base;
  if (b < bMin) b = bMin;
  if (b > bMax) b = bMax;

  for (int i = 0; i < 4; i++) {
    float d = b + dev[i];
    d = constrain(d, IDLE_PCT, 100.0f);
    tM[i] = (int)(d + 0.5f);
    if (write) pwmWrite(i, (int)(d * 2.55f + 0.5f));
  }
  tSat = sat;
}

static float pt1(float in, float &state, float freq, float dt) {
  if (freq <= 0) return in;
  float tau = 1.0f / (6.2831853f * freq);
  float k = dt / (tau + dt);
  state += (in - state) * k;
  return state;
}

static void controlTask(void *) {
  const TickType_t period = pdMS_TO_TICKS(1000 / LOOP_HZ);
  TickType_t wake = xTaskGetTickCount();

  float roll = 0, pitch = 0;
  float fR = 0, fP = 0, fY = 0;
  float fD = 0;
  float prevR = 0, prevP = 0;
  float lastRollRate = 0, lastPitchRate = 0, lastYawRate = 0;
  int rejCount = 0;
  float iR = 0, iP = 0, iY = 0;
  float base = 0;
  bool seeded = false;
  int fails = 0;
  unsigned long hopStart = 0;
  uint32_t prevUs = micros();
  uint32_t hzCount = 0;
  uint32_t hzT0 = millis();
  int16_t v[7];
  int baroTick = 0, baroFails = 0;
  bool baroSeeded = false;
  float baroP0 = 101325.0f, altPrev = 0, fVz = 0;
  uint32_t baroPrevUs = micros();

  for (;;) {
    vTaskDelayUntil(&wake, period);

    uint32_t nowUs = micros();
    float dt = (nowUs - prevUs) * 1e-6f;
    prevUs = nowUs;
    if (dt < 0.0005f) dt = 0.0005f;
    if (dt > 0.02f)   dt = 0.02f;

    hzCount++;
    if (millis() - hzT0 >= 500) {
      tLoopHz = hzCount * 1000.0f / (millis() - hzT0);
      hzCount = 0;
      hzT0 = millis();
    }

    if (!imuRead(v)) {
      if (++fails > 5) {
        armed = false;
        setAll(0);
        for (int i = 0; i < 4; i++) tM[i] = 0;
        tOutR = tOutP = tOutY = 0; tSat = 0;
        if (fails > 200) killLatch = true;
      }
      continue;
    }
    fails = 0;

    float ax = (float)v[0] - aBias[0];
    float ay = (float)v[1] - aBias[1];
    float az = (float)v[2] - aBias[2];

    float rollRate  = ((float)v[5] - gBias[1]) / GYRO_LSB * ROLL_SIGN;
    float pitchRate = ((float)v[4] - gBias[0]) / GYRO_LSB * PITCH_SIGN;
    float yawRate   = ((float)v[6] - gBias[2]) / GYRO_LSB;

    if (fabsf(rollRate  - lastRollRate)  > GLITCH_DPS ||
        fabsf(pitchRate - lastPitchRate) > GLITCH_DPS ||
        fabsf(yawRate   - lastYawRate)   > GLITCH_DPS) {
      if (++rejCount < 3) {
        rollRate  = lastRollRate;
        pitchRate = lastPitchRate;
        yawRate   = lastYawRate;
      } else {
        rejCount = 0;
        lastRollRate = rollRate; lastPitchRate = pitchRate; lastYawRate = yawRate;
      }
    } else {
      rejCount = 0;
      lastRollRate  = rollRate;
      lastPitchRate = pitchRate;
      lastYawRate   = yawRate;
    }

    fR = pt1(rollRate,  fR, GYRO_LPF_HZ, dt);
    fP = pt1(pitchRate, fP, GYRO_LPF_HZ, dt);
    fY = pt1(yawRate,   fY, GYRO_LPF_HZ, dt);

    float aMag = sqrtf(ax * ax + ay * ay + az * az) / ACC_LSB;
    float trust = 1.0f - fabsf(aMag - 1.0f) / LEVEL_ACC_TRUST_WINDOW;
    trust = constrain(trust, 0.0f, 1.0f);
    if (aMag < 0.25f) trust = 0.0f;

    float accRoll  = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f * ROLL_SIGN;
    float accPitch = atan2f( ay, sqrtf(ax * ax + az * az)) * 57.29578f * PITCH_SIGN;

    if (!seeded) { roll = accRoll; pitch = accPitch; seeded = true; }

    float dtc = dt;
    if (dtc > 0.01f) dtc = 0.01f;
    float tauL  = armed ? LEVEL_TAU_FLIGHT_S : LEVEL_TAU_S;
    float alpha = tauL / (tauL + dtc);
    float wAcc  = (1.0f - alpha) * trust;

    roll  = (1.0f - wAcc) * (roll  + fR * dtc) + wAcc * accRoll;
    pitch = (1.0f - wAcc) * (pitch + fP * dtc) + wAcc * accPitch;

    float relRoll  = roll  - levelRollRef  - trimRoll;
    float relPitch = pitch - levelPitchRef - trimPitch;

    tRoll = relRoll;  tPitch = relPitch;
    tRollRate = fR;   tPitchRate = fP;   tYawRate = fY;

    if (baroOK && ++baroTick >= BARO_EVERY) {
      baroTick = 0;
      float pPa, tC;
      if (baroRead(pPa, tC)) {
        baroFails = 0;
        uint32_t nu = micros();
        float bdt = (nu - baroPrevUs) * 1e-6f;
        baroPrevUs = nu;
        tBaroP = pPa; tBaroT = tC;
        if (!baroSeeded) {
          baroP0 = pPa; baroSeeded = true; altPrev = 0; fVz = 0;
        } else {
          if (!armed) baroP0 += (pPa - baroP0) * BARO_P0_TRACK;
          float alt = 44330.0f * (1.0f - powf(pPa / baroP0, 0.1903f));
          if (bdt > 0.005f && bdt < 0.3f)
            fVz = pt1((alt - altPrev) / bdt, fVz, VZ_LPF_HZ, bdt);
          altPrev = alt;
          tAlt = alt;
          tVz  = fVz;
        }
      } else if (++baroFails > 25) {
        baroOK = false;
        tAlt = 0; tVz = 0;
      }
    }

    bool killNow = killLatch;
#if USE_BOOT_KILL
    static int bootLow = 0;
    if (digitalRead(0) == LOW) { if (++bootLow > 25) killNow = true; }
    else bootLow = 0;
#endif

    if (killNow) {
      armed = false;
      base = 0; iR = iP = iY = 0;
      setAll(0);
      for (int i = 0; i < 4; i++) tM[i] = 0;
      tOutR = tOutP = tOutY = 0; tSat = 0;
      killLatch = true;
      hopStart = 0;
      continue;
    }

    if (testMotor >= 0) {
      if (millis() > testUntil || armed) { testMotor = -1; setAll(0); }
      else {
        int i = testMotor;
        float d = testMotorPct;
        for (int k = 0; k < 4; k++) {
          pwmWrite(k, k == i ? (int)(d * 2.55f) : 0);
          tM[k] = (k == i) ? (int)d : 0;
        }
        continue;
      }
    }

    if (!armed) {
      if (relevelReq) { relevelReq = false; relevel(); continue; }
      base = 0; iR = iP = iY = 0; hopStart = 0;
      tOutR = tOutP = tOutY = 0; tSat = 0;
      setAll(0);
      for (int i = 0; i < 4; i++) tM[i] = 0;
      if (benchMode) {
        float rSP = constrain(KP_ANGLE * (0 - relRoll),  -MAX_RATE_DPS, MAX_RATE_DPS);
        float pSP = constrain(KP_ANGLE * (0 - relPitch), -MAX_RATE_DPS, MAX_RATE_DPS);
        float eR = fR - rSP, eP = fP - pSP;
        float oR = constrain(KP_RATE * eR, -MAX_CORR_PCT, MAX_CORR_PCT);
        float oP = constrain(KP_RATE * eP, -MAX_CORR_PCT, MAX_CORR_PCT);
        float oY = constrain(-KP_YAW * fY, -MAX_YAW_PCT, MAX_YAW_PCT);
        mixAndWrite(40.0f, oR, oP, oY, false);
        tOutR = oR; tOutP = oP; tOutY = oY;
      }
      continue;
    }

    if (fabsf(relRoll) > TILT_CUT_DEG || fabsf(relPitch) > TILT_CUT_DEG ||
        millis() - lastBeat > FAILSAFE_MS) {
      armed = false; base = 0; iR = iP = iY = 0; hopStart = 0;
      setAll(0);
      for (int i = 0; i < 4; i++) tM[i] = 0;
      continue;
    }

    if (throttlePct < 3.0f) {
      base = 0; iR = iP = iY = 0; hopStart = 0;
      setAll(0);
      for (int i = 0; i < 4; i++) tM[i] = 0;
      tOutR = tOutP = tOutY = 0; tSat = 0;
      continue;
    }
    float target = throttlePct * MAX_PCT / 100.0f;
    if (target < IDLE_PCT) target = IDLE_PCT;
    if (base < target) base = fminf(base + 1.5f, target);
    else               base = fmaxf(base - 3.0f, target);

    if (hopStart == 0 && base > IDLE_PCT + 3.0f) hopStart = millis();
    if (HOP_LIMIT_MS > 0 && hopStart && millis() - hopStart > HOP_LIMIT_MS) {
      armed = false; base = 0; iR = iP = iY = 0; hopStart = 0;
      setAll(0);
      continue;
    }

    float kpa = KP_ANGLE;
    float rSP = kpa * (0.0f - relRoll);
    float pSP = kpa * (0.0f - relPitch);
    rSP = constrain(rSP, -MAX_RATE_DPS, MAX_RATE_DPS);
    pSP = constrain(pSP, -MAX_RATE_DPS, MAX_RATE_DPS);

    float eR = fR - rSP;
    float eP = fP - pSP;
    float eY = 0.0f - fY;

    float kpr = KP_RATE, kir = KI_RATE, kdr = KD_RATE;
    float kpy = KP_YAW,  kiy = KI_YAW;

    if (base > I_ENABLE_PCT) {
      iR = constrain(iR + eR * dt, -I_RATE_LIMIT, I_RATE_LIMIT);
      iP = constrain(iP + eP * dt, -I_RATE_LIMIT, I_RATE_LIMIT);
      iY = constrain(iY + eY * dt, -I_RATE_LIMIT, I_RATE_LIMIT);
    } else {
      iR = iP = iY = 0;
    }

    float dRawR = (fR - prevR) / dt;
    float dRawP = (fP - prevP) / dt;
    prevR = fR; prevP = fP;
    fD = pt1(dRawR, fD, DTERM_LPF_HZ, dt);
    static float fDP = 0;
    fDP = pt1(dRawP, fDP, DTERM_LPF_HZ, dt);

    float oR = constrain(kpr * eR + kir * iR + kdr * fD,  -MAX_CORR_PCT, MAX_CORR_PCT);
    float oP = constrain(kpr * eP + kir * iP + kdr * fDP, -MAX_CORR_PCT, MAX_CORR_PCT);
    float oY = constrain(kpy * eY + kiy * iY,             -MAX_YAW_PCT,  MAX_YAW_PCT);

    if (tSat > 1.0f) {
      iR *= 0.98f; iP *= 0.98f; iY *= 0.98f;
    }

    tOutR = oR; tOutP = oP; tOutY = oY;

    mixAndWrite(base, oR, oP, oY, true);

    int li = logIdx;
    logBuf[li][0] = relRoll; logBuf[li][1] = relPitch;
    logBuf[li][2] = rSP;     logBuf[li][3] = fR;
    logBuf[li][4] = oR;      logBuf[li][5] = base;
    logIdx = (li + 1) % LOG_DEPTH;
  }
}

static const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head><meta name=viewport content="width=device-width,initial-scale=1">
<style>
body{font-family:system-ui,sans-serif;background:#101014;color:#e8e8ea;text-align:center;padding:12px;margin:0}
.k{width:100%;height:84px;font-size:28px;font-weight:700;background:#c0392b;color:#fff;border:0;border-radius:14px;margin-bottom:10px}
#sw{width:132px;height:66px;border-radius:33px;background:#3a3a42;margin:8px auto;position:relative;transition:.15s}
#sw div{width:56px;height:56px;border-radius:50%;background:#fff;position:absolute;top:5px;left:5px;transition:.15s}
#sw.on{background:#27ae60}#sw.on div{left:71px}
input[type=range]{width:94%;height:56px}
input[type=number]{width:64px;font-size:15px;background:#1e1e24;color:#eee;border:1px solid #444;border-radius:6px;padding:4px}
button{font-size:15px;padding:8px 10px;border-radius:8px;border:1px solid #444;background:#23232a;color:#eee;margin:2px}
#st{font-size:17px;font-weight:600}
.mono{font-family:ui-monospace,monospace;font-size:12px;line-height:1.6;color:#9fe6b0;white-space:pre-wrap;text-align:left;background:#000;padding:8px;border-radius:8px;margin:8px 0}
h4{margin:14px 0 4px;color:#8ab4f8;font-size:14px;font-weight:600}
</style></head><body>
<h3 style="margin:4px 0">Micro Quad v3</h3>
<button class=k onclick=kill()>KILL</button>
<button class=k style="height:54px;font-size:18px;background:#2c3e50" onclick=clr()>RESET KILL</button>
<div id=sw onclick=tog()><div></div></div>
<p id=st>OFF</p>
<p style="margin:2px">Throttle <b id=tv>0</b>%</p>
<input id=th type=range min=0 max=100 value=0 oninput=setT(this.value)>
<div class=mono id=im>waiting...</div>

<h4>Angles</h4>
P <input id=kpa type=number step=0.5 value=4>
<h4>Rate loop</h4>
P <input id=kpr type=number step=0.01 value=0.15>
I <input id=kir type=number step=0.02 value=0.10>
D <input id=kdr type=number step=0.005 value=0>
<h4>Yaw</h4>
P <input id=kpy type=number step=0.05 value=0.25>
I <input id=kiy type=number step=0.05 value=0.20>
<button onclick=yawflip()>Flip yaw dir</button>
<br><button onclick=setG()>Set gains</button>

<h4>Trim / level</h4>
<button onclick=nt(-1,0)>R-</button> <button onclick=nt(1,0)>R+</button>
<button onclick=nt(0,-1)>P-</button> <button onclick=nt(0,1)>P+</button>
<button onclick=tz()>Trim 0</button>
<br>R <input id=trr type=number step=0.1 value=0>
P <input id=trp type=number step=0.1 value=0>
<button onclick=setTrim()>Set trim</button>
<br><button onclick=level()>LEVEL (flat &amp; still)</button>
<button onclick=save()>Save to flash</button>
<button onclick=dl()>Dump log</button>

<h4>Bench (props OFF)</h4>
<button onclick=bench(1)>Bench ON</button>
<button onclick=bench(0)>Bench OFF</button>
<br>
<button onclick=mt(0)>M1</button><button onclick=mt(1)>M2</button>
<button onclick=mt(2)>M3</button><button onclick=mt(3)>M4</button>
Duty <input id=mtd type=number step=1 value=18>

<script>
let on=false,tq=0,lastT=0,pend=false;
const $=i=>document.getElementById(i);
window.kill=()=>{tq=0;pend=false;on=false;$('sw').className='';$('st').textContent='KILLED';
 $('th').value=0;$('tv').textContent=0;
 for(let i=0;i<3;i++)fetch('/off').catch(()=>{});};
window.clr=()=>fetch('/clear').then(r=>r.text()).then(t=>{
 $('st').textContent=(t=='ok'?'KILL CLEARED':'SET THROTTLE TO 0');
}).catch(()=>{});
window.tog=()=>{
 if(on){on=false;$('sw').className='';$('st').textContent='OFF';fetch('/off').catch(()=>{});return;}
 fetch('/on').then(r=>r.text()).then(t=>{
   if(t=='ok'){on=true;$('sw').className='on';$('st').textContent='ARMED';}
   else $('st').textContent=({tilt:'NOT LEVEL',notlevel:'PRESS LEVEL ON A FLAT SURFACE',noimu:'IMU ERROR',killed:'KILL LATCHED - press RESET KILL',bench:'BENCH IS ON'}[t]||t);
 }).catch(()=>$('st').textContent='LINK LOST');
};
window.setT=v=>{
 $('tv').textContent=v;
 const n=Date.now();
 if(on&&n-lastT<120){tq=v;pend=true;return;}
 lastT=n;tq=v;pend=false;
 if(on)fetch('/t?v='+v).catch(()=>{});
};
setInterval(()=>{if(pend&&on){pend=false;lastT=Date.now();fetch('/t?v='+tq).catch(()=>{});}},140);
const num=(i,d)=>{const v=parseFloat($(i).value);return isNaN(v)?d:v;};
window.setG=()=>fetch('/g?kpa='+num('kpa',4)+'&kpr='+num('kpr',0.15)+'&kir='+num('kir',0.1)+'&kdr='+num('kdr',0)+'&kpy='+num('kpy',0.25)+'&kiy='+num('kiy',0.2)).catch(()=>{});
window.yawflip=()=>fetch('/yawflip').catch(()=>{});

const syncTrim=t=>{const a=t.split(',');if(a.length<2)return;$('trr').value=a[0];$('trp').value=a[1];};
window.nt=(r,p)=>{
 let q=[];
 if(r!==null&&r!==undefined)q.push('r='+r);
 if(p!==null&&p!==undefined)q.push('p='+p);
 if(!q.length)q.push('r=0','p=0');
 fetch('/nt?'+q.join('&')).then(r=>r.text()).then(syncTrim).catch(()=>{});
};
window.tz=()=>fetch('/tz').then(r=>r.text()).then(syncTrim).catch(()=>{});
window.setTrim=()=>fetch('/ts?r='+num('trr',0)+'&p='+num('trp',0)).then(r=>r.text()).then(syncTrim).catch(()=>{});
window.level=()=>fetch('/level').then(()=>{
 $('st').textContent='LEVELED';
 setTimeout(()=>fetch('/ts').then(r=>r.text()).then(syncTrim).catch(()=>{}),700);
}).catch(()=>{});

window.save=()=>fetch('/save').catch(()=>{});
window.dl=()=>fetch('/log').then(()=>{$('st').textContent='LOG -> SERIAL';}).catch(()=>{});
window.bench=v=>fetch('/bench?on='+v).catch(()=>{});
window.mt=i=>fetch('/mt?i='+i+'&d='+num('mtd',18)).catch(()=>{});
window.addEventListener('pagehide',()=>{on=false;fetch('/off').catch(()=>{});});
fetch('/gs').then(r=>r.text()).then(t=>{const g=t.split(',');$('kpa').value=g[0];$('kpr').value=g[1];$('kir').value=g[2];$('kdr').value=g[3];$('kpy').value=g[4];$('kiy').value=g[5];}).catch(()=>{});
fetch('/ts').then(r=>r.text()).then(syncTrim).catch(()=>{});
setInterval(()=>{
 fetch('/hb').then(r=>r.text()).then(t=>{
  const a=t.split(',');
  if(on&&a[1]=='1'){on=false;$('sw').className='';$('st').textContent='KILLED';}
  else if(on&&a[0]=='0'){on=false;$('sw').className='';$('st').textContent='DISARMED';}
  $('im').textContent='R '+a[2]+'   P '+a[3]+'\n'+
   'rR '+a[4]+'   rP '+a[5]+'   Y '+a[6]+' dps\n'+
   'M '+a[9]+' '+a[10]+' '+a[11]+' '+a[12]+'   SAT '+a[7]+'%\n'+
   'trim '+a[13]+' / '+a[14]+'   '+a[15]+' Hz\n'+
   (a[18]=='1'?'alt '+a[16]+' m   vz '+a[17]+' m/s':'baro: not detected');
 }).catch(()=>{});
},220);
</script></body></html>)HTML";

static void loadSettings() {
  prefs.begin("quad", false);
  KP_ANGLE = prefs.getFloat("kpa", KP_ANGLE);
  KP_RATE  = prefs.getFloat("kpr", KP_RATE);
  KI_RATE  = prefs.getFloat("kir", KI_RATE);
  KD_RATE  = prefs.getFloat("kdr", KD_RATE);
  KP_YAW   = prefs.getFloat("kpy", KP_YAW);
  KI_YAW   = prefs.getFloat("kiy", KI_YAW);
  YAW_SIGN = prefs.getFloat("yaw", YAW_SIGN);
  trimRoll  = prefs.getFloat("tr", 0);
  trimPitch = prefs.getFloat("tp", 0);
  prefs.end();
}

static void saveSettings() {
  prefs.begin("quad", false);
  prefs.putFloat("kpa", KP_ANGLE); prefs.putFloat("kpr", KP_RATE);
  prefs.putFloat("kir", KI_RATE);  prefs.putFloat("kdr", KD_RATE);
  prefs.putFloat("kpy", KP_YAW);   prefs.putFloat("kiy", KI_YAW);
  prefs.putFloat("yaw", YAW_SIGN);
  prefs.putFloat("tr", trimRoll);  prefs.putFloat("tp", trimPitch);
  prefs.end();
}

static void sendTrim() {
  char b[32];
  snprintf(b, sizeof(b), "%.2f,%.2f", (double)trimRoll, (double)trimPitch);
  server.send(200, "text/plain", b);
}

static void setupWeb() {
  server.on("/", []() {
    pageHits++;
    if (pageHits <= 3) Serial.printf("[web] GET / from %s\n",
                                     server.client().remoteIP().toString().c_str());
    server.send_P(200, "text/html", PAGE);
  });

  server.on("/ping", []() {
    server.send(200, "text/plain", "pong");
  });

  server.onNotFound([]() {
    notFound++;
    Serial.printf("[web] 404 %s from %s\n", server.uri().c_str(),
                  server.client().remoteIP().toString().c_str());
    server.send(404, "text/plain", "no");
  });

  server.on("/on", []() {
    if (!imuOK) { server.send(200, "text/plain", "noimu"); return; }
    if (killLatch) { server.send(200, "text/plain", "killed"); return; }
    if (benchMode) { server.send(200, "text/plain", "bench"); return; }
    if (!levelValid) { server.send(200, "text/plain", "notlevel"); return; }
    if (fabsf(tRoll) > ARM_MAX_TILT || fabsf(tPitch) > ARM_MAX_TILT) {
      server.send(200, "text/plain", "tilt"); return;
    }
    throttlePct = 0;
    lastBeat = millis();
    armMillis = millis();
    armed = true;
    server.send(200, "text/plain", "ok");
  });

  server.on("/off", []() {
    armed = false; throttlePct = 0;
    server.send(200, "text/plain", "ok");
  });

  server.on("/clear", []() {
    if (throttlePct > 0.5f) { server.send(200, "text/plain", "throttle"); return; }
    armed = false;
    killLatch = false;
    server.send(200, "text/plain", "ok");
  });

  server.on("/t", []() {
    if (server.hasArg("v")) {
      int v = constrain(server.arg("v").toInt(), 0, 100);
      if (armed) throttlePct = v;
      lastBeat = millis();
    }
    server.send(200, "text/plain", "ok");
  });

  server.on("/hb", []() {
    hbCount++;
    lastBeat = millis();
    char buf[230];
    snprintf(buf, sizeof(buf),
             "%d,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%.2f,%.2f,%.0f,%.2f,%.2f,%d",
             armed ? 1 : 0, killLatch ? 1 : 0,
             (double)tRoll, (double)tPitch,
             (double)tRollRate, (double)tPitchRate, (double)tYawRate,
             (double)tSat, (int)throttlePct,
             tM[0], tM[1], tM[2], tM[3],
             (double)trimRoll, (double)trimPitch, (double)tLoopHz,
             (double)tAlt, (double)tVz, baroOK ? 1 : 0);
    server.send(200, "text/plain", buf);
  });

  server.on("/gs", []() {
    char buf[96];
    snprintf(buf, sizeof(buf), "%.2f,%.3f,%.3f,%.4f,%.2f,%.2f",
             (double)KP_ANGLE, (double)KP_RATE, (double)KI_RATE,
             (double)KD_RATE, (double)KP_YAW, (double)KI_YAW);
    server.send(200, "text/plain", buf);
  });

  server.on("/g", []() {
    if (server.hasArg("kpa")) KP_ANGLE = constrain(server.arg("kpa").toFloat(), 0.0f, 20.0f);
    if (server.hasArg("kpr")) KP_RATE  = constrain(server.arg("kpr").toFloat(), 0.0f, 2.0f);
    if (server.hasArg("kir")) KI_RATE  = constrain(server.arg("kir").toFloat(), 0.0f, 2.0f);
    if (server.hasArg("kdr")) KD_RATE  = constrain(server.arg("kdr").toFloat(), 0.0f, 1.0f);
    if (server.hasArg("kpy")) KP_YAW   = constrain(server.arg("kpy").toFloat(), 0.0f, 3.0f);
    if (server.hasArg("kiy")) KI_YAW   = constrain(server.arg("kiy").toFloat(), 0.0f, 3.0f);
    server.send(200, "text/plain", "ok");
  });

  server.on("/baro", []() {
    char buf[96];
    snprintf(buf, sizeof(buf), "%d,0x%02X,%.1f,%.2f,%.3f,%.3f",
             baroOK ? 1 : 0, BMP, (double)tBaroP, (double)tBaroT,
             (double)tAlt, (double)tVz);
    server.send(200, "text/plain", buf);
  });

  server.on("/yawflip", []() {
    YAW_SIGN = -YAW_SIGN;
    server.send(200, "text/plain", YAW_SIGN > 0 ? "+" : "-");
  });

  server.on("/nt", []() {
    if (server.hasArg("r")) trimRoll  = trimRoll  + server.arg("r").toFloat();
    if (server.hasArg("p")) trimPitch = trimPitch + server.arg("p").toFloat();
    trimRoll  = constrain((float)trimRoll,  -TRIM_LIMIT_DEG, TRIM_LIMIT_DEG);
    trimPitch = constrain((float)trimPitch, -TRIM_LIMIT_DEG, TRIM_LIMIT_DEG);
    sendTrim();
  });

  server.on("/ts", []() {
    if (server.hasArg("r")) trimRoll  = constrain(server.arg("r").toFloat(), -TRIM_LIMIT_DEG, TRIM_LIMIT_DEG);
    if (server.hasArg("p")) trimPitch = constrain(server.arg("p").toFloat(), -TRIM_LIMIT_DEG, TRIM_LIMIT_DEG);
    sendTrim();
  });

  server.on("/tz", []() {
    trimRoll = trimPitch = 0;
    sendTrim();
  });

  server.on("/level", []() {
    relevelReq = true;
    server.send(200, "text/plain", "ok");
  });

    server.on("/log", []() {
    String s;
    s.reserve(LOG_DEPTH * 32 + 64);
    s += "roll,pitch,rateSP,rollRate,outR,base\n";
    char line[64];
    for (int n = 0; n < LOG_DEPTH; n++) {
      int i = (logIdx + n) % LOG_DEPTH;
      snprintf(line, sizeof(line), "%.2f,%.2f,%.1f,%.1f,%.2f,%.1f\n",
               (double)logBuf[i][0], (double)logBuf[i][1], (double)logBuf[i][2],
               (double)logBuf[i][3], (double)logBuf[i][4], (double)logBuf[i][5]);
      s += line;
    }
    server.send(200, "text/plain", s);
  });

  server.on("/bench", []() {
    if (server.hasArg("on")) {
      benchMode = server.arg("on").toInt() != 0;
      armed = false;
    }
    server.send(200, "text/plain", benchMode ? "1" : "0");
  });

  server.on("/mt", []() {
    if (armed) { server.send(200, "text/plain", "armed"); return; }
    int i = constrain(server.arg("i").toInt(), 0, 3);
    testMotor = i;
    testMotorPct = server.hasArg("d")
                     ? constrain(server.arg("d").toFloat(), 0.0f, 40.0f)
                     : 18.0f;
    testUntil = millis() + 1500;
    server.send(200, "text/plain", "ok");
  });

  server.on("/save", []() {
    saveSettings();
    server.send(200, "text/plain", "ok");
  });

  server.on("/diag", []() {
    char buf[420];
    snprintf(buf, sizeof(buf),
             "reset=%d imu=%d sta=%d page=%lu hb=%lu 404=%lu ctlHz=%.0f webHz=%.0f "
             "heap=%u minHeap=%u ap=%s rssi_ch=%d gyrobias=%.1f,%.1f,%.1f lvl=%d sat=%.1f",
             tResetReason, imuOK ? 1 : 0, WiFi.softAPgetStationNum(),
             pageHits, hbCount, notFound,
             (double)tLoopHz, (double)tWebHz,
             (unsigned)ESP.getFreeHeap(), (unsigned)tMinHeap,
             WiFi.softAPIP().toString().c_str(), WiFi.channel(),
             (double)(gBias[0] / GYRO_LSB), (double)(gBias[1] / GYRO_LSB),
             (double)(gBias[2] / GYRO_LSB),
             levelValid ? 1 : 0, (double)tSat);
    server.send(200, "text/plain", buf);
  });

  server.begin();
}

static void webTask(void *) {
  uint32_t n = 0;
  uint32_t t0 = millis();
  for (;;) {
    server.handleClient();
    n++;
    if (millis() - t0 >= 500) {
      tWebHz = n * 1000.0f / (millis() - t0);
      n = 0; t0 = millis();
    }
    delay(1);
  }
}

void setup() {
  for (int i = 0; i < 4; i++) { pinMode(PINS[i], OUTPUT); digitalWrite(PINS[i], LOW); }
  pwmInit();
  setAll(0);

#if USE_BOOT_KILL
  pinMode(0, INPUT_PULLUP);
#endif

  Serial.begin(115200);
  delay(150);
  tResetReason = (int)esp_reset_reason();
  Serial.printf("\nMicro Quad v3 | reset reason %d (1=PWRON 3=SW 4=PANIC 6=TASK_WDT 9=BROWN_OUT)\n",
                tResetReason);

  Wire.begin(21, 22);
  Wire.setClock(400000);
  Wire.setTimeOut(20);

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  IPAddress apIP(192, 168, 4, 1);
  IPAddress mask(255, 255, 255, 0);
  WiFi.softAPConfig(apIP, apIP, mask);
  bool apOK = WiFi.softAP(AP_SSID, AP_PASS, 6, 0, 4);
  Serial.printf("AP ok=%d ssid=%s ip=%s ch=%d\n",
                apOK ? 1 : 0, AP_SSID,
                WiFi.softAPIP().toString().c_str(), WiFi.channel());

  setupWeb();
  xTaskCreatePinnedToCore(webTask, "web", 8192, NULL, 2, NULL, 0);
  Serial.printf("WEB READY -> http://%s/\n", WiFi.softAPIP().toString().c_str());

  imuOK = imuInit();
  if (imuOK) {
    loadSettings();
    if (!calibrateImu()) imuOK = false;
  }
  Serial.println(imuOK ? "IMU OK" : "IMU FAILED");

  baroOK = baroInit();
  Serial.println(baroOK ? "BARO OK" : "BARO NOT FOUND (continuing without it)");

  xTaskCreatePinnedToCore(controlTask, "ctl", 8192, NULL, 4, NULL, 1);
}

void loop() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 250) {
    lastPrint = millis();
    Serial.printf("R %6.1f P %6.1f | rR %6.1f rP %6.1f Y %6.1f | M %3d %3d %3d %3d | sat %4.1f | alt %5.2f vz %5.2f | %s\n",
                  (double)tRoll, (double)tPitch,
                  (double)tRollRate, (double)tPitchRate, (double)tYawRate,
                  tM[0], tM[1], tM[2], tM[3], (double)tSat, (double)tAlt, (double)tVz,
                  armed ? "ARMED" : (killLatch ? "KILLED" : "off"));
  }

  static unsigned long lastStat = 0;
  if (millis() - lastStat >= 3000) {
    lastStat = millis();
    uint32_t heap = ESP.getFreeHeap();
    if (tMinHeap == 0 || heap < tMinHeap) tMinHeap = heap;
    tStations = WiFi.softAPgetStationNum();
    Serial.printf("[net] sta=%d page=%lu hb=%lu 404=%lu loop=%.0fHz web=%.0fHz heap=%u\n",
                  tStations, pageHits, hbCount, notFound,
                  (double)tLoopHz, (double)tWebHz, (unsigned)heap);
  }

  delay(20);
}
