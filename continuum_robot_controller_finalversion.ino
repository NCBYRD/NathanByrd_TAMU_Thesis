#include <Arduino.h>
#include <Wire.h>
#include "MPU9250.h"
#include <math.h>

// ============================================================
// FreeRTOS Mutexes for Cross-Core Thread Safety
// ============================================================

portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t psiMutex = NULL;
SemaphoreHandle_t pwmMutex = NULL;

// ============================================================
// IMUs
// ============================================================

MPU9250 mpuTip;
MPU9250 mpuBase;

// Quaternions stored as {w, x, y, z}
float qTip[4]    = {1.0f, 0.0f, 0.0f, 0.0f};
float qBase[4]   = {1.0f, 0.0f, 0.0f, 0.0f};
float qOffset[4] = {1.0f, 0.0f, 0.0f, 0.0f};

uint32_t lastTipMicros  = 0;
uint32_t lastBaseMicros = 0;

const float pi = 3.14159265f;

// ============================================================
// Pins
// ============================================================

const int blueLEDPin = 2;
#define M1_POS 4
#define M1_NEG 12
#define M2_POS 26
#define M2_NEG 25
#define M3_POS 13
#define M3_NEG 27

// Dual I2C Bus Pins
#define BASE_SDA_PIN 21
#define BASE_SCL_PIN 22
#define TIP_SDA_PIN 16
#define TIP_SCL_PIN 17

#define ENC1_A 35
#define ENC1_B 34
#define ENC2_A 33
#define ENC2_B 32
#define ENC3_A 39
#define ENC3_B 36

#define STBY_A 23
#define STBY_B 14

// ============================================================
// Structs
// ============================================================

typedef struct
{
  float phi;
  float theta;
} Psi;

typedef struct
{
  float phi_desired;
  float theta_desired;
} Psi_desired;

// ============================================================
// Globals
// ============================================================

volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;
volatile long encoderCount3 = 0;

Psi psi = {0.0001f, 0.0001f};
Psi_desired psi_desired = {0.0f, 0.0f};

float pwm_output[3] = {0.0f, 0.0f, 0.0f};

// PID variables kept for Simulink serial packet compatibility
float ep_prev = 0.0f;
float ep_integral = 0.0f;
float et_prev = 0.0f;
float et_integral = 0.0f;

volatile float Kp_phi = 0.0f;
volatile float Ki_phi = 0.0f;
volatile float Kd_phi = 0.0f;

volatile float Kp_theta = 0.0f;
volatile float Ki_theta = 0.0f;
volatile float Kd_theta = 0.0f;

volatile float INTEGRAL_CLAMP = 0.0f;

// Feedforward gain used by open-loop controller
volatile float K_FEEDFORWARD = 1.0f;

volatile bool serialDataReceived = false;

// ============================================================
// Robot constants
// ============================================================

const float TENDON_R = 0.02f;
const float DEAD_BAND_LENGTH = 0.01f;

const float L0[3] = {0.10f, 0.10f, 0.10f};

const float PWM_MAX = 255.0f;

const long ENCODER_MAX[3] = {0L, 0L, 0L};

// ============================================================
// Quaternion observer parameters
// ============================================================

float sx_f = 0.0f;
float sy_f = 0.0f;

const float swingAlpha = 0.35f;
bool swingFilterPrimed = false;
float theta_rel_held = 0.0f;

const float PHI_GATE_THRESHOLD = 0.05f;

// Debounce / decay for theta latching. theta is an angle on a near-zero-
// length swing vector when phi is small, so it's poorly conditioned right
// at the gate. These guard against a single noisy cycle latching a bogus
// theta_rel_held that then freezes forever once phi drops back down.
const int      THETA_LATCH_CONSECUTIVE = 5;    // consecutive above-gate cycles required before trusting theta (~100ms at 20ms loop)
const uint32_t THETA_RESET_TIMEOUT_MS  = 300;  // sustained below-gate time before theta_rel_held resets to 0

float TILT_KP = 3.0f;
const float RUNTIME_TILT_KP = 0.6f;
const float TILT_KI = 0.002f;

const float ACC_GATE_TOLERANCE = 0.20f;
const float GYRO_GATE_THRESHOLD = 0.50f;

float tipBias[3]  = {0.0f, 0.0f, 0.0f};
float baseBias[3] = {0.0f, 0.0f, 0.0f};

float tipStaticBias[3]  = {0.0f, 0.0f, 0.0f};
float baseStaticBias[3] = {0.0f, 0.0f, 0.0f};

// ============================================================
// Serial float structures
// ============================================================

typedef union
{
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

// Incoming from Simulink
FLOATUNION_t myValue1, myValue2;
FLOATUNION_t Kp_Phi, Kp_Theta;
FLOATUNION_t Ki_Phi, Ki_Theta;
FLOATUNION_t Kd_Phi, Kd_Theta;
FLOATUNION_t integral_clamp, k_ff_val;

// Outgoing to Simulink
FLOATUNION_t phiVal, thetaVal;
FLOATUNION_t phi_error, theta_error;
FLOATUNION_t pwm1_output_u, pwm2_output_u, pwm3_output_u;
FLOATUNION_t e1, e2, e3;
FLOATUNION_t dL1, dL2, dL3;
FLOATUNION_t phi_u, theta_u;

// ============================================================
// Helper Functions
// ============================================================

float getFloatFromSerial()
{
  FLOATUNION_t f;
  for (int i = 0; i < 4; i++)
  {
    while (!Serial.available())
    {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    f.bytes[i] = Serial.read();
  }
  return f.number;
}

void setMotorPWM(int posPin, int negPin, float pwm, long enc_cnt)
{
  float mag = fabs(pwm);

  if (pwm < 0.0f)
  {
    // Winding
    analogWrite(posPin, (int)constrain(mag, 0.0f, PWM_MAX));
    analogWrite(negPin, 0);
  }
  else
  {
    // Unwinding scaled (clamped to prevent >255 overflow)
    analogWrite(posPin, 0);
    analogWrite(negPin, (int)constrain(mag * 2.5f, 0.0f, PWM_MAX));
  }
}

// ============================================================
// Encoder ISRs
// ============================================================

void IRAM_ATTR encoder1_ISR()
{
  portENTER_CRITICAL_ISR(&encMux);
  if (digitalRead(ENC1_A) == digitalRead(ENC1_B))
    encoderCount1++;
  else
    encoderCount1--;
  portEXIT_CRITICAL_ISR(&encMux);
}

void IRAM_ATTR encoder2_ISR()
{
  portENTER_CRITICAL_ISR(&encMux);
  if (digitalRead(ENC2_A) == digitalRead(ENC2_B))
    encoderCount2++;
  else
    encoderCount2--;
  portEXIT_CRITICAL_ISR(&encMux);
}

void IRAM_ATTR encoder3_ISR()
{
  portENTER_CRITICAL_ISR(&encMux);
  if (digitalRead(ENC3_A) == digitalRead(ENC3_B))
    encoderCount3++;
  else
    encoderCount3--;
  portEXIT_CRITICAL_ISR(&encMux);
}

void resetEncoders()
{
  portENTER_CRITICAL(&encMux);
  encoderCount1 = 0;
  encoderCount2 = 0;
  encoderCount3 = 0;
  portEXIT_CRITICAL(&encMux);
  Serial.println("Encoders reset.");
}

// ============================================================
// Kinematics & Quaternion Math
// ============================================================

void computeJacobian(float phi, float theta, float J[3][2])
{
  const float r = TENDON_R;
  const float epsilon = 1e-4f;
  float safe_phi = phi;

  if (fabs(phi) < epsilon)
  {
    safe_phi = (phi >= 0.0f) ? epsilon : -epsilon;
  }

  // Column 1: dL / dphi
  J[0][0] = -r * cos(theta);
  J[1][0] = -r * (sqrt(3.0f) * sin(theta) - cos(theta)) / 2.0f;
  J[2][0] =  r * (sqrt(3.0f) * sin(theta) + cos(theta)) / 2.0f;

  // Column 2: dL / dtheta
  J[0][1] = safe_phi * r * sin(theta);
  J[1][1] = -safe_phi * r * (sqrt(3.0f) * cos(theta) + sin(theta)) / 2.0f;
  J[2][1] =  safe_phi * r * (sqrt(3.0f) * cos(theta) - sin(theta)) / 2.0f;
}

void quatIntegrate(float q[4], float gx, float gy, float gz, float dt)
{
  float w = q[0], x = q[1], y = q[2], z = q[3];

  float dw = 0.5f * (-x * gx - y * gy - z * gz);
  float dx = 0.5f * ( w * gx + y * gz - z * gy);
  float dy = 0.5f * ( w * gy - x * gz + z * gx);
  float dz = 0.5f * ( w * gz + x * gy - y * gx);

  q[0] += dw * dt;
  q[1] += dx * dt;
  q[2] += dy * dt;
  q[3] += dz * dt;

  float norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (norm > 1e-6f)
  {
    float invNorm = 1.0f / norm;
    q[0] *= invNorm;
    q[1] *= invNorm;
    q[2] *= invNorm;
    q[3] *= invNorm;
  }
}

void quatConjMul(const float a[4], const float b[4], float out[4])
{
  float aw =  a[0], ax = -a[1], ay = -a[2], az = -a[3];
  float bw =  b[0], bx =  b[1], by =  b[2], bz =  b[3];

  out[0] = aw * bw - ax * bx - ay * by - az * bz;
  out[1] = aw * bx + ax * bw + ay * bz - az * by;
  out[2] = aw * by - ax * bz + ay * bw + az * bx;
  out[3] = aw * bz + ax * by - ay * bx + az * bw;
}

void applyTiltCorrection(float q[4], float ax, float ay, float az, float gx, float gy, float gz, float dt, float bias[3])
{
  float gyroMag = sqrt(gx * gx + gy * gy + gz * gz);
  if (gyroMag > GYRO_GATE_THRESHOLD) return;

  float accMag = sqrt(ax * ax + ay * ay + az * az);
  if (accMag < 1e-6f || fabs(accMag - 1.0f) > ACC_GATE_TOLERANCE) return;

  float invMag = 1.0f / accMag;
  ax *= invMag; ay *= invMag; az *= invMag;

  float vx = 2.0f * (q[1] * q[3] - q[0] * q[2]);
  float vy = 2.0f * (q[0] * q[1] + q[2] * q[3]);
  float vz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

  float ex = (ay * vz - az * vy);
  float ey = (az * vx - ax * vz);
  float ez = (ax * vy - ay * vx);

  bias[0] += TILT_KI * ex * dt;
  bias[1] += TILT_KI * ey * dt;
  bias[2] += TILT_KI * ez * dt;

  float cwx = TILT_KP * ex + bias[0];
  float cwy = TILT_KP * ey + bias[1];
  float cwz = TILT_KP * ez + bias[2];

  quatIntegrate(q, cwx, cwy, cwz, dt);
}

void readAndIntegrate(MPU9250 &mpu, float q[4], uint32_t &lastMicros, float bias[3], const float staticBias[3])
{
  float gx = mpu.getGyroX() * pi / 180.0f - staticBias[0];
  float gy = mpu.getGyroY() * pi / 180.0f - staticBias[1];
  float gz = mpu.getGyroZ() * pi / 180.0f - staticBias[2];

  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();

  uint32_t now_us = micros();
  float dt = (lastMicros == 0) ? 0.01f : (now_us - lastMicros) / 1.0e6f;
  lastMicros = now_us;

  if (dt <= 0.0f || dt > 0.2f) dt = 0.01f;

  quatIntegrate(q, gx, gy, gz, dt);
  applyTiltCorrection(q, ax, ay, az, gx, gy, gz, dt, bias);
}

// ============================================================
// Static gyro bias calibration
// ============================================================
// Averages raw gyro output (converted to rad/s) from both IMUs while the
// module is assumed stationary, and stores the result into
// tipStaticBias / baseStaticBias. These are subtracted from every gyro
// sample in readAndIntegrate(). Without this step both arrays stay at
// {0,0,0} forever and each IMU's independent resting gyro bias (which does
// NOT cancel between qBase and qTip, since it's not a shared/rigid-body
// rotation) integrates into drift over time -- most noticeably about the
// yaw axis, which the accelerometer-based tilt correction cannot observe.
void calibrateStaticGyroBias()
{
  Serial.println("[IMU] Calibrating residual static gyro bias - keep module still...");

  const uint32_t CAL_DURATION_MS = 3000;

  double sumTipX = 0.0, sumTipY = 0.0, sumTipZ = 0.0;
  double sumBaseX = 0.0, sumBaseY = 0.0, sumBaseZ = 0.0;
  uint32_t tipSamples = 0, baseSamples = 0;

  uint32_t start = millis();
  while (millis() - start < CAL_DURATION_MS)
  {
    if (mpuTip.update())
    {
      sumTipX += mpuTip.getGyroX() * pi / 180.0;
      sumTipY += mpuTip.getGyroY() * pi / 180.0;
      sumTipZ += mpuTip.getGyroZ() * pi / 180.0;
      tipSamples++;
    }

    if (mpuBase.update())
    {
      sumBaseX += mpuBase.getGyroX() * pi / 180.0;
      sumBaseY += mpuBase.getGyroY() * pi / 180.0;
      sumBaseZ += mpuBase.getGyroZ() * pi / 180.0;
      baseSamples++;
    }

    delay(2);
  }

  if (tipSamples > 0)
  {
    tipStaticBias[0] = (float)(sumTipX / tipSamples);
    tipStaticBias[1] = (float)(sumTipY / tipSamples);
    tipStaticBias[2] = (float)(sumTipZ / tipSamples);
  }
  else
  {
    Serial.println("[IMU] WARNING: no tip gyro samples collected during calibration.");
  }

  if (baseSamples > 0)
  {
    baseStaticBias[0] = (float)(sumBaseX / baseSamples);
    baseStaticBias[1] = (float)(sumBaseY / baseSamples);
    baseStaticBias[2] = (float)(sumBaseZ / baseSamples);
  }
  else
  {
    Serial.println("[IMU] WARNING: no base gyro samples collected during calibration.");
  }

  Serial.print("[IMU] Tip static gyro bias (rad/s): ");
  Serial.print(tipStaticBias[0], 6); Serial.print(", ");
  Serial.print(tipStaticBias[1], 6); Serial.print(", ");
  Serial.println(tipStaticBias[2], 6);

  Serial.print("[IMU] Base static gyro bias (rad/s): ");
  Serial.print(baseStaticBias[0], 6); Serial.print(", ");
  Serial.print(baseStaticBias[1], 6); Serial.print(", ");
  Serial.println(baseStaticBias[2], 6);

  // Reset integration timestamps so the dt used right after calibration
  // isn't inflated by the time spent sitting in this loop.
  lastTipMicros = 0;
  lastBaseMicros = 0;
}

void captureOffset()
{
  float qRel_raw[4];
  quatConjMul(qBase, qTip, qRel_raw);

  qOffset[0] = qRel_raw[0];
  qOffset[1] = qRel_raw[1];
  qOffset[2] = qRel_raw[2];
  qOffset[3] = qRel_raw[3];

  for (int i = 0; i < 3; i++)
  {
    tipBias[i] = 0.0f;
    baseBias[i] = 0.0f;
  }

  swingFilterPrimed = false;
  theta_rel_held = 0.0f;
}

void warmupAndCaptureOffset()
{
  TILT_KP = 3.0f;

  float qTipPrev[4]  = {qTip[0], qTip[1], qTip[2], qTip[3]};
  float qBasePrev[4] = {qBase[0], qBase[1], qBase[2], qBase[3]};

  const uint32_t MIN_WARMUP_MS = 4000;
  const uint32_t MAX_WARMUP_MS = 20000;
  const float STABLE_DELTA = 0.0005f;
  const int REQUIRED_CONSECUTIVE_STABLE = 10;

  int consecutiveStable = 0;
  uint32_t settleStart = millis();
  uint32_t lastCheck = millis();

  while (true)
  {
    if (mpuTip.update())
      readAndIntegrate(mpuTip, qTip, lastTipMicros, tipBias, tipStaticBias);

    if (mpuBase.update())
      readAndIntegrate(mpuBase, qBase, lastBaseMicros, baseBias, baseStaticBias);

    delay(2);
    uint32_t elapsed = millis() - settleStart;

    if (millis() - lastCheck >= 100)
      {
      float deltaTip = fabs(qTip[0] - qTipPrev[0]) + fabs(qTip[1] - qTipPrev[1]) +
                       fabs(qTip[2] - qTipPrev[2]) + fabs(qTip[3] - qTipPrev[3]);

      float deltaBase = fabs(qBase[0] - qBasePrev[0]) + fabs(qBase[1] - qBasePrev[1]) +
                        fabs(qBase[2] - qBasePrev[2]) + fabs(qBase[3] - qBasePrev[3]);

      for (int i = 0; i < 4; i++)
      {
        qTipPrev[i] = qTip[i];
        qBasePrev[i] = qBase[i];
      }

      lastCheck = millis();

      if (elapsed >= MIN_WARMUP_MS && deltaTip < STABLE_DELTA && deltaBase < STABLE_DELTA)
        consecutiveStable++;
      else
        consecutiveStable = 0;

      if (consecutiveStable >= REQUIRED_CONSECUTIVE_STABLE || elapsed > MAX_WARMUP_MS)
        break;
    }
  }

  TILT_KP = RUNTIME_TILT_KP;
  captureOffset();
}

// ============================================================
// Tasks
// ============================================================

void TaskReadIMU(void *pvParameters)
{
  (void)pvParameters;
  static uint32_t prev_ms = 0;

  for (;;)
  {
    if (mpuTip.update())
      readAndIntegrate(mpuTip, qTip, lastTipMicros, tipBias, tipStaticBias);

    if (mpuBase.update())
      readAndIntegrate(mpuBase, qBase, lastBaseMicros, baseBias, baseStaticBias);

    uint32_t now = millis();

    if (now - prev_ms >= 20)
    {
      prev_ms = now;

      float qRel_raw[4], qRel[4];
      quatConjMul(qBase, qTip, qRel_raw);
      quatConjMul(qOffset, qRel_raw, qRel);

      float w = qRel[0], x = qRel[1], y = qRel[2], z = qRel[3];
      float twistNorm = sqrt(w * w + z * z);
      float ws, xs, ys;

      if (twistNorm > 1e-6f)
      {
        float invTwist = 1.0f / twistNorm;
        ws = twistNorm;
        xs = (w * x - y * z) * invTwist;
        ys = (w * y + x * z) * invTwist;
      }
      else
      {
        ws = 1.0f; xs = 0.0f; ys = 0.0f;
      }

      float phi_raw_instant = 2.0f * atan2(sqrt(xs * xs + ys * ys), ws);

      if (phi_raw_instant > PHI_GATE_THRESHOLD)
      {
        if (!swingFilterPrimed)
        {
          sx_f = xs; sy_f = ys;
          swingFilterPrimed = true;
        }
        else
        {
          sx_f = swingAlpha * xs + (1.0f - swingAlpha) * sx_f;
          sy_f = swingAlpha * ys + (1.0f - swingAlpha) * sy_f;
        }
      }

      float phi_rel = 2.0f * atan2(sqrt(sx_f * sx_f + sy_f * sy_f), ws);
      phi_rel = constrain(phi_rel, 0.0f, pi);

      float theta_rel_raw = -atan2(sx_f, sy_f);

      static int aboveGateCount = 0;
      static bool belowGateTimerActive = false;
      static uint32_t belowGateSinceMs = 0;

      if (phi_rel > PHI_GATE_THRESHOLD)
      {
        aboveGateCount++;
        belowGateTimerActive = false;

        // Only trust theta once we've seen sustained bend, not a single
        // noisy cycle -- avoids latching onto a transient during handling.
        if (aboveGateCount >= THETA_LATCH_CONSECUTIVE)
        {
          theta_rel_held = theta_rel_raw;
        }
      }
      else
      {
        aboveGateCount = 0;

        if (!belowGateTimerActive)
        {
          belowGateTimerActive = true;
          belowGateSinceMs = now;
        }
        else if (now - belowGateSinceMs >= THETA_RESET_TIMEOUT_MS)
        {
          // phi has been near zero for a while -- there's no meaningful
          // bend direction anymore, so stop reporting a stale latched
          // theta and let the swing filter re-prime cleanly on the next
          // real bend.
          theta_rel_held = 0.0f;
          swingFilterPrimed = false;
          sx_f = 0.0f;
          sy_f = 0.0f;
        }
      }

      if (xSemaphoreTake(psiMutex, portMAX_DELAY) == pdTRUE)
      {
        psi.phi = phi_rel;
        psi.theta = theta_rel_held;
        xSemaphoreGive(psiMutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void ControllerTask(void *pvParameters) {
  (void)pvParameters;
  const int CONTROL_PERIOD_MS = 20;
  const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
  TickType_t lastWake = xTaskGetTickCount();
  float J[3][2];

  static float phi_des_prev = 0.0f;
  static float theta_des_prev = 0.0f;
  static float theta_hold = 0.0f;

  for (;;) {
    if (!serialDataReceived) {
      ep_integral = 0.0f;
      et_integral = 0.0f;
      ep_prev = 0.0f;
      et_prev = 0.0f;
      phi_des_prev = 0.0f;
      theta_des_prev = 0.0f;

      noInterrupts();
      pwm_output[0] = pwm_output[1] = pwm_output[2] = 0.0f;
      interrupts();

      vTaskDelayUntil(&lastWake, period);
      continue;
    }

    float phi_a, theta_a, phi_d, theta_d;
    noInterrupts();
    phi_a = psi.phi;
    theta_a = psi.theta;
    phi_d = psi_desired.phi_desired;
    theta_d = psi_desired.theta_desired;
    interrupts();

    const float PHI_THRESHOLD = 1.0f;
    float theta_gain_scale = 1.0f;
    if (fabs(phi_a) < PHI_THRESHOLD) {
      theta_gain_scale = fabs(phi_a) / PHI_THRESHOLD;
    }

    float phi_err   = atan2f(sin(phi_d - phi_a), cos(phi_d - phi_a));
    float theta_err = atan2f(sin(theta_d - theta_a), cos(theta_d - theta_a));

    phi_error.number = phi_err;
    theta_error.number = theta_err;

    float dt = (float)CONTROL_PERIOD_MS / 1000.0f;
    if (dt <= 0.0f) dt = 0.02f;

    ep_integral = constrain(ep_integral + (phi_err * dt), -INTEGRAL_CLAMP, INTEGRAL_CLAMP);
    float ep_deriv = (phi_err - ep_prev) / dt;
    float u_phi_FB = Kp_phi * phi_err + Ki_phi * ep_integral + Kd_phi * ep_deriv;
    ep_prev = phi_err;

    et_integral = constrain(et_integral + (theta_err * dt * theta_gain_scale), -INTEGRAL_CLAMP, INTEGRAL_CLAMP);
    float et_deriv = (theta_err - et_prev) / dt;
    float u_theta_FB = (Kp_theta * theta_err + Ki_theta * et_integral + Kd_theta * et_deriv) * theta_gain_scale;
    et_prev = theta_err;

    const float PHI_DEADZONE = 0.14f;
    float dL[3];

    float p_dot = (phi_d - phi_des_prev);
    float t_dot = (theta_d - theta_des_prev) / dt;
    t_dot = constrain(t_dot, -5.0f, 5.0f);
    float ff = K_FEEDFORWARD;

    float theta_safe = theta_a;
    if (fabs(phi_a) < PHI_DEADZONE) {
      theta_safe = 0.0f;
    }

    if (fabs(phi_a) > PHI_DEADZONE) {
      theta_hold = theta_a;
    }

    computeJacobian(phi_a, theta_hold, J);

    if (fabs(phi_a) < PHI_DEADZONE) {
      dL[0] = (J[0][0] * u_phi_FB) + (J[0][1] * t_dot * ff);
      dL[1] = (J[1][0] * u_phi_FB) + (J[1][1] * t_dot * ff);
      dL[2] = (J[2][0] * u_phi_FB) + (J[2][1] * t_dot * ff);
      et_integral = 0.0f;
    } else {
      dL[0] = (J[0][0] * u_phi_FB + J[0][1] * u_theta_FB);
      dL[1] = (J[1][0] * u_phi_FB + J[1][1] * u_theta_FB);
      dL[2] = (J[2][0] * u_phi_FB + J[2][1] * u_theta_FB);
    }

    phi_des_prev = phi_d;
    theta_des_prev = theta_d;

    for (int i = 0; i < 3; i++) {
      float pwm = dL[i];

      long enc_cnt;
      noInterrupts();
      if(i == 0)      enc_cnt = encoderCount1;
      else if(i == 1) enc_cnt = encoderCount2;
      else            enc_cnt = encoderCount3;
      interrupts();

      if (enc_cnt >= 0 && pwm > 0) {
          pwm = 0;
      }

      if (enc_cnt <= -10000000 && pwm < 0) {
          pwm = 0;
      }

      noInterrupts();
      pwm_output[i] = constrain(pwm, -PWM_MAX, PWM_MAX);
      interrupts();
    }

    dL1.number = dL[0]; dL2.number = dL[1]; dL3.number = dL[2];
    pwm1_output_u.number = pwm_output[0];
    pwm2_output_u.number = pwm_output[1];
    pwm3_output_u.number = pwm_output[2];

    vTaskDelayUntil(&lastWake, period);
  }
}

void MotorTask(void *pvParameters)
{
  int motor = *((int*)pvParameters);

  for (;;)
  {
    float pwm = 0.0f;
    long enc_cnt = 0;

    if (xSemaphoreTake(pwmMutex, portMAX_DELAY) == pdTRUE)
    {
      pwm = pwm_output[motor - 1];
      xSemaphoreGive(pwmMutex);
    }

    portENTER_CRITICAL(&encMux);
    if (motor == 1)      { enc_cnt = encoderCount1; e1.number = encoderCount1; }
    else if (motor == 2) { enc_cnt = encoderCount2; e2.number = encoderCount2; }
    else if (motor == 3) { enc_cnt = encoderCount3; e3.number = encoderCount3; }
    portEXIT_CRITICAL(&encMux);

    switch (motor)
    {
      case 1: setMotorPWM(M1_POS, M1_NEG, pwm, enc_cnt); break;
      case 2: setMotorPWM(M2_POS, M2_NEG, pwm, enc_cnt); break;
      case 3: setMotorPWM(M3_POS, M3_NEG, pwm, enc_cnt); break;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void SerialTask(void *pvParameters)
{
  (void)pvParameters;

  static float theta_d_prev = 0.0f;
  static float theta_d_cont = 0.0f;

  for (;;)
  {
    while (Serial.available() && Serial.peek() != 'A')
    {
      Serial.read();
    }

    myValue1.number       = getFloatFromSerial();
    myValue2.number       = getFloatFromSerial();
    Kp_Phi.number         = getFloatFromSerial();
    Kp_Theta.number       = getFloatFromSerial();
    Ki_Phi.number         = getFloatFromSerial();
    Ki_Theta.number       = getFloatFromSerial();
    Kd_Phi.number         = getFloatFromSerial();
    Kd_Theta.number       = getFloatFromSerial();
    integral_clamp.number = getFloatFromSerial();
    k_ff_val.number       = getFloatFromSerial();

    K_FEEDFORWARD = k_ff_val.number;
    psi_desired.phi_desired = myValue1.number;

    // Continuous unwrapping for theta
    float theta_wrapped = myValue2.number;
    float delta = atan2f(sin(theta_wrapped - theta_d_prev), cos(theta_wrapped - theta_d_prev));
    theta_d_cont += delta;
    theta_d_prev = theta_wrapped;
    psi_desired.theta_desired = theta_d_cont;

    // Retained for compatibility
    Kp_phi = Kp_Phi.number;         Kp_theta = Kp_Theta.number;
    Ki_phi = Ki_Phi.number;         Ki_theta = Ki_Theta.number;
    Kd_phi = Kd_Phi.number;         Kd_theta = Kd_Theta.number;
    INTEGRAL_CLAMP = integral_clamp.number;

    serialDataReceived = true;

    // Send Feedback to Simulink
    float phi_actual, theta_actual;
    noInterrupts();
    phi_actual = psi.phi; theta_actual = psi.theta;
    interrupts();

    phiVal.number = phi_actual;
    thetaVal.number = theta_actual;

    Serial.write('A');
    for (int i = 0; i < 4; i++) Serial.write(phiVal.bytes[i]);
    for (int i = 0; i < 4; i++) Serial.write(thetaVal.bytes[i]);
    for (int i = 0; i < 4; i++) Serial.write(phi_error.bytes[i]);
    for (int i = 0; i < 4; i++) Serial.write(theta_error.bytes[i]);
    for (int i = 0; i < 4; i++) Serial.write(pwm1_output_u.bytes[i]);
    for (int i = 0; i < 4; i++) Serial.write(pwm2_output_u.bytes[i]);
    for (int i = 0; i < 4; i++) Serial.write(pwm3_output_u.bytes[i]);
    for (int i = 0; i < 4; i++) Serial.write(e1.bytes[i]);
    for (int i = 0; i < 4; i++) Serial.write(e2.bytes[i]);
    for (int i = 0; i < 4; i++) Serial.write(e3.bytes[i]);
    for (int i = 0; i < 4; i++) Serial.write(dL1.bytes[i]);
    for (int i = 0; i < 4; i++) Serial.write(dL2.bytes[i]);
    for (int i = 0; i < 4; i++) Serial.write(dL3.bytes[i]);
    Serial.write('\n');

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ============================================================
// Setup
// ============================================================

void setup()
{
  Serial.begin(500000);
  delay(1000);
  Serial.println("\n[BOOT] Starting ESP32 Open-Loop Continuum Robot Controller...");

  psiMutex = xSemaphoreCreateMutex();
  pwmMutex = xSemaphoreCreateMutex();

  // Dual I2C Setup
  Serial.println("[I2C] Initializing Wire (Tip IMU)...");
  Wire.begin(TIP_SDA_PIN, TIP_SCL_PIN);
  Wire.setClock(400000);

  Serial.println("[I2C] Initializing Wire1 (Base IMU)...");
  Wire1.begin(BASE_SDA_PIN, BASE_SCL_PIN);
  Wire1.setClock(400000);

  delay(500);

  pinMode(blueLEDPin, OUTPUT);
  digitalWrite(blueLEDPin, LOW);

  // Motor & Encoder Pin Setup
  pinMode(M1_POS, OUTPUT); pinMode(M1_NEG, OUTPUT);
  pinMode(M2_POS, OUTPUT); pinMode(M2_NEG, OUTPUT);
  pinMode(M3_POS, OUTPUT); pinMode(M3_NEG, OUTPUT);

  analogWrite(M1_POS, 0); analogWrite(M1_NEG, 0);
  analogWrite(M2_POS, 0); analogWrite(M2_NEG, 0);
  analogWrite(M3_POS, 0); analogWrite(M3_NEG, 0);

  pinMode(ENC1_A, INPUT_PULLUP); pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP); pinMode(ENC2_B, INPUT_PULLUP);
  pinMode(ENC3_A, INPUT_PULLUP); pinMode(ENC3_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC3_A), encoder3_ISR, RISING);

  // IMU Setup
  Serial.print("[IMU] Initializing Tip MPU9250 on Wire... ");
  if (!mpuTip.setup(0x68, MPU9250Setting(), Wire))
    Serial.println("FAILED!");
  else
    Serial.println("SUCCESS!");

  Serial.print("[IMU] Initializing Base MPU9250 on Wire1... ");
  if (!mpuBase.setup(0x68, MPU9250Setting(), Wire1))
    Serial.println("FAILED!");
  else
    Serial.println("SUCCESS!");

  // Library-level accel/gyro calibration first: this measures and applies
  // its own internal bias correction, so every getGyroX/Y/Z() call below
  // already reflects it. The manual pass in calibrateStaticGyroBias() then
  // only needs to clean up whatever small residual bias is left over,
  // rather than removing the sensor's full raw resting bias by itself.
  // NOTE: keep the module level and still during this call -- it also
  // derives the accelerometer's zero-g offset, which assumes gravity is
  // aligned with the sensor's expected "up" axis at this moment.
  Serial.println("[IMU] Running library accel/gyro calibration - keep module level and still...");
  mpuTip.calibrateAccelGyro();
  mpuBase.calibrateAccelGyro();

  Serial.println("[IMU] Hold module still for gyro bias calibration...");
  calibrateStaticGyroBias();

  Serial.println("[IMU] Warming up and capturing base offset...");
  warmupAndCaptureOffset();
  Serial.println("[IMU] Calibration complete.");

  // Motor IDs
  static int m1_id = 1;
  static int m2_id = 2;
  static int m3_id = 3;

  // FreeRTOS Task Creation
  xTaskCreatePinnedToCore(TaskReadIMU,    "IMU_Task",    4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(ControllerTask, "Ctrl_Task",   4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(SerialTask,     "Serial_Task", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(MotorTask,      "Motor1_Task", 2048, &m1_id, 2, NULL, 1);
  xTaskCreatePinnedToCore(MotorTask,      "Motor2_Task", 2048, &m2_id, 2, NULL, 1);
  xTaskCreatePinnedToCore(MotorTask,      "Motor3_Task", 2048, &m3_id, 2, NULL, 1);

  digitalWrite(blueLEDPin, HIGH);
  Serial.println("[BOOT] System initialization complete.");
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}
