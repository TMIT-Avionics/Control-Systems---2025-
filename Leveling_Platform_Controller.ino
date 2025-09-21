#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <math.h>

// controller parameters
static const float Ts   = 0.01f;            // 10 ms (100 Hz)
static const float Kp   = 5.520505348f;     // proportional gain
static const float Ki   = 10.785968954f;    // integral gain
static const float UMIN = -25.0f;           // servo command clamp [deg]
static const float UMAX =  25.0f;


static const int   SERVO_PIN    = 9;        
static const float SERVO_CENTER = 90.0f;    // neutral angle
static const float SERVO_MIN    = 0.0f;     
static const float SERVO_MAX    = 180.0f;   

static const int   CALIB_SAMPLES = 600;     // ~6s at 100 Hz for zero offset
static const int   WARMUP_MS     = 300;     // let MPU settle


Adafruit_MPU6050 mpu;
Servo servo;

volatile unsigned long nextTickMicros;
float xI = 0.0f;             // integrator state 
float u_deg = 0.0f;          // control output 
float angle_deg = 0.0f;      // measured platform angle
float zero_offset = 0.0f;    // measured at startup
float tsec = 0.0f;

// clamping the output to the range (-25,+25) to keep the system linear
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

bool initMPU() {
  if (!mpu.begin()) return false;
  // low pass for accelerometer to reduce noice
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  delay(WARMUP_MS);
  return true;
}

float readAngleDeg(); 

void calibrateZero() {
  double sum = 0.0;
  for (int i = 0; i < CALIB_SAMPLES; ++i) {
    sum += readAngleDeg();
    delay((int)(Ts * 1000)); 
  }
  zero_offset = (float)(sum / CALIB_SAMPLES);
}

// Kalman Filter for X
float kalmanFilterX(float measurement) {
  static float kalman_x = 0.0f;   // Initial estimate
  static float P_x = 1.0f;        // Initial estimate uncertainty
  const float Q_x = 0.008f;       // Process noise
  const float R_x = 1.4f;         // Measurement noise

  // Prediction update
  P_x += Q_x;

  // Kalman gain
  float K = P_x / (P_x + R_x);

  // Update estimate
  kalman_x = kalman_x + K * (measurement - kalman_x);

  // Update error covariance
  P_x = (1.0f - K) * P_x;

  return kalman_x;
}

// Kalman filter for Y
float kalmanFilterY(float measurement) {
  static float kalman_y = 0.0f;
  static float P_y = 1.0f;
  const float Q_y = 0.008f;
  const float R_y = 1.4f;

  P_y += Q_y;
  float K = P_y / (P_y + R_y);
  kalman_y = kalman_y + K * (measurement - kalman_y);
  P_y = (1.0f - K) * P_y;

  return kalman_y;
}

// Kalman filter for Z
float kalmanFilterZ(float measurement) {
  static float kalman_z = 0.0f;
  static float P_z = 1.0f;
  const float Q_z = 0.008f;
  const float R_z = 1.4f;

  P_z += Q_z;
  float K = P_z / (P_z + R_z);
  kalman_z = kalman_z + K * (measurement - kalman_z);
  P_z = (1.0f - K) * P_z;

  return kalman_z;
}

float readAngleDeg() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x; 
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  float filt_ax = kalmanFilterX(ax);
  float filt_ay = kalmanFilterY(ay);
  float filt_az = kalmanFilterZ(az);

  float ang;
  ang = atan2f(-filt_ax, filt_az) * 180.0f / PI;
  return ang;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  if (!initMPU()) {
    Serial.println(F("MPU6050 not found. Check wiring and I2C address."));
    while (1) { delay(10); }
  }

  servo.attach(SERVO_PIN);

  Serial.println(F("Calibrating level... keep platform level and still."));
  calibrateZero();
  Serial.print(F("Zero offset (deg): ")); Serial.println(zero_offset, 3);

  nextTickMicros = micros() + (unsigned long)(Ts * 1e6f);
}

void loop() {
  // 100 Hz scheduler
  unsigned long now = micros();
  if ((long)(now - nextTickMicros) < 0) return;
  nextTickMicros += (unsigned long)(Ts * 1e6f);
  tsec += Ts;

  // Setpoint (level)
  const float r_deg = 0.0f;

  // Measurement
  angle_deg = readAngleDeg() - zero_offset;

  // PI control with anti-windup
  float e = r_deg - angle_deg;

  // small deadband to reduce chatter
  if (fabsf(e) < 0.2f) e = 0.0f;

  float up = Kp * e;

  // integral update
  float xI_cand = xI + e * Ts;
  float ui_cand = Ki * xI_cand;

  // pre-saturation control
  float u_unsat = up + ui_cand;

  float u_sat = clampf(u_unsat, UMIN, UMAX);

  // anti-windup: block integration if we're pushing further into saturation
  bool pushing_high = (u_unsat > UMAX) && (e > 0.0f);
  bool pushing_low  = (u_unsat < UMIN) && (e < 0.0f);
  if (!(pushing_high || pushing_low)) {
    xI = xI_cand;
  }

  // Use the saturated control for the actuator
  u_deg = u_sat;

  delay(100);

  // Command servo
  float servo_cmd = SERVO_CENTER + u_deg;   // 90° +/- u_deg
  servo_cmd = clampf(servo_cmd, SERVO_MIN, SERVO_MAX);
  servo.write((int)(servo_cmd + 0.5f));

  // Log for Serial Plotter
  Serial.print("Time(s) : "); Serial.print(tsec, 3); Serial.print(',');
  Serial.print("Current Angle : "); Serial.print(angle_deg, 3);   Serial.print(',');
  Serial.print("Commanded Angle : "); Serial.println(u_deg, 3);
}
