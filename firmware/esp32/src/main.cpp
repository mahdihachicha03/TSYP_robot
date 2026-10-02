// ESP32 low-level controller.
//
// Serial protocol (ASCII, newline-terminated):
//   Pi -> ESP32
//     V <left_rad_s> <right_rad_s>   drive sprocket velocity targets
//     P <kp> <ki> <kd> <kf>          set PID gains (both sides)
//     R                              reset encoder counts
//   ESP32 -> Pi (at CONTROL_HZ)
//     E <left_ticks> <right_ticks> <ax> <ay> <az> <gx> <gy> <gz>
//       ticks are cumulative counts, acceleration in m/s^2, angular rate in rad/s
//     I <text>                       informational message

#include <Arduino.h>
#include <Wire.h>

#include "config.h"

// ---------------------------------------------------------------------------
// Encoders: full quadrature decoding with interrupts on both channels
// ---------------------------------------------------------------------------

static const int8_t QUAD_TABLE[16] = {0, -1, 1, 0, 1, 0, 0, -1,
                                      -1, 0, 0, 1, 0, 1, -1, 0};

struct Encoder {
  uint8_t pin_a, pin_b;
  volatile int32_t count;
  volatile uint8_t state;
};

static Encoder enc_left = {LEFT_ENC_A, LEFT_ENC_B, 0, 0};
static Encoder enc_right = {RIGHT_ENC_A, RIGHT_ENC_B, 0, 0};
static portMUX_TYPE enc_mux = portMUX_INITIALIZER_UNLOCKED;

static inline void IRAM_ATTR update_encoder(Encoder &e) {
  uint8_t s = (digitalRead(e.pin_a) << 1) | digitalRead(e.pin_b);
  e.count += QUAD_TABLE[(e.state << 2) | s];
  e.state = s;
}

static void IRAM_ATTR isr_left() { update_encoder(enc_left); }
static void IRAM_ATTR isr_right() { update_encoder(enc_right); }

static void setup_encoder(Encoder &e, void (*isr)()) {
  pinMode(e.pin_a, INPUT_PULLUP);
  pinMode(e.pin_b, INPUT_PULLUP);
  e.state = (digitalRead(e.pin_a) << 1) | digitalRead(e.pin_b);
  attachInterrupt(digitalPinToInterrupt(e.pin_a), isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(e.pin_b), isr, CHANGE);
}

static void read_encoders(int32_t &left, int32_t &right) {
  portENTER_CRITICAL(&enc_mux);
  left = enc_left.count;
  right = enc_right.count;
  portEXIT_CRITICAL(&enc_mux);
  if (LEFT_ENC_INVERT) left = -left;
  if (RIGHT_ENC_INVERT) right = -right;
}

// ---------------------------------------------------------------------------
// Motors
// ---------------------------------------------------------------------------

struct Motor {
  uint8_t pwm, in1, in2, channel;
  bool invert;
};

static Motor motor_left = {LEFT_PWM, LEFT_IN1, LEFT_IN2, 0, LEFT_MOTOR_INVERT};
static Motor motor_right = {RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, 1, RIGHT_MOTOR_INVERT};

static void setup_motor(const Motor &m) {
  pinMode(m.in1, OUTPUT);
  pinMode(m.in2, OUTPUT);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(m.pwm, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
#else
  ledcSetup(m.channel, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(m.pwm, m.channel);
#endif
}

static void set_motor(const Motor &m, int pwm) {
  if (m.invert) pwm = -pwm;
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  digitalWrite(m.in1, pwm > 0);
  digitalWrite(m.in2, pwm < 0);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(m.pwm, abs(pwm));
#else
  ledcWrite(m.channel, abs(pwm));
#endif
}

// ---------------------------------------------------------------------------
// PID speed control
// ---------------------------------------------------------------------------

struct Pid {
  float kp, ki, kd, kf;
  float integral, prev_error;

  float update(float target, float measured, float dt) {
    if (target == 0.0f) {
      integral = 0.0f;
      prev_error = 0.0f;
      return 0.0f;
    }
    float error = target - measured;
    float derivative = (error - prev_error) / dt;
    prev_error = error;
    float out = kf * target + kp * error + kd * derivative + ki * integral;
    // Integrate only while not saturated (anti-windup)
    if (fabsf(out) < PWM_MAX) integral += error * dt;
    return constrain(out, -(float)PWM_MAX, (float)PWM_MAX);
  }
};

static Pid pid_left = {PID_KP, PID_KI, PID_KD, PID_KF, 0, 0};
static Pid pid_right = {PID_KP, PID_KI, PID_KD, PID_KF, 0, 0};

// ---------------------------------------------------------------------------
// MPU6050 accelerometer + gyroscope
// ---------------------------------------------------------------------------

static bool imu_ok = false;
static const float G = 9.80665f;
static const float ACCEL_LSB_PER_G = 16384.0f;    // +-2 g
static const float GYRO_LSB_PER_DPS = 65.5f;      // +-500 deg/s
static float gyro_bias[3] = {0, 0, 0};             // raw counts

static void mpu_write(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Reads accel and gyro raw counts (temperature skipped). Returns false on I2C error.
static bool mpu_read_raw(int16_t accel[3], int16_t gyro[3]) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU6050_ADDR, 14) != 14) return false;
  int16_t raw[7];
  for (int i = 0; i < 7; i++) raw[i] = (int16_t)((Wire.read() << 8) | Wire.read());
  for (int i = 0; i < 3; i++) {
    accel[i] = raw[i];
    gyro[i] = raw[i + 4];
  }
  return true;
}

static bool setup_imu() {
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x75);  // WHO_AM_I
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(MPU6050_ADDR, 1);
  if (!Wire.available()) return false;
  uint8_t who = Wire.read();
  // Genuine MPU6050 answers 0x68; clones and MPU6500-based boards answer other values
  if (who != 0x68) Serial.printf("I imu WHO_AM_I=0x%02X (clone?), continuing\n", who);

  mpu_write(0x6B, 0x01);  // PWR_MGMT_1: wake up, clock from gyro X PLL
  delay(100);
  mpu_write(0x1A, 0x03);  // CONFIG: 44 Hz low-pass filter
  mpu_write(0x1B, 0x08);  // GYRO_CONFIG: +-500 deg/s
  mpu_write(0x1C, 0x00);  // ACCEL_CONFIG: +-2 g
  delay(50);

  // Gyro bias: average readings while the robot is standing still
  int16_t accel[3], gyro[3];
  long sum[3] = {0, 0, 0};
  int n = 0;
  for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
    if (mpu_read_raw(accel, gyro)) {
      for (int k = 0; k < 3; k++) sum[k] += gyro[k];
      n++;
    }
    delay(2);
  }
  if (n == 0) return false;
  for (int k = 0; k < 3; k++) gyro_bias[k] = (float)sum[k] / n;
  return true;
}

static void read_imu(float accel[3], float gyro[3]) {
  for (int k = 0; k < 3; k++) accel[k] = gyro[k] = 0.0f;
  int16_t a[3], g[3];
  if (!imu_ok || !mpu_read_raw(a, g)) return;
  const float rad_per_lsb = (PI / 180.0f) / GYRO_LSB_PER_DPS;
  for (int k = 0; k < 3; k++) {
    accel[k] = a[k] / ACCEL_LSB_PER_G * G;
    gyro[k] = (g[k] - gyro_bias[k]) * rad_per_lsb;
  }
}

// ---------------------------------------------------------------------------
// Serial command handling
// ---------------------------------------------------------------------------

static float target_left = 0.0f, target_right = 0.0f;
static uint32_t last_cmd_ms = 0;
static char line_buf[96];
static size_t line_len = 0;

static void handle_line(char *line) {
  float a, b, c, d;
  switch (line[0]) {
    case 'V':
      if (sscanf(line + 1, "%f %f", &a, &b) == 2) {
        target_left = constrain(a, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
        target_right = constrain(b, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
        last_cmd_ms = millis();
      }
      break;
    case 'P':
      if (sscanf(line + 1, "%f %f %f %f", &a, &b, &c, &d) == 4) {
        pid_left = {a, b, c, d, 0, 0};
        pid_right = {a, b, c, d, 0, 0};
        Serial.printf("I pid %.3f %.3f %.3f %.3f\n", a, b, c, d);
      }
      break;
    case 'R':
      portENTER_CRITICAL(&enc_mux);
      enc_left.count = 0;
      enc_right.count = 0;
      portEXIT_CRITICAL(&enc_mux);
      break;
  }
}

static void poll_serial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (line_len > 0) {
        line_buf[line_len] = '\0';
        handle_line(line_buf);
        line_len = 0;
      }
    } else if (line_len < sizeof(line_buf) - 1) {
      line_buf[line_len++] = ch;
    } else {
      line_len = 0;  // overflow: drop the line
    }
  }
}

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  setup_encoder(enc_left, isr_left);
  setup_encoder(enc_right, isr_right);
  setup_motor(motor_left);
  setup_motor(motor_right);
  set_motor(motor_left, 0);
  set_motor(motor_right, 0);
  imu_ok = setup_imu();
  Serial.printf("I boot imu=%s\n", imu_ok ? "ok" : "missing");
}

void loop() {
  static const uint32_t period_us = 1000000UL / CONTROL_HZ;
  static uint32_t last_us = micros();
  static int32_t prev_left = 0, prev_right = 0;

  poll_serial();

  uint32_t now_us = micros();
  if (now_us - last_us < period_us) return;
  float dt = (now_us - last_us) * 1e-6f;
  last_us = now_us;

  int32_t left, right;
  read_encoders(left, right);
  const float rad_per_count = 2.0f * PI / COUNTS_PER_WHEEL_REV;
  float vel_left = (left - prev_left) * rad_per_count / dt;
  float vel_right = (right - prev_right) * rad_per_count / dt;
  prev_left = left;
  prev_right = right;

  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    target_left = target_right = 0.0f;
  }
  set_motor(motor_left, (int)pid_left.update(target_left, vel_left, dt));
  set_motor(motor_right, (int)pid_right.update(target_right, vel_right, dt));

  float accel[3], gyro[3];
  read_imu(accel, gyro);
  Serial.printf("E %ld %ld %.3f %.3f %.3f %.4f %.4f %.4f\n", (long)left, (long)right,
                accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);
}
