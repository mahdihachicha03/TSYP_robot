#pragma once

// ---------------------------------------------------------------------------
// Hardware configuration. Values marked TBD must match your parts and wiring.
// Motor A (ENA, IN1/IN2) is the RIGHT track, motor B (ENB, IN3/IN4) the LEFT,
// as in tools/motor_encoder_control.py.
// ---------------------------------------------------------------------------

// Serial link to the Raspberry Pi (must match the bridge node's `baud` param)
#define SERIAL_BAUD 115200

// GA25-370 hall encoders (green = A, yellow = B, blue = VCC 3.3 V, black = GND).
// Pins with internal pull-ups, enabled in firmware.
#define RIGHT_ENC_A 32
#define RIGHT_ENC_B 33
#define LEFT_ENC_A 25
#define LEFT_ENC_B 26

// Encoder resolution: 11 pulses per motor revolution per channel, 15.5:1 gearbox
// (measured by hand-turn calibration, datasheet value 15.5:1).
#define ENCODER_PPR 11
#define GEAR_RATIO 15.5f
#define COUNTS_PER_WHEEL_REV (ENCODER_PPR * 4 * GEAR_RATIO)  // 682, full quadrature

// Flip if a track counts down when the robot drives forward
#define LEFT_ENC_INVERT false   // TBD: check with calibration step 2
#define RIGHT_ENC_INVERT false  // TBD: check with calibration step 2
// Flip if a track runs backwards for a positive command
#define LEFT_MOTOR_INVERT false
#define RIGHT_MOTOR_INVERT false

// L298N dual H-bridge (remove the ENA/ENB jumpers so the pins can be PWM-driven)
#define RIGHT_PWM 18  // ENA
#define RIGHT_IN1 19  // IN1
#define RIGHT_IN2 23  // IN2
#define LEFT_PWM 14   // ENB
#define LEFT_IN1 27   // IN3
#define LEFT_IN2 13   // IN4

// The L298N's bipolar transistors switch slowly: keep the PWM frequency low
#define PWM_FREQ_HZ 1000
#define PWM_RESOLUTION_BITS 8
#define PWM_MAX 255

// I2C to the MPU6050
#define I2C_SDA 21
#define I2C_SCL 22
#define MPU6050_ADDR 0x68        // 0x69 if AD0 is pulled high
#define GYRO_CALIBRATION_SAMPLES 500  // robot must be still at boot

// Control loop
#define CONTROL_HZ 50
#define CMD_TIMEOUT_MS 500       // stop the motors if no command arrives
#define MAX_WHEEL_SPEED 60.0f    // rad/s at the sprocket (620 RPM no-load = 64.9 rad/s)

// Default PID gains (sprocket speed in rad/s -> PWM). Tune with the `P` command.
#define PID_KP 2.0f
#define PID_KI 10.0f
#define PID_KD 0.0f
#define PID_KF 4.5f              // feed-forward: PWM per rad/s (~255 / 57 rad/s after L298N drop)
