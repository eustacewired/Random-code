#pragma once
#include <Arduino.h>

// --- Pins (README "Real robot") ---
// Direction bridged per side (IN1+IN3 / IN2+IN4), PWM per wheel.
// No Servo library: it takes Timer1 and kills PWM on D9/D10.
constexpr uint8_t PIN_RF_EN = 5;
constexpr uint8_t PIN_RB_EN = 6;
constexpr uint8_t PIN_R_FWD = 7;
constexpr uint8_t PIN_R_REV = 8;
constexpr uint8_t PIN_LB_EN = 9;
constexpr uint8_t PIN_LF_EN = 10;
constexpr uint8_t PIN_L_FWD = A0;
constexpr uint8_t PIN_L_REV = A1;
// Rear encoders, 1x decoding: RISING on INT pin, DIR pin HIGH = forward.
constexpr uint8_t PIN_RB_ENC_INT = 2; 
constexpr uint8_t PIN_RB_ENC_DIR = 4;
constexpr uint8_t PIN_LB_ENC_INT = 3;
constexpr uint8_t PIN_LB_ENC_DIR = 13;
// MPU6050 on A4 (SDA) / A5 (SCL).

constexpr long BAUDRATE = 115200;  // must match ros2_control.xacro

// --- Motors (stall ramp test) ---
constexpr int MAX_PWM = 255;
constexpr uint8_t LB_MIN_PWM = 45;
constexpr uint8_t RB_MIN_PWM = 60;
constexpr uint8_t LF_MIN_PWM = 45;
constexpr uint8_t RF_MIN_PWM = 90;
// Front (no encoder) PWM = rear PWM * offset. See README front-offset test.
constexpr float LF_OFFSET = 1.15f;
constexpr float RF_OFFSET = 1.5f;

// --- PID (incremental form, ticks/s). Kp was tuned at 50 ms. ---
constexpr float PID_KP = 0.08f;
constexpr float PID_KI = 0.0f;
constexpr float PID_KD = 0.0f;
constexpr unsigned long CONTROL_PERIOD_MS = 50;
constexpr float DEADZONE_TICKS_S = 25.0f;    // 1 tick / 50 ms = 20 ticks/s jitter
constexpr float MIN_TARGET_TICKS_S = 25.0f;  // below this = stop
constexpr float MAX_TICKS_S = 1400.0f;

// --- Safety ---
constexpr unsigned long COMMS_TIMEOUT_MS = 500;  // stop if the Pi goes quiet
