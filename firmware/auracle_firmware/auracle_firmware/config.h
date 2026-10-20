#pragma once
#include <Arduino.h>

// ============================================================
// Every PWM constant below was measured with ALL of:
//   - Timer1 (D9/D10) switched to ~976 Hz by MotorDriver::configurePwmTimers()
//     so all four enables run at the same frequency (Timer0 = D5/D6 already does)
//   - 2x encoder decoding (CHANGE on channel A, channel B = direction)
//   - robot propped up (no load)  -> re-check on the ground
// Change any of those and the PWM numbers are no longer valid.
// ============================================================

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
// Rear encoders, 2x decoding: CHANGE interrupt on the INT pin (channel A),
// DIR pin = channel B. A == B counts as ENC_SIGN, A != B as -ENC_SIGN.
constexpr uint8_t PIN_RB_ENC_INT = 2;
constexpr uint8_t PIN_RB_ENC_DIR = 4;
constexpr uint8_t PIN_LB_ENC_INT = 3;
constexpr uint8_t PIN_LB_ENC_DIR = 13;
// MPU6050 on A4 (SDA) / A5 (SCL).

constexpr long BAUDRATE = 115200;  // must match ros2_control.xacro

// --- Encoders ---
// Measured: A == B happens when driving BACKWARD on both rear wheels, so the sign is -1.
constexpr int8_t LB_ENC_SIGN = -1;
constexpr int8_t RB_ENC_SIGN = -1;
// 2x decoding gives 2 counts per old 1x tick. All speeds/targets stay in the old 1x
// "ticks/s", so the PI constants and the Pi side don't change.
constexpr float COUNT_TO_TICK = 0.5f;
// false: telemetry sends 1x-equivalent counts (nothing changes on the Pi).
// true : telemetry sends raw 2x counts (then double the encoder counts-per-rev in ros2_control.xacro).
constexpr bool TELEMETRY_RAW_2X = false;

// --- Motors ---
constexpr int MAX_PWM = 255;

// KICK  = PWM held from rest (until the wheel passes KICK_EXIT_TICKS_S or KICK_MS runs out)
//         so the wheel beats static friction. Worst breakaway seen + margin.
//         RB breakaway moved between 81 and 129 over repeated runs (stiction depends on wheel position).
// FLOOR = lowest PWM the controller may command while turning. Kept ~15% below the PWM needed at
//         MIN_TARGET_TICKS_S. PROVISIONAL until the 'h' (hold-min) test is run at 976 Hz.
constexpr uint8_t LB_KICK_FWD  = 60,  LB_KICK_REV  = 60;
constexpr uint8_t RB_KICK_FWD  = 118, RB_KICK_REV  = 135;
constexpr uint8_t LB_FLOOR_FWD = 60,  LB_FLOOR_REV = 60;
constexpr uint8_t RB_FLOOR_FWD = 85,  RB_FLOOR_REV = 85;

// Feedforward: PWM = BASE + K * |ramped target ticks/s|. Least-squares fits of settled
// steady-state means (speed -> PWM) from the 976 Hz auto test, no load:
//   LB fwd  600->102  1013->156  1204->195   (+600 coming down ->97)
//   LB rev  604->98    800->122  1009->156
//   RB fwd  603->108  1006->165  1204->206
//   RB rev  807->117  1003->152                (only 2 settled points: the -600 row was lost to a bad wire,
//                                               so this line is LB rev scaled by 0.965, which fits both)
// Fit residuals are +-5 PWM; the PI trims the rest.
constexpr float LB_FF_FWD_BASE = 6.7f,  LB_FF_FWD_K = 0.1531f;
constexpr float LB_FF_REV_BASE = 10.0f, LB_FF_REV_K = 0.1434f;
constexpr float RB_FF_FWD_BASE = 9.7f,  RB_FF_FWD_K = 0.1599f;
constexpr float RB_FF_REV_BASE = 9.7f,  RB_FF_REV_K = 0.1384f;

// Front wheels (no encoders) copy their side's REAR PWM:  front = rear * TRIM, at least FRONT_MIN.
// UNVALIDATED. Once PWM frequencies match, LB and RB need within ~5% of the same PWM at every speed, so
// equal PWM (trim 1.0) is the best guess for same-model motors. The old 1.15 / 1.5 offsets were measured
// at mismatched PWM frequencies (and while RB was failing); re-run the front-offset test and set
//   new_trim = old_trim * (rear_speed / front_speed)
constexpr float LF_TRIM = 1.3f;
constexpr float RF_TRIM = 1.0f;
constexpr uint8_t LF_MIN_PWM = 70;   // stall by eye at the old 490 Hz
constexpr uint8_t RF_MIN_PWM = 51;   // stall by eye

// --- Speed targets (1x ticks/s) ---
// MIN: lowest speed validated for both rears (600 settled within 1% on both). Anything lower is a stop.
//      Lower it only after the 'h' test and a 'run t300 / t450' check in the test sketch.
// MAX: validated at 1200 with 20% PWM headroom (LB 195, RB 206 of 255). RB reaches ~1540 at PWM 255.
constexpr float MIN_TARGET_TICKS_S = 600.0f;
constexpr float MAX_TICKS_S        = 1200.0f;

// Same-sign wheel pairs can differ by at most MAX - MIN. Beyond that, counter-rotate.
constexpr float CMD_DEADBAND_TICKS_S = 150.0f;   // faster wheel below this = stop

// --- Rear wheel controller: PI + feedforward (tuned in the auto test) ---
constexpr float LB_KP = 0.03f;
constexpr float LB_KI = 0.15f;    // PWM per second per (tick/s of error)
constexpr float RB_KP = 0.025f;
constexpr float RB_KI = 0.20f;
constexpr unsigned long CONTROL_PERIOD_MS = 50;
constexpr float SPEED_FILTER_ALPHA    = 0.4f;     // speed += alpha * (raw - speed)
constexpr float ERR_CLAMP_TICKS_S     = 300.0f;   // limits the push during spin-up
constexpr float INTEG_MAX_PWM         = 60.0f;    // integral clamp (anti-windup, +/-)
constexpr float MAX_PWM_STEP          = 12.0f;    // max PWM change per control period
constexpr float TARGET_RAMP_TICKS_S2  = 2000.0f;  // setpoint ramp (ticks/s per second)
constexpr float OVERSPEED_COAST_RATIO = 1.25f;    // above this x target: cut PWM to 0
constexpr unsigned long KICK_MS       = 300;
constexpr float KICK_EXIT_TICKS_S     = 150.0f;   // kick ends early once the wheel is this fast

// --- Stopping and reversing ---
// Stop / reverse = short brake (both L298N inputs HIGH, EN HIGH) for BRAKE_MS if the wheel is
// moving faster than BRAKE_MIN_TICKS_S, then coast until standstill before driving the other way.
// Set BRAKE_MS to 0 to coast only.
constexpr unsigned long BRAKE_MS         = 200;
constexpr float         BRAKE_MIN_TICKS_S = 50.0f;
constexpr unsigned long REVERSE_WAIT_MS  = 800;   // max wait for standstill before reversing
constexpr float         STILL_TICKS_S    = 30.0f;

// Extra front-wheel PWM while turning (fronts have no encoder, so they can't sense the scrub load).
constexpr uint8_t FRONT_TURN_BOOST_PWM      = 40;      // tune: raise until the stutter stops
constexpr float   BOOST_FULL_DIFF_TICKS_S   = 600.0f;  // |right - left| target where the full boost applies
// --- Safety ---
constexpr unsigned long COMMS_TIMEOUT_MS = 500;   // stop if the Pi goes quiet
// A wheel that runs away or never reaches its target latches a fault: every motor stops and
// stays stopped until an 's' or a zero 'v' command. This is what an unplugged encoder looks like.
constexpr unsigned long FAULT_GRACE_MS   = 800;   // no fault checks this long after a start
constexpr float FAULT_WRONG_SIGN_TICKS_S = 100.0f; // moving this fast against the command ...
constexpr uint8_t FAULT_WRONG_SIGN_COUNT = 6;      // ... for this many control periods (300 ms)
constexpr uint8_t FAULT_SATURATED_COUNT  = 40;     // PWM at 255 and < 80% of target for 2 s
constexpr float   FAULT_NO_MOTION_FRAC   = 0.25f;  // wheel slower than this fraction of the ramped target ...
constexpr uint8_t FAULT_NO_MOTION_COUNT  = 20;     // ... for 1 s = dead encoder or blocked wheel
// Send "F <L|R> <code>" lines on a fault (1 = wrong sign, 2 = saturated, 3 = no motion). Make sure the Pi-side
// parser ignores lines that are not 'e ...' / 'READY', or set this to false.
constexpr bool REPORT_FAULTS = true;