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
constexpr uint8_t PIN_LB_ENC_DIR = 12;  // moved off D13 (onboard LED loads the pull-up)
// MPU6050 on A4 (SDA) / A5 (SCL).

constexpr long BAUDRATE = 115200;  // must match ros2_control.xacro

// --- Encoders ---
// Measured: A == B happens when driving BACKWARD on both rear wheels, so the sign is -1.
constexpr int8_t LB_ENC_SIGN = -1;
constexpr int8_t RB_ENC_SIGN = -1;
// 2x decoding gives 2 counts per old 1x tick. All speeds/targets stay in the old 1x
// "ticks/s", so the PI constants and the Pi side don't change.
constexpr float COUNT_TO_TICK = 0.5f;
// 1x ticks per wheel revolution. MUST match left/right_enc_counts_per_rev in ros2_control.xacro.
// conditionTargets() mixes the two sides in rev/s with these, so equal wheel speeds really are equal.
constexpr float LB_TICKS_PER_REV = 985.0f;   // hand-counted 2026-10-23 (10 turns = 9847)
constexpr float RB_TICKS_PER_REV = 942.0f;   // hand-counted 2026-10-23 (10 turns = 9416)
// Right rear encoder: its signal drops out (D2 counts nothing while the wheel turns, 2026-10-23).
// false = the right side runs open loop on the RB feedforward below (no PI, no fault checks) and
// telemetry reports its commanded position instead of encoder counts. Set true once it's fixed.
constexpr bool RB_ENCODER_OK = true;
// Open-loop right-side PWM multipliers. Straight-line test on the ground: if the robot veers RIGHT
// going forward, raise RB_OPEN_GAIN_FWD (e.g. 1.05, 1.10); veers left, lower it. Same for reverse.
constexpr float RB_OPEN_GAIN_FWD = 1.0f;
constexpr float RB_OPEN_GAIN_REV = 1.0f;
constexpr float OPEN_STOP_TICKS_S = 150.0f;   // ramped right target below this = coast
// Right rear encoder: channel A (D2) drops pulses, channel B (D4) is clean (probe, 2026-10-23).
// true = count the right wheel on B edges only (pin-change interrupt), direction from the drive
// command. Set false once the A wire is fixed to go back to normal quadrature decoding.
constexpr bool RB_COUNT_ON_B = true;
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
// RB forward re-measured 2026-10-23 (wheels up, 976 Hz): 120->290, 180->488 ticks/s. RB has lost
// most of its forward strength (reverse is unchanged: -120 -> -806), likely one half of its L298
// channel. Old fit was BASE 9.7, K 0.1599.
constexpr float RB_FF_FWD_BASE = 32.1f, RB_FF_FWD_K = 0.303f;
constexpr float RB_FF_REV_BASE = 9.7f,  RB_FF_REV_K = 0.1384f;

// Front wheels (no encoders) copy their side's REAR PWM:  front = rear * TRIM, at least FRONT_MIN.
// UNVALIDATED. Once PWM frequencies match, LB and RB need within ~5% of the same PWM at every speed, so
// equal PWM (trim 1.0) is the best guess for same-model motors. The old 1.15 / 1.5 offsets were measured
// at mismatched PWM frequencies (and while RB was failing); re-run the front-offset test and set
//   new_trim = old_trim * (rear_speed / front_speed)
// Teleop 2026-10-23: fronts visibly faster than rears, robot drifted left holding 'w'. RB is weak
// FORWARD, so its PI asks for ~2x the PWM a healthy motor needs, and RF copied that and out-ran it.
// RF forward is halved to match; reverse (RB healthy) stays 1.0. LF was 1.3 (unvalidated) -> 1.0.
// Straight-line test: drifts LEFT going forward -> lower RF_TRIM_FWD; drifts RIGHT -> raise it.
constexpr float LF_TRIM_FWD = 1.0f, LF_TRIM_REV = 1.0f;
constexpr float RF_TRIM_FWD = 0.5f, RF_TRIM_REV = 1.0f;
constexpr uint8_t LF_MIN_PWM = 70;   // stall by eye at the old 490 Hz
constexpr uint8_t RF_MIN_PWM = 51;   // stall by eye

// --- Speed targets (1x ticks/s) ---
// MIN: lowest speed validated for both rears (600 settled within 1% on both). Anything lower is a stop.
//      Lower it only after the 'h' test and a 'run t300 / t450' check in the test sketch.
// MAX: validated at 1200 with 20% PWM headroom (LB 195, RB 206 of 255). RB reaches ~1540 at PWM 255.
constexpr float MIN_TARGET_TICKS_S = 450.0f;   // was 600; lowered so arcs fit under RB's weak forward cap
constexpr float MAX_TICKS_S        = 1200.0f;
// RB reaches only ~735 ticks/s forward at PWM 255 (see RB_FF_FWD). Keep ~10% headroom for the PI.
// conditionTargets() caps the right wheel's forward speed at this. Raise it once RB is repaired.
constexpr float RB_FWD_MAX_TICKS_S = 740.0f;   // was 650; raised for speed (~0.16 m/s). Lower again if it curves right

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
constexpr float INTEG_MAX_PWM         = 100.0f;   // integral clamp (anti-windup, +/-). Was 60: too little to un-stall a spin on the floor
constexpr float MAX_PWM_STEP          = 12.0f;    // max PWM change per control period
// Soft start in MotorDriver::drive(): max PWM rise per control period, for every motor (rear and
// front). 0 -> 135 kick in 4 periods (200 ms) instead of one step. Limits the start-up current
// spike that browns out the Pi when it runs off the same LiPo. Decreases are not limited.
constexpr uint8_t PWM_RISE_PER_CALL   = 35;
constexpr float TARGET_RAMP_TICKS_S2  = 2000.0f;  // setpoint ramp (ticks/s per second)
constexpr float OVERSPEED_COAST_RATIO = 1.25f;    // above this x target: cut PWM to 0
constexpr unsigned long KICK_MS       = 450;    // was 300; +150 ms covers the soft-start ramp up to the kick PWM
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
constexpr uint8_t FRONT_TURN_BOOST_PWM      = 80;      // was 40; spins still stalled the right side
// Same idea for the closed-loop REAR wheels: extra feedforward PWM while turning. A skid-steer
// spin on the floor needed more than the PI could add in its 1 s before NO_MOTION fired: all 54
// faults in the first teleop run were the right side stalling (speed 0) during a spin.
constexpr uint8_t REAR_TURN_BOOST_PWM       = 60;
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
// A latched fault no longer needs a stick release to recover: if the Pi is still asking for motion
// FAULT_RETRY_MS after the fault, the fault is cleared and the wheel is restarted from rest (kick PWM
// again, which also beats a stiction stall). After FAULT_MAX_RETRIES in a row with no zero command in
// between it stays latched, so a really dead encoder can't keep driving a wheel forever.
constexpr unsigned long FAULT_RETRY_MS  = 300; 
constexpr uint8_t       FAULT_MAX_RETRIES = 3;
// What a stall (NO_MOTION / SATURATED) does. true = stop every motor and retry, as before.
// false = report it to the Pi and keep driving: stopping can't free a wheel that is stalling in a
// skid-steer spin, it only turned every stall into the stop-start "burst". WRONG_SIGN always stops.
constexpr bool FAULT_STOP_ON_STALL = false;
// Send "F <L|R> <code> <speed> <target>" lines on a fault (1 = wrong sign, 2 = saturated, 3 = no motion). Make sure the Pi-side
// parser ignores lines that are not 'e ...' / 'READY', or set this to false.
constexpr bool REPORT_FAULTS = true;