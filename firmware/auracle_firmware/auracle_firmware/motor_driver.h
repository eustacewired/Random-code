#pragma once
#include <Arduino.h>

// One side: two motors sharing direction pins, separate PWM enables.
// The rear motor is closed-loop (PIDController); the front motor has no encoder and
// copies the rear PWM: front = rear * frontTrim (per direction), but at least frontMinPwm while driving.
class MotorDriver {
public:
    MotorDriver(uint8_t pinFwd, uint8_t pinRev, uint8_t pinRearEn, uint8_t pinFrontEn,
                float frontTrimFwd, float frontTrimRev, uint8_t frontMinPwm);

    // Call once, before begin(). Moves Timer1 (D9 LB, D10 LF) from 490 Hz phase-correct to
    // ~976 Hz fast PWM so it matches Timer0 (D5 RF, D6 RB). Does not touch Timer0 / millis().
    static void configurePwmTimers();

    void begin();
    // Signed REAR PWM -255..255 (no floor). 0 = coast. Increases are limited to
    // PWM_RISE_PER_CALL per call (soft start); decreases are immediate.
    void drive(int effort);
    // Short brake: both direction inputs HIGH with the enables HIGH.
    void brake();
    // Everything off.
    void coast();
    void setFrontBoost(uint8_t b) { frontBoost_ = b; }
    // +1 / -1 while driving, 0 while coasting or braking.
    int8_t direction() const { return outDir_; }

private:
    uint8_t pinFwd_, pinRev_, pinRearEn_, pinFrontEn_;
    float frontTrimFwd_, frontTrimRev_;
    uint8_t frontMinPwm_;
    uint8_t frontBoost_ = 0;
    // Last PWM actually written, for the soft-start limit in drive(). 0 after coast/brake.
    int8_t outDir_ = 0;
    int rearOut_ = 0, frontOut_ = 0;
};