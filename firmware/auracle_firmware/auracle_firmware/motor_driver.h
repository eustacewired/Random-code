#pragma once
#include <Arduino.h>

// One side: two motors sharing direction pins, separate PWM enables.
// The rear motor is closed-loop (PIDController); the front motor has no encoder and
// copies the rear PWM: front = rear * frontTrim, but at least frontMinPwm while driving.
class MotorDriver {
public:
    MotorDriver(uint8_t pinFwd, uint8_t pinRev, uint8_t pinRearEn, uint8_t pinFrontEn,
                float frontTrim, uint8_t frontMinPwm);

    // Call once, before begin(). Moves Timer1 (D9 LB, D10 LF) from 490 Hz phase-correct to
    // ~976 Hz fast PWM so it matches Timer0 (D5 RF, D6 RB). Does not touch Timer0 / millis().
    static void configurePwmTimers();

    void begin();
    // Signed REAR PWM -255..255, used as given (no floor). 0 = coast.
    void drive(int effort);
    // Short brake: both direction inputs HIGH with the enables HIGH.
    void brake();
    // Everything off.
    void coast();
        void setFrontBoost(uint8_t b) { frontBoost_ = b; }

private:
    uint8_t pinFwd_, pinRev_, pinRearEn_, pinFrontEn_;
    float frontTrim_;
    uint8_t frontMinPwm_;
        uint8_t frontBoost_ = 0;   
};