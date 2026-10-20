#include "motor_driver.h"
#include "config.h"

MotorDriver::MotorDriver(uint8_t pinFwd, uint8_t pinRev, uint8_t pinRearEn, uint8_t pinFrontEn,
                         float frontTrim, uint8_t frontMinPwm)
    : pinFwd_(pinFwd), pinRev_(pinRev), pinRearEn_(pinRearEn), pinFrontEn_(pinFrontEn),
      frontTrim_(frontTrim), frontMinPwm_(frontMinPwm) {}

void MotorDriver::configurePwmTimers() {
    // Timer1: phase-correct 8-bit (490 Hz) -> fast PWM 8-bit, /64 prescaler = ~976 Hz.
    // Only the waveform and clock bits are changed; the output-compare bits stay as they are.
    TCCR1A = (TCCR1A & 0xFC) | _BV(WGM10);
    TCCR1B = (TCCR1B & 0xE0) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
}

void MotorDriver::begin() {
    pinMode(pinFwd_, OUTPUT);
    pinMode(pinRev_, OUTPUT);
    pinMode(pinRearEn_, OUTPUT);
    pinMode(pinFrontEn_, OUTPUT);
    coast();
}

void MotorDriver::coast() {
    analogWrite(pinRearEn_, 0);
    analogWrite(pinFrontEn_, 0);
    digitalWrite(pinFwd_, LOW);
    digitalWrite(pinRev_, LOW);
}

void MotorDriver::brake() {
    digitalWrite(pinFwd_, HIGH);
    digitalWrite(pinRev_, HIGH);
    analogWrite(pinRearEn_, 255);
    analogWrite(pinFrontEn_, 255);
}

void MotorDriver::drive(int effort) {
    if (effort == 0) {
        coast();
        return;
    }
    bool forward = effort > 0;
    int rear = constrain(abs(effort), 0, MAX_PWM);
    int front = constrain((int)(rear * frontTrim_ + 0.5f) + frontBoost_, (int)frontMinPwm_, MAX_PWM);

    digitalWrite(pinFwd_, forward ? HIGH : LOW);
    digitalWrite(pinRev_, forward ? LOW : HIGH);
    analogWrite(pinRearEn_, rear);
    analogWrite(pinFrontEn_, front);
}