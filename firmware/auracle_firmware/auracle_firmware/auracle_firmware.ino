#include <avr/wdt.h>
#include "config.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "pid_controller.h"
#include "imu_driver.h"
#include "serial_protocol.h"

// Clear the WDT flag right after reset, or older bootloaders reset-loop.
uint8_t mcusr_mirror __attribute__((section(".noinit")));
void get_mcusr(void) __attribute__((naked, used, section(".init3")));
void get_mcusr(void) {
    mcusr_mirror = MCUSR;
    MCUSR = 0;
    wdt_disable();
}

// MODE_PID with zero targets is "stopped": the controllers brake (if still rolling) and coast.
enum Mode : uint8_t { MODE_PID, MODE_OPEN_LOOP };

const WheelParams LB_PARAMS = {LB_KICK_FWD, LB_KICK_REV, LB_FLOOR_FWD, LB_FLOOR_REV, LB_KP, LB_KI,
                               LB_FF_FWD_BASE, LB_FF_FWD_K, LB_FF_REV_BASE, LB_FF_REV_K};
const WheelParams RB_PARAMS = {RB_KICK_FWD, RB_KICK_REV, RB_FLOOR_FWD, RB_FLOOR_REV, RB_KP, RB_KI,
                               RB_FF_FWD_BASE, RB_FF_FWD_K, RB_FF_REV_BASE, RB_FF_REV_K};

MotorDriver leftSide(PIN_L_FWD, PIN_L_REV, PIN_LB_EN, PIN_LF_EN, LF_TRIM, LF_MIN_PWM);
MotorDriver rightSide(PIN_R_FWD, PIN_R_REV, PIN_RB_EN, PIN_RF_EN, RF_TRIM, RF_MIN_PWM);
EncoderDriver leftEnc(PIN_LB_ENC_INT, PIN_LB_ENC_DIR, LB_ENC_SIGN);
EncoderDriver rightEnc(PIN_RB_ENC_INT, PIN_RB_ENC_DIR, RB_ENC_SIGN);
PIDController leftPid(LB_PARAMS), rightPid(RB_PARAMS);
IMUDriver imu;
SerialProtocol link;

Mode mode = MODE_PID;
bool faultLatched = false;
float targetL = 0, targetR = 0;  // 1x ticks/s
int pwmL = 0, pwmR = 0;          // open loop
unsigned long lastCmdMs = 0, lastCtrlMs = 0, lastSampleUs = 0;
long lastCountL = 0, lastCountR = 0;
int16_t imuRaw[6] = {0, 0, 0, 0, 0, 0};

void isrLeft() { leftEnc.tick(); }
void isrRight() { rightEnc.tick(); }

float clampTarget(float t) {
    if (fabs(t) < MIN_TARGET_TICKS_S) return 0.0f;
    return constrain(t, -MAX_TICKS_S, MAX_TICKS_S);
}

// Zero the targets and cut the motors now. A wheel that is still rolling gets a short brake
// from the controller on the next control period, then coasts.
void stopAll() {
    mode = MODE_PID;
    targetL = targetR = 0;
    pwmL = pwmR = 0;
    leftSide.coast();
    rightSide.coast();
}

void apply(MotorDriver& side, const DriveCmd& c) {
    switch (c.mode) {
        case DRIVE_RUN:   side.drive(c.effort); break;
        case DRIVE_BRAKE: side.brake(); break;
        default:          side.coast(); break;
    }
}

// A latched fault stops everything until the Pi sends 's'.
void checkFaults() {
    Fault fl = leftPid.fault(), fr = rightPid.fault();
    if (fl == FAULT_NONE && fr == FAULT_NONE) return;
    faultLatched = true;
    stopAll();
    if (REPORT_FAULTS) {
        if (fl != FAULT_NONE) link.sendFault('L', fl);
        if (fr != FAULT_NONE) link.sendFault('R', fr);
    }
}

void handle(const Command& cmd) {
    switch (cmd.type) {
        case 'v':
            if (faultLatched) break;
            lastCmdMs = millis();
            targetL = clampTarget(cmd.a);
            targetR = clampTarget(cmd.b);
            if (mode != MODE_PID) {
                leftPid.reset();
                rightPid.reset();
                mode = MODE_PID;
            }
            break;
        case 'o':
            if (faultLatched) break;
            lastCmdMs = millis();
            pwmL = constrain((int)cmd.a, -MAX_PWM, MAX_PWM);
            pwmR = constrain((int)cmd.b, -MAX_PWM, MAX_PWM);
            targetL = targetR = 0;
            if (mode != MODE_OPEN_LOOP) {
                leftPid.reset();
                rightPid.reset();
                mode = MODE_OPEN_LOOP;
            }
            break;
        case 's':
            faultLatched = false;
            leftPid.clearFault();
            rightPid.clearFault();
            stopAll();
            break;
    }
}

void setup() {
    MotorDriver::configurePwmTimers();  // Timer1 -> ~976 Hz, before any analogWrite
    leftSide.begin();
    rightSide.begin();
    link.begin();

    leftEnc.begin();
    rightEnc.begin();
    attachInterrupt(digitalPinToInterrupt(PIN_LB_ENC_INT), isrLeft, CHANGE);   // 2x decoding
    attachInterrupt(digitalPinToInterrupt(PIN_RB_ENC_INT), isrRight, CHANGE);

    imu.begin();
    imu.calibrateGyro(200);  // ~0.5 s, robot must be still

    wdt_enable(WDTO_2S);  // after calibration so it can't trip it
    lastCmdMs = lastCtrlMs = millis();
    lastSampleUs = micros();
    link.sendReady();
}

void loop() {
    wdt_reset();

    Command cmd;
    while (link.poll(cmd)) handle(cmd);

    unsigned long now = millis();
    bool driving = (mode == MODE_OPEN_LOOP) || targetL != 0 || targetR != 0;
    if (driving && now - lastCmdMs > COMMS_TIMEOUT_MS) stopAll();

    if (now - lastCtrlMs < CONTROL_PERIOD_MS) return;
    lastCtrlMs += CONTROL_PERIOD_MS;
    if (now - lastCtrlMs > CONTROL_PERIOD_MS) lastCtrlMs = now;  // fell behind - don't burst

    unsigned long nowUs = micros();
    float dt = (nowUs - lastSampleUs) * 1e-6f;
    lastSampleUs = nowUs;

    long countL = leftEnc.count();   // 2x counts
    long countR = rightEnc.count();
    float rawL = (countL - lastCountL) * COUNT_TO_TICK / dt;   // 1x ticks/s
    float rawR = (countR - lastCountR) * COUNT_TO_TICK / dt;
    lastCountL = countL;
    lastCountR = countR;

    if (mode == MODE_PID) {
        apply(leftSide, leftPid.update(targetL, rawL, dt, now));
        apply(rightSide, rightPid.update(targetR, rawR, dt, now));
        if (!faultLatched) checkFaults();
    } else {
        leftPid.observe(rawL);
        rightPid.observe(rawR);
        leftSide.drive(pwmL);
        rightSide.drive(pwmR);
    }

    imu.read(imuRaw);
    if (TELEMETRY_RAW_2X) link.sendTelemetry(now, countL, countR, imuRaw);
    else                  link.sendTelemetry(now, countL >> 1, countR >> 1, imuRaw);   // 1x-equivalent
}