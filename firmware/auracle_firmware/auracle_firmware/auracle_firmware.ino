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

// If the faster wheel is commanded below this, the Pi is asking for a stop.
// Anything above it keeps both wheels turning (see conditionTargets).


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

// Conditions the left/right pair TOGETHER so a moving robot never has a stationary side
// (a stopped wheel means brake -> coast -> kick restart, which is the vibration and jumping).
// Rule: the yaw demand (r - l) is kept exact, forward speed absorbs the correction.
//   - both wheels near zero            -> real stop
//   - pure spin request (l ~= -r)      -> stays a spin in place, each wheel clamped to MIN..MAX
//   - difference > MAX - MIN           -> counter-rotate (pivot), no wheel can run slower than MIN
//   - otherwise (arc)                  -> shift both wheels together until the slower is at MIN
void conditionTargets(float& l, float& r) {
    if (max(fabs(l), fabs(r)) < CMD_DEADBAND_TICKS_S) { l = r = 0; return; }

    float d = r - l;                       // yaw demand, preserved
    float m = 0.5f * (l + r);              // forward demand
    const float maxDiff = MAX_TICKS_S - MIN_TARGET_TICKS_S;

    if (fabs(m) < 1.0f || fabs(d) > maxDiff) {   // spin or tight turn: counter-rotate
        float h = constrain(fabs(d) * 0.5f, MIN_TARGET_TICKS_S, MAX_TICKS_S);
        float s = (d >= 0) ? 1.0f : -1.0f;
        l = -s * h;
        r =  s * h;
        return;
    }

    // Arc: both wheels turn the same way. Shift them together, keeping r - l exact.
    float dir = (m >= 0) ? 1.0f : -1.0f;
    float a = dir * l, b = dir * r;        // work in the direction of travel
    float lo = min(a, b);
    if (lo < MIN_TARGET_TICKS_S) { float s = MIN_TARGET_TICKS_S - lo; a += s; b += s; }
    float hi = max(a, b);
    if (hi > MAX_TICKS_S)        { float s = hi - MAX_TICKS_S;        a -= s; b -= s; }
    l = dir * a;
    r = dir * b;
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

// A latched fault stops everything until the Pi sends 's' or a zero velocity command.
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
        case 'v': {
            float tl = cmd.a, tr = cmd.b;
            conditionTargets(tl, tr);
            if (faultLatched) {
                if (tl != 0 || tr != 0) break;   // stay stopped while the Pi keeps asking for motion
                faultLatched = false;            // a zero command (key released) clears the fault
                leftPid.clearFault();
                rightPid.clearFault();
            }
            lastCmdMs = millis();
            targetL = tl;
            targetR = tr;
            if (mode != MODE_PID) {
                leftPid.reset();
                rightPid.reset();
                mode = MODE_PID;
            }
            break;
        }
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