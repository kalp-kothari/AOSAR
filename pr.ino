#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <Wire.h>

// =====================================================================
//  Autonomous Kung-Fu Scroll robot
//
//  Mission (no manual input needed):
//    WAIT_VISION -> SCAN (360 deg) -> CENTER -> APPROACH -> ORIENT (gyro)
//    -> HOLD (15 s) -> RELOCATE (move away) -> DONE
//
//  The PC runs the YOLO script and streams lines to this ESP32
//  (WiFi telnet port 23, or USB serial):
//      D,<R|B>,<distance_cm>,<cx_err>     one line per detected class
//      H                                  heartbeat, once per processed frame
//  cx_err = (box centre x - aligned x) / frame width, + = box is to the RIGHT.
//
//  Optional keys (not needed for the mission):  s = emergency stop,  r = restart
// =====================================================================

enum State {
    ST_WAIT_VISION, ST_SCAN, ST_CENTER, ST_APPROACH,
    ST_ORIENT, ST_HOLD, ST_RELOCATE, ST_DONE, ST_FAIL
};

struct Obs {
    float distCm;
    float cxErr;
    unsigned long lastMs;
    unsigned long seq;
};

// ===================== WiFi / Telnet =====================
const char* WIFI_SSID = ":!";
const char* WIFI_PASS = "1234567980";
const char* HOSTNAME  = "robot";
const uint16_t TELNET_PORT = 23;

WiFiServer telnetServer(TELNET_PORT);
WiFiClient telnetClient;

class DualOut : public Print
{
public:
    size_t write(uint8_t c) override
    {
        Serial.write(c);
        if (telnetClient && telnetClient.connected()) telnetClient.write(c);
        return 1;
    }
    size_t write(const uint8_t* buf, size_t size) override
    {
        Serial.write(buf, size);
        if (telnetClient && telnetClient.connected()) telnetClient.write(buf, size);
        return size;
    }
};
DualOut out;

int readInput()
{
    if (Serial.available() > 0) return Serial.read();
    if (telnetClient && telnetClient.connected() && telnetClient.available() > 0)
        return telnetClient.read();
    return -1;
}

void connectWiFi()
{
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(HOSTNAME);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.print("Connecting to WiFi");
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
        delay(300);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("WiFi connected. IP address: ");
        Serial.println(WiFi.localIP());
        if (MDNS.begin(HOSTNAME)) Serial.printf("mDNS: %s.local\n", HOSTNAME);
        telnetServer.begin();
        telnetServer.setNoDelay(true);
        Serial.printf("Telnet server on port %u\n", TELNET_PORT);
    } else {
        Serial.println("WiFi FAILED (check SSID/password, 2.4 GHz only). USB Serial still works.");
    }
}

void handleTelnetClient()
{
    if (WiFi.status() != WL_CONNECTED) return;
    if (telnetServer.hasClient()) {
        if (!telnetClient || !telnetClient.connected()) {
            telnetClient = telnetServer.accept();
            telnetClient.setNoDelay(true);
            out.println("\n[WiFi connected] vision link ready");
        } else {
            WiFiClient extra = telnetServer.accept();
            extra.println("Another client is already connected.");
            extra.stop();
        }
    }
}

// ===================== Motors / encoders =====================
const uint8_t M1_PWM = 25, M1_DIR = 26;
const uint8_t M2_PWM = 27, M2_DIR = 14;
const uint8_t M3_PWM = 22, M3_DIR = 23;

const uint8_t pwmPins[3] = {M1_PWM, M2_PWM, M3_PWM};
const uint8_t dirPins[3] = {M1_DIR, M2_DIR, M3_DIR};

// Flip an entry to -1 if that motor physically spins the wrong way
// for a positive wheel command.
const int MOTOR_SIGN[3] = {+1, +1, +1};

const int NUM_MOTORS = 3;
const uint8_t encoderA[NUM_MOTORS] = {39, 36, 33};
const uint8_t encoderB[NUM_MOTORS] = {34, 35, 32};
volatile long encoderCount[NUM_MOTORS] = {0, 0, 0};

void IRAM_ATTR encoderHandler(void* arg)
{
    int motor = (int)(intptr_t)arg;
    int A = digitalRead(encoderA[motor]);
    int B = digitalRead(encoderB[motor]);
    if (A == B) encoderCount[motor]++;
    else        encoderCount[motor]--;
}

void encoderInit()
{
    for (int i = 0; i < NUM_MOTORS; i++) {
        pinMode(encoderA[i], INPUT_PULLUP);
        pinMode(encoderB[i], INPUT_PULLUP);
        attachInterruptArg(digitalPinToInterrupt(encoderA[i]), encoderHandler,
                           (void*)(intptr_t)i, CHANGE);
    }
}

void printEncoders()
{
    out.printf("Encoders: M1=%ld  M2=%ld  M3=%ld\n",
               encoderCount[0], encoderCount[1], encoderCount[2]);
}

void zeroEncoders()
{
    noInterrupts();
    for (int i = 0; i < NUM_MOTORS; i++) encoderCount[i] = 0;
    interrupts();
}

void setMotor(int i, int speed)
{
    speed = constrain(speed * MOTOR_SIGN[i], -255, 255);
    digitalWrite(dirPins[i], speed >= 0 ? HIGH : LOW);
    analogWrite(pwmPins[i], abs(speed));
}

void stopAll()
{
    for (int i = 0; i < 3; i++) setMotor(i, 0);
}

// ===================== Kinematics =====================
// Robot frame:  +x = FORWARD,  +y = LEFT,  +w = CCW rotation (seen from above).
const float WHEEL_ANGLE_DEG[3] = {120.0f, -120.0f, 0.0f};
float wheelSin[3], wheelCos[3];

void kinematicsInit()
{
    for (int i = 0; i < 3; i++) {
        float a = WHEEL_ANGLE_DEG[i] * DEG_TO_RAD;
        wheelSin[i] = sinf(a);
        wheelCos[i] = cosf(a);
        if (fabsf(wheelSin[i]) < 1e-4f) wheelSin[i] = 0;   // clean up float noise
        if (fabsf(wheelCos[i]) < 1e-4f) wheelCos[i] = 0;
    }
}

// vx, vy, w are normalised, roughly -1..1.  pwmMax is the PWM that a command of
// magnitude 1 maps to.  The fastest wheel is clamped to pwmMax while keeping the
// ratios between wheels (so the direction of motion stays correct).
void drive(float vx, float vy, float w, int pwmMax)
{
    float m[3];
    float maxv = 1.0f;
    for (int i = 0; i < 3; i++) {
        m[i] = -wheelSin[i] * vx + wheelCos[i] * vy + w;
        maxv = max(maxv, fabsf(m[i]));
    }
    for (int i = 0; i < 3; i++) setMotor(i, (int)(m[i] / maxv * pwmMax));
}

// ===================== MPU6050 (yaw from gyro Z) =====================
// NOTE: GPIO22 is used by M3_PWM, so I2C is moved off the default pins.
const uint8_t MPU_SDA = 21;
const uint8_t MPU_SCL = 19;
const uint8_t MPU_ADDR = 0x68;
const float   GYRO_LSB_PER_DPS = 131.0f;  // +/-250 dps range
const int     GYRO_SIGN = +1;             // set to -1 if +yaw should be CCW but isn't

bool  mpuOk = false;
float gyroBias = 0;           // deg/s
float yawDeg = 0;             // integrated heading (unwrapped, + = CCW)
float targetYaw = 0;          // reference heading
unsigned long lastGyroUs = 0;

// Heading control
const float HOLD_KP    = 0.02f;   // normalised w per degree of error
const float HOLD_MAX_W = 0.35f;
const float ALIGN_KP   = 1.2f;    // PWM per degree of error
const int   ALIGN_MIN_PWM = 30;   // overcome motor deadband
const int   ALIGN_MAX_PWM = 60;
const float ALIGN_TOL_DEG = 1.5f;
const unsigned long ALIGN_SETTLE_MS  = 300;
const unsigned long ALIGN_TIMEOUT_MS = 6000;
unsigned long alignStart = 0, alignInTolSince = 0;

void mpuWrite(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

bool mpuReadGyroZ(float& dps)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);                         // GYRO_ZOUT_H
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)MPU_ADDR, 2) != 2) return false;
    int16_t raw = (Wire.read() << 8) | Wire.read();
    dps = raw / GYRO_LSB_PER_DPS;
    return true;
}

float wrap180(float a)
{
    while (a > 180.0f)  a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

float headingError() { return wrap180(targetYaw - yawDeg); }   // + = need to turn CCW

void gyroUpdate()
{
    if (!mpuOk) return;
    unsigned long now = micros();
    float dt = (now - lastGyroUs) * 1e-6f;
    lastGyroUs = now;
    float dps;
    if (!mpuReadGyroZ(dps)) return;
    float rate = (dps - gyroBias) * GYRO_SIGN;
    if (fabsf(rate) < 0.1f) rate = 0;         // small deadband against noise
    if (dt > 0 && dt < 0.2f) yawDeg += rate * dt;
}

void calibrateGyro()
{
    if (!mpuOk) { out.println("MPU6050 not available."); return; }
    stopAll();
    Serial.println("Calibrating gyro - keep the robot STILL...");
    const int N = 500;
    float sum = 0; int got = 0;
    for (int i = 0; i < N; i++) {
        float dps;
        if (mpuReadGyroZ(dps)) { sum += dps; got++; }
        delay(3);
    }
    if (got > 0) gyroBias = sum / got;
    yawDeg = 0; targetYaw = 0;
    lastGyroUs = micros();
    Serial.printf("Gyro bias: %.3f deg/s. Yaw and reference zeroed.\n", gyroBias);
}

void mpuInit()
{
    Wire.begin(MPU_SDA, MPU_SCL);
    Wire.setClock(400000);
    Wire.beginTransmission(MPU_ADDR);
    mpuOk = (Wire.endTransmission() == 0);
    if (!mpuOk) {
        Serial.println("MPU6050 NOT FOUND (check SDA/SCL wiring, address 0x68).");
        return;
    }
    mpuWrite(0x6B, 0x00);   // wake up
    mpuWrite(0x1A, 0x03);   // DLPF ~44 Hz
    mpuWrite(0x1B, 0x00);   // gyro +/-250 dps
    delay(100);
    calibrateGyro();
}

// Gyro-based correction (normalised w) that keeps the robot on targetYaw.
float holdW()
{
    if (!mpuOk) return 0;
    return constrain(HOLD_KP * headingError(), -HOLD_MAX_W, HOLD_MAX_W);
}

// Translate along x (+1 forward, -1 back) while holding the reference heading.
void driveHeld(float vx, int pwm)
{
    drive(vx, 0, holdW(), pwm);
}

// Rotate until the heading error is within tolerance.
// Returns 0 = still running, 1 = aligned, -1 = timed out.
int alignStep(unsigned long timeoutMs)
{
    float err = headingError();
    unsigned long now = millis();

    if (fabsf(err) <= ALIGN_TOL_DEG) {
        stopAll();
        if (alignInTolSince == 0) alignInTolSince = now;
        if (now - alignInTolSince >= ALIGN_SETTLE_MS) {
            out.printf("Aligned. Residual error %.2f deg\n", err);
            return 1;
        }
        return 0;
    }
    alignInTolSince = 0;

    if (now - alignStart > timeoutMs) {
        stopAll();
        out.printf("Align timeout. Residual error %.2f deg\n", err);
        return -1;
    }

    int pwm = (int)constrain(ALIGN_KP * fabsf(err), (float)ALIGN_MIN_PWM, (float)ALIGN_MAX_PWM);
    float w = (err > 0) ? 1.0f : -1.0f;       // +err => rotate CCW (+w)
    drive(0, 0, w, pwm);
    return 0;
}

void beginAlign()
{
    alignStart = millis();
    alignInTolSince = 0;
}

// ===================== Vision link =====================
Obs obs[2] = {{0, 0, 0, 0}, {0, 0, 0, 0}};      // 0 = RED, 1 = BLUE
const char* CLS_NAME[2] = {"RED", "BLUE"};
const float TARGET_CM[2] = {40.0f, 60.0f};       // settle distance per class

const unsigned long OBS_FRESH_MS = 500;         // a detection older than this is "gone"
const unsigned long LINK_DEAD_MS = 1500;        // no packets at all for this long = link lost
const int DETECT_CONFIRM = 2;                   // consecutive frames before scan stops
const int ACCEPT_CLASS = -1;                    // -1 = any, 0 = red only, 1 = blue only

unsigned long linkMs = 0;
bool linkEver = false;
int detStreak[2] = {0, 0};
unsigned long hSeq[2] = {0, 0};

bool obsFresh(int c)
{
    return obs[c].lastMs != 0 && millis() - obs[c].lastMs < OBS_FRESH_MS;
}

bool visionAlive()
{
    return linkEver && millis() - linkMs < LINK_DEAD_MS;
}

// Parse one complete line from the PC.
void handleLine(const char* s)
{
    if (s[0] == 'D') {
        char c = 0; float d = 0, x = 0;
        if (sscanf(s, "D,%c,%f,%f", &c, &d, &x) == 3) {
            int i = (c == 'R') ? 0 : (c == 'B') ? 1 : -1;
            if (i >= 0 && d > 5.0f && d < 300.0f) {
                obs[i].distCm = d;
                obs[i].cxErr  = x;
                obs[i].lastMs = millis();
                obs[i].seq++;
            }
        }
        linkMs = millis(); linkEver = true;
    } else if (s[0] == 'H') {
        linkMs = millis(); linkEver = true;
        // consecutive-frame detection streak per class
        for (int i = 0; i < 2; i++) {
            if (obs[i].seq != hSeq[i]) { detStreak[i]++; hSeq[i] = obs[i].seq; }
            else detStreak[i] = 0;
        }
    }
}

// ===================== Mission state =====================
State state = ST_WAIT_VISION;
unsigned long stateSince = 0;
unsigned long lostSince = 0;
int lockCls = -1;               // class we are going for (0 red, 1 blue)

// scan
const int   SCAN_PWM = 38;
const float SCAN_FULL_TURN_DEG = 360.0f;
const int   SCAN_MAX_SWEEPS = 3;
float scanStartYaw = 0;
int   scanSweeps = 0;

// centre
const float CENTER_TOL = 0.03f;                 // fraction of frame width
const int   CENTER_CONFIRM = 3;
const int   CENTER_PWM_MIN = 30, CENTER_PWM_MAX = 50;
const float CENTER_KP = 200.0f;                 // PWM per unit of cx_err
const unsigned long CENTER_PULSE_MS = 120;
const float VIS_SIGN = +1.0f;                   // flip to -1 if the robot turns AWAY from the box
int centerOk = 0;

// approach
const int   APPROACH_PWM = 60;
const int   CREEP_PWM = 40;
const float SLOW_ZONE_CM = 15.0f;               // switch to stop-and-measure inside this
const float DIST_TOL_FRAC = 0.04f;              // +/-4% distance (== +/-8% box area)
const int   DIST_OK_CONFIRM = 3;
const float REJOIN_CX = 0.10f;                  // drifted this far off-centre -> re-centre
const float PULSE_MS_PER_CM = 30.0f;            // tune: 1000 / (cm per second at CREEP_PWM)
const long  PULSE_MIN_MS = 80, PULSE_MAX_MS = 350;
const unsigned long SETTLE_MS = 250;            // stand still this long before trusting a reading
const unsigned long APPROACH_TIMEOUT_MS = 45000;
unsigned long approachStart = 0;
int  distOk = 0;
bool farDriving = false;

// orient / verify
bool orientVerify = false;
int  verifyFails = 0;
const int MAX_VERIFY_RETRIES = 3;

// hold
const unsigned long HOLD_MS = 15000;
const float HOLD_REALIGN_DEG = 3.0f;
unsigned long holdStart = 0;
int  holdLastPrintS = -1;
bool holdAligning = false;

// relocate
const unsigned long RELOCATE_BACK_MS = 1500;
const float         RELOCATE_TURN_DEG = -90.0f;     // negative = clockwise
const unsigned long RELOCATE_FWD_MS = 1200;
const unsigned long RELOCATE_TURN_TIMEOUT_MS = 12000;
int relocStage = 0;
unsigned long relocStart = 0;

// timeouts
const unsigned long TARGET_LOST_MS = 1500;
const unsigned long LINK_FAIL_MS = 8000;
const unsigned long START_DELAY_MS = 2000;
unsigned long visionSeenAt = 0;

// pulses (move briefly, stop, wait for a still camera reading)
bool pulsing = false;
unsigned long pulseEnd = 0;
float pulseVx = 0, pulseW = 0;
int   pulsePwm = 0;
bool  pulseHold = false;
unsigned long stopMs = 0, seqAtStop = 0, lastDecisionSeq = 0;

const char* stateName(State s)
{
    switch (s) {
        case ST_WAIT_VISION: return "WAIT_VISION";
        case ST_SCAN:        return "SCAN";
        case ST_CENTER:      return "CENTER";
        case ST_APPROACH:    return "APPROACH";
        case ST_ORIENT:      return "ORIENT";
        case ST_HOLD:        return "HOLD";
        case ST_RELOCATE:    return "RELOCATE";
        case ST_DONE:        return "DONE";
        default:             return "FAIL";
    }
}

void setState(State s)
{
    state = s;
    stateSince = millis();
    lostSince = 0;
    out.printf("[STATE] %s\n", stateName(s));
}

void fail(const char* why)
{
    stopAll();
    pulsing = false;
    out.printf("FAIL: %s\n", why);
    setState(ST_FAIL);
}

void markStopped()
{
    stopMs = millis();
    seqAtStop = (lockCls >= 0) ? obs[lockCls].seq : 0;
}

void applyPulse()
{
    float w = pulseHold ? holdW() : pulseW;
    drive(pulseVx, 0, w, pulsePwm);
}

void startPulse(float vx, float w, int pwm, unsigned long ms, bool holdYaw)
{
    pulsing = true;
    pulseEnd = millis() + ms;
    pulseVx = vx; pulseW = w; pulsePwm = pwm; pulseHold = holdYaw;
    applyPulse();
}

void endPulse()
{
    pulsing = false;
    stopAll();
    markStopped();
}

// True once per new camera reading taken while the robot is standing still.
bool newStillReading()
{
    if (pulsing || lockCls < 0) return false;
    if (obs[lockCls].seq == lastDecisionSeq) return false;
    if (millis() - stopMs < SETTLE_MS) return false;
    if (obs[lockCls].seq - seqAtStop < 2) return false;
    lastDecisionSeq = obs[lockCls].seq;
    return true;
}

bool targetVisible()
{
    return lockCls >= 0 && obsFresh(lockCls);
}

void startScan();
void startCenter();

// Safety wrapper for states that need the camera. Returns true if the target
// is currently visible; otherwise stops the robot and handles the timeouts.
bool trackTarget()
{
    unsigned long now = millis();
    if (!visionAlive()) {
        stopAll(); pulsing = false; farDriving = false;
        if (lostSince == 0) { lostSince = now; out.println("Vision link lost - stopped."); }
        if (now - lostSince > LINK_FAIL_MS) fail("vision link lost");
        return false;
    }
    if (!targetVisible()) {
        if (lostSince == 0) { lostSince = now; stopAll(); pulsing = false; farDriving = false; }
        if (now - lostSince > TARGET_LOST_MS) {
            out.println("Target lost - scanning again.");
            startScan();
        }
        return false;
    }
    lostSince = 0;
    return true;
}

// ===================== Mission steps =====================
void startScan()
{
    stopAll();
    pulsing = false; farDriving = false;
    lockCls = -1;
    scanStartYaw = yawDeg;
    setState(ST_SCAN);
}

void scanStep()
{
    if (!visionAlive()) { stopAll(); return; }

    for (int c = 0; c < 2; c++) {
        if ((ACCEPT_CLASS < 0 || ACCEPT_CLASS == c) && obsFresh(c) && detStreak[c] >= DETECT_CONFIRM) {
            stopAll();
            lockCls = c;
            out.printf("Detected %s box: %.1f cm, offset %+.3f, yaw %.1f\n",
                       CLS_NAME[c], obs[c].distCm, obs[c].cxErr, yawDeg);
            startCenter();
            return;
        }
    }

    float turned = fabsf(yawDeg - scanStartYaw);
    if (turned >= SCAN_FULL_TURN_DEG) {
        stopAll();
        scanSweeps++;
        if (scanSweeps >= SCAN_MAX_SWEEPS) { fail("no box found after full sweeps"); return; }
        out.printf("Sweep %d complete, nothing found. Scanning again.\n", scanSweeps);
        scanStartYaw = yawDeg;
        return;
    }
    drive(0, 0, +1.0f, SCAN_PWM);               // rotate CCW
}

void startCenter()
{
    stopAll();
    pulsing = false; farDriving = false;
    centerOk = 0;
    markStopped();
    lastDecisionSeq = obs[lockCls].seq;
    setState(ST_CENTER);
}

void startApproach()
{
    stopAll();
    pulsing = false; farDriving = false;
    distOk = 0;
    approachStart = millis();
    markStopped();
    lastDecisionSeq = obs[lockCls].seq;
    setState(ST_APPROACH);
}

void centerStep()
{
    if (pulsing) return;
    if (!trackTarget()) return;
    if (!newStillReading()) return;

    float e = obs[lockCls].cxErr;
    if (fabsf(e) <= CENTER_TOL) {
        if (++centerOk >= CENTER_CONFIRM) {
            targetYaw = yawDeg;                 // reference heading = facing the box
            out.printf("Centered on %s box. Reference yaw %.1f\n", CLS_NAME[lockCls], targetYaw);
            startApproach();
        }
        return;
    }
    centerOk = 0;
    int pwm = (int)constrain(CENTER_KP * fabsf(e), (float)CENTER_PWM_MIN, (float)CENTER_PWM_MAX);
    float w = ((e > 0) ? -1.0f : 1.0f) * VIS_SIGN;     // box on the right -> rotate CW (-w)
    startPulse(0, w, pwm, CENTER_PULSE_MS, false);
}

void startOrient()
{
    stopAll();
    pulsing = false; farDriving = false;
    orientVerify = false;
    beginAlign();
    setState(ST_ORIENT);
}

void approachStep()
{
    if (pulsing) return;
    if (millis() - approachStart > APPROACH_TIMEOUT_MS) { fail("approach timeout"); return; }
    if (!trackTarget()) return;

    float target = TARGET_CM[lockCls];
    float err = obs[lockCls].distCm - target;           // + = too far
    float tol = target * DIST_TOL_FRAC;
    float cx  = obs[lockCls].cxErr;

    if (err > SLOW_ZONE_CM) {
        // Far: drive continuously, gyro holds the heading.
        if (fabsf(cx) > REJOIN_CX) {
            out.println("Drifted off-centre - re-centering.");
            startCenter();
            return;
        }
        farDriving = true;
        distOk = 0;
        driveHeld(1.0f, APPROACH_PWM);
        return;
    }

    // Near: stop, read, creep in short pulses.
    if (farDriving) { stopAll(); farDriving = false; markStopped(); }
    if (!newStillReading()) return;

    if (fabsf(err) <= tol) {
        if (++distOk >= DIST_OK_CONFIRM) {
            out.printf("Distance OK: %.1f cm (target %.0f cm). Checking orientation.\n",
                       obs[lockCls].distCm, target);
            startOrient();
        }
        return;
    }
    distOk = 0;
    if (fabsf(cx) > REJOIN_CX) { startCenter(); return; }

    float sgn = (err > 0) ? 1.0f : -1.0f;
    long ms = constrain((long)(fabsf(err) * PULSE_MS_PER_CM), PULSE_MIN_MS, PULSE_MAX_MS);
    startPulse(sgn, 0, CREEP_PWM, (unsigned long)ms, true);
}

void startHold()
{
    stopAll();
    pulsing = false; farDriving = false;
    holdStart = millis();
    holdLastPrintS = -1;
    holdAligning = false;
    setState(ST_HOLD);
    out.printf("Holding position for %lu s\n", HOLD_MS / 1000);
}

void orientStep()
{
    if (!orientVerify) {
        if (!mpuOk) { orientVerify = true; markStopped(); lastDecisionSeq = obs[lockCls].seq; return; }
        int r = alignStep(ALIGN_TIMEOUT_MS);
        if (r == 0) return;
        if (r < 0) out.println("Gyro align timed out - continuing with residual error.");
        orientVerify = true;
        markStopped();
        lastDecisionSeq = obs[lockCls].seq;
        return;
    }

    // Final camera check once the robot is facing the reference heading.
    if (!trackTarget()) return;
    if (!newStillReading()) return;

    float target = TARGET_CM[lockCls];
    float err = obs[lockCls].distCm - target;
    float tol = target * DIST_TOL_FRAC;

    if (verifyFails < MAX_VERIFY_RETRIES) {
        if (fabsf(err) > 2.0f * tol) {
            verifyFails++;
            out.printf("Distance %.1f cm after orient - approaching again.\n", obs[lockCls].distCm);
            startApproach();
            return;
        }
        if (fabsf(obs[lockCls].cxErr) > REJOIN_CX) {
            verifyFails++;
            out.println("Off-centre after orient - re-centering.");
            startCenter();
            return;
        }
    }
    startHold();
}

void startRelocate()
{
    stopAll();
    relocStage = 0;
    relocStart = millis();
    setState(ST_RELOCATE);
    out.println("Hold finished. Moving away from the box.");
}

void holdStep()
{
    unsigned long el = millis() - holdStart;
    if (el >= HOLD_MS) { startRelocate(); return; }

    int remaining = (int)((HOLD_MS - el) / 1000) + 1;
    if (remaining != holdLastPrintS) {
        holdLastPrintS = remaining;
        out.printf("Hold: %d s left (heading error %.2f deg)\n", remaining, headingError());
    }

    // Keep the heading correct if the robot gets nudged or the gyro drifts.
    if (holdAligning) {
        if (alignStep(ALIGN_TIMEOUT_MS) != 0) holdAligning = false;
        return;
    }
    if (mpuOk && fabsf(headingError()) > HOLD_REALIGN_DEG) {
        holdAligning = true;
        beginAlign();
    }
}

void finishMission()
{
    stopAll();
    pulsing = false;
    printEncoders();
    out.println("Mission complete. Send 'r' to run again.");
    setState(ST_DONE);
}

void relocateStep()
{
    switch (relocStage) {
        case 0:                                   // back away, keep heading
            if (millis() - relocStart >= RELOCATE_BACK_MS) {
                stopAll();
                targetYaw += RELOCATE_TURN_DEG;
                beginAlign();
                relocStage = 1;
            } else {
                driveHeld(-1.0f, APPROACH_PWM);
            }
            break;
        case 1:                                   // turn to the new heading
            if (alignStep(RELOCATE_TURN_TIMEOUT_MS) != 0) {
                relocStage = 2;
                relocStart = millis();
            }
            break;
        default:                                  // drive off a bit
            if (millis() - relocStart >= RELOCATE_FWD_MS) finishMission();
            else driveHeld(1.0f, APPROACH_PWM);
            break;
    }
}

void waitVisionStep()
{
    static unsigned long lastMsg = 0;
    if (!mpuOk) { fail("MPU6050 missing - cannot scan"); return; }

    if (visionAlive()) {
        if (visionSeenAt == 0) { visionSeenAt = millis(); out.println("Vision link is up. Starting shortly."); }
        if (millis() - visionSeenAt >= START_DELAY_MS) {
            scanSweeps = 0;
            verifyFails = 0;
            startScan();
        }
        return;
    }
    visionSeenAt = 0;
    if (millis() - lastMsg > 2000) {
        lastMsg = millis();
        out.println("Waiting for the vision link (run the Python script)...");
    }
}

void emergencyStop()
{
    stopAll();
    pulsing = false; farDriving = false;
    out.println("EMERGENCY STOP");
    setState(ST_DONE);
}

void restartMission()
{
    stopAll();
    pulsing = false; farDriving = false;
    lockCls = -1;
    visionSeenAt = 0;
    scanSweeps = 0;
    verifyFails = 0;
    out.println("Restarting mission.");
    setState(ST_WAIT_VISION);
}

// ===================== Input handling =====================
char lineBuf[64];
int  lineLen = 0;

void feedChar(char c)
{
    if (lineLen == 0 && c == 's') { emergencyStop(); return; }
    if (lineLen == 0 && c == 'r') { restartMission(); return; }

    if (c == '\n' || c == '\r') {
        if (lineLen > 0) { lineBuf[lineLen] = 0; handleLine(lineBuf); lineLen = 0; }
        return;
    }
    if (lineLen < (int)sizeof(lineBuf) - 1) lineBuf[lineLen++] = c;
    else lineLen = 0;                           // overflow: drop the line
}

void printStatus()
{
    static unsigned long last = 0;
    if (millis() - last < 1000) return;
    last = millis();
    out.printf("[%s] yaw=%.1f ref=%.1f", stateName(state), yawDeg, targetYaw);
    for (int c = 0; c < 2; c++) {
        if (obsFresh(c)) out.printf("  %s=%.1fcm/%+.2f", CLS_NAME[c], obs[c].distCm, obs[c].cxErr);
    }
    out.println();
}

// ===================== Arduino =====================
void setup()
{
    Serial.begin(115200);
    for (int i = 0; i < NUM_MOTORS; i++) {
        pinMode(pwmPins[i], OUTPUT);
        pinMode(dirPins[i], OUTPUT);
    }
    kinematicsInit();
    stopAll();
    encoderInit();
    mpuInit();
    connectWiFi();
    delay(300);
    setState(ST_WAIT_VISION);
}

void loop()
{
    handleTelnetClient();
    gyroUpdate();

    int ch;
    while ((ch = readInput()) != -1) feedChar((char)ch);

    if (pulsing) {
        if (millis() >= pulseEnd) endPulse();
        else applyPulse();
    }

    switch (state) {
        case ST_WAIT_VISION: waitVisionStep(); break;
        case ST_SCAN:        scanStep();       break;
        case ST_CENTER:      centerStep();     break;
        case ST_APPROACH:    approachStep();   break;
        case ST_ORIENT:      orientStep();     break;
        case ST_HOLD:        holdStep();       break;
        case ST_RELOCATE:    relocateStep();   break;
        default:             break;            // DONE / FAIL: motors stay off
    }

    printStatus();
    delay(1);
}
