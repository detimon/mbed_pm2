// CargoSweep — PROTOTYPE_03_V27
// Clean rewrite. Uses ServoFeedback360 (Parallax 360°) instead of mechanical
// endstop + bit-bang servo. State machine has 20 states across the intro
// sequence, the wide-bar pickup phase and the narrow-line delivery phase.
#include "test_config.h"

#ifdef PROTOTYPE_03_V27

#include "prototype03_v27.h"
#include "ColorSensor.h"
#include "DCMotor.h"
#include "LineFollower.h"
#include "NeoPixel.h"
#include "Servo.h"
#include "ServoFeedback360.h"
#include <cmath>

// ===========================================================================
// Hardware constants (Drahtzugliste V10)
// ===========================================================================
//
// DC-Motor M10 (RIGHT): PWM=PB_13, ENC_A=PB_6, ENC_B=PB_7
// DC-Motor M11 (LEFT):  PWM=PA_9,  ENC_A=PA_6, ENC_B=PC_7
// Motor enable: PB_15
// Servo M21 (vertical, 180°):   PB_2  (D0)
// Servo M20 (horizontal, 180°): PC_8  (D1)
// Servo M22 (Parallax 360° FB): PWM=PB_12 (D3), feedback=PC_2 (A0, ~910 Hz)
// SensorBar B1 (I2C1): SDA=PB_9, SCL=PB_8
// ColorSensor B4 (TCS3200): OUT=PB_3, LED=PC_3, S0=PA_1, S1=PA_4, S2=PC_1, S3=PC_0
// NeoPixel P1: PC_5 (A2)

// ===========================================================================
// Tunables — values harvested from V23 (kalibriert)
// ===========================================================================

// Motor / geometry (Gear 100:1, KN = 140/12)
static const float GEAR_RATIO  = 100.0f;
static const float KN          = 140.0f / 12.0f;
static const float VOLTAGE_MAX = 12.0f;
static const float VEL_SIGN    = 1.0f;

static const float D_WHEEL  = 0.0291f;
static const float B_WHEEL  = 0.1493f;
static const float BAR_DIST = 0.182f;

// Line-follower PID (V23 verified)
static const float KP           = 2.8f;
static const float KP_NL        = 4.55f;
static const float KP_FOLLOW    = 1.1f;   // soft entry gains after TURN_SPOT
static const float KP_NL_FOLLOW = 0.7f;
static const float MAX_SPEED    = 1.6f;
static const float SPEED_SCALE  = 1.0f / MAX_SPEED;

// Drive speeds
static const float BLIND_SPEED    = 1.0f;
static const float STRAIGHT_SPEED = 1.0f;
static const float TURN_SPEED     = 0.5f;
static const float BACKWARD_SPEED = 1.0f;
static const float APPROACH_SPEED = 0.5f;

// Intro-sequence timing
static const int STRAIGHT_LOOPS       = 70;   // 1.4 s
static const int FOLLOW_1S_LOOPS      = static_cast<int>(50 * SPEED_SCALE);
static const int BRAKE_LOOPS          = 12;
static const int PAUSE_LOOPS          = 20;   // 0.4 s
static const int BACKWARD_LOOPS       = 222;  // 4.44 s
static const int ACCEL_LOOPS          = 12;
static const int RESTART_ACCEL_LOOPS  = 50;
static const int START_GUARD          = static_cast<int>(300 * SPEED_SCALE);
static const int STOP_GUARD           = static_cast<int>(75  * SPEED_SCALE);

// Real-program timing
static const int   TOTAL_CROSSINGS         = 4;
static const int   TOTAL_SMALL_CROSSINGS   = 4;
static const int   CROSSING_STOP_LOOPS     = 50;   // 1 s — colour read at standstill
static const int   COLOR_READ_PHASE        = 25;   // 0.5 s
static const int   COLOR_STABLE_CNT        = 3;    // driving stable count (60 ms)
static const int   COLOR_STOP_STABLE_CNT   = 3;    // standstill stable count
static const int   COLOR_READ_DELAY        = 50;   // 1.0 s sensor lockout after restart
static const int   SLOWDOWN_LOOPS          = 13;
static const float SLOW_FACTOR             = 0.2f;
static const int   SMALL_FOLLOW_START_GUARD = static_cast<int>(781 * SPEED_SCALE);
static const int   SMALL_REENTRY_GUARD      = static_cast<int>(100 * SPEED_SCALE);

// Drive-past distance (~120 mm) for ROT/GELB
static const int ROT_GELB_DRIVE_LOOPS = 55;
static const int ROT_GELB_ACCEL_LOOPS = 10;
static const int ROT_GELB_BRAKE_LOOPS = 12;

// 180° servo positions (V23 verified)
static const float D1_RETRACTED       = 0.0f;
static const float D1_EXTENDED        = 0.95f;
static const float D2_UP              = 1.0f;
static const float D2_PARTIAL_DOWN    = 0.85f;     // ~10 mm
static const float D2_DOWN_BLAU       = 0.38f;
static const float D2_DOWN_GRUEN      = 0.22f;

// PICKUP / DELIVER timing
static const int PICKUP_PHASE1_LOOPS = 30;   // 0.6 s — drehteller +50°
static const int PICKUP_PHASE2_LOOPS = 25;   // 0.5 s — D2 partial down
static const int PICKUP_PHASE3_LOOPS = 30;   // 0.6 s — drehteller back to target
static const int PICKUP_PHASE4_LOOPS = 25;   // 0.5 s — D2 full down
static const int JIGGLE_LOOPS        = 75;   // 1.5 s — 2D jiggle (one cycle)
static const int RAISE_LOOPS         = 30;   // 0.6 s — arm back up + retract
static const float PICKUP_OFFSET_DEG = 50.0f;
static const float JIGGLE_AMPL_DEG   = 5.0f;
static const float D1_JIGGLE_OUT     = 1.00f;
static const float D1_JIGGLE_IN      = 0.90f;

// Sensor thresholds
static const float SENSOR_THRESHOLD = 0.40f;

// Parallax 360° servo tuning (from TEST_PARALLAX_360)
static const float SF360_KP        = 0.005f;
static const float SF360_TOL_DEG   = 2.1f;
static const float SF360_MIN_SPEED = 0.01f;
static const float SF360_OFFSET    = 70.0f;
static const int   SF360_WARMUP_LOOPS  = 25;   // 500 ms feedback stabilisation
static const int   SF360_TIMEOUT_LOOPS = 250;  // 5 s max wait (like TEST_PARALLAX_360)

// Colour-to-slot mapping
//   colour codes: 3=ROT, 4=GELB, 5=GRÜN, 7=BLAU
//   slot index : 0=ROT, 1=GRÜN, 2=BLAU, 3=GELB
//   target angle in degrees, CW only:
//     ROT=0°, GRÜN=90°, BLAU=180°, GELB=270°
static const float COLOR_ANGLE[4] = {0.0f, 90.0f, 180.0f, 270.0f};

// ===========================================================================
// State machine
// ===========================================================================
enum State {
    STATE_BLIND                 = 1,
    STATE_STRAIGHT              = 2,
    STATE_TURN_SPOT             = 3,
    STATE_FOLLOW                = 4,
    STATE_FOLLOW_1S             = 5,
    STATE_BRAKE                 = 6,
    STATE_PAUSE                 = 7,
    STATE_BACKWARD              = 8,
    STATE_START_PAUSE           = 9,
    STATE_APPROACH              = 10,
    STATE_LINE_FOLLOW           = 11,
    STATE_COLOUR_ASSUMPTION     = 12,
    STATE_CROSSING_STOP         = 13,
    STATE_COLOUR_DRIVE_PAST     = 14,
    STATE_PICKUP                = 15,
    STATE_SMALL_FOLLOW          = 16,
    STATE_SMALL_COLOUR_ASSUMPTION = 17,
    STATE_SMALL_CROSSING_STOP   = 18,
    STATE_DELIVER               = 19,
    STATE_FINAL_HALT            = 20
};

// ===========================================================================
// Hardware pointers (static-local instantiated in init)
// ===========================================================================
static DCMotor*          g_M10        = nullptr;  // physical RIGHT
static DCMotor*          g_M11        = nullptr;  // physical LEFT
static DigitalOut*       g_en         = nullptr;
static LineFollower*     g_lf         = nullptr;
static ColorSensor*      g_cs         = nullptr;
static Servo*            g_servoVert  = nullptr;  // M21 (PB_2)
static Servo*            g_servoHoriz = nullptr;  // M20 (PC_8)
static ServoFeedback360* g_servoTray  = nullptr;  // M22 (PB_12 / PC_2)
static NeoPixel*         g_np         = nullptr;

// ===========================================================================
// Runtime state
// ===========================================================================
static State m_state = STATE_BLIND;

// 360° feedback servo
static int  m_sf360_warmup_ctr = 0;
static bool m_sf360_ready      = false;

// Slot tracking (colour index 0=ROT, 1=GRÜN, 2=BLAU, 3=GELB)
static bool m_slots[4] = {false, false, false, false};

// Counters
static int m_straight_ctr      = 0;
static int m_follow_ctr        = 0;
static int m_brake_ctr         = 0;
static int m_pause_ctr         = 0;
static int m_backward_ctr      = 0;
static int m_guard_ctr         = 0;
static int m_approach_ctr      = 0;
static int m_accel_ctr         = 0;     // restart-acceleration ramp
static int m_crossing_ctr      = 0;
static int m_small_crossing_ctr = 0;
static int m_drive_past_ctr    = 0;
static int  m_tray_timeout_ctr  = 0;
static bool m_tray_moving       = false; // true while P-controller should run
static int m_phase_ctr         = 0;     // generic per-state internal counter
static int m_phase_idx         = 0;     // generic phase index inside PICKUP/DELIVER

static int m_crossings_left       = 0;
static int m_small_crossings_left = 0;

static float m_brake_start_M10 = 0.0f;
static float m_brake_start_M11 = 0.0f;
static float m_cmd_M10         = 0.0f;
static float m_cmd_M11         = 0.0f;

// Colour reading
static int  m_current_color    = 0;
static int  m_color_pending    = 0;
static int  m_color_stable     = 0;
static int  m_color_delay_ctr  = 0;
static bool m_slowing          = false;
static int  m_slow_ctr         = 0;
static int  m_action_color     = 0;     // colour confirmed at the bar/line
static int  m_color_fallback   = 0;     // last driving-phase colour
static bool m_assumption_set   = false; // true while drehteller is pre-rotating
static int  m_assumption_color = 0;

// Pickup/Deliver dispatch
static bool  m_is_delivering   = false; // true on the deliver leg, false on pickup leg
static float m_target_angle    = 0.0f;
static int   m_target_slot     = -1;
static bool  m_needs_drive_past = false;

// LED blink helpers
static int m_led_ctr   = 0;
static int m_led_color = 0;

// ===========================================================================
// Helpers
// ===========================================================================
static int colorToSlot(int c)
{
    switch (c) {
        case 3: return 0; // ROT
        case 5: return 1; // GRÜN
        case 7: return 2; // BLAU
        case 4: return 3; // GELB
        default: return -1;
    }
}

static bool isActionColor(int c)
{
    return c == 3 || c == 4 || c == 5 || c == 7;
}

static int activeSensorCount()
{
    int n = 0;
    for (int i = 0; i < 8; i++) {
        if (g_lf->getAvgBit(i) >= SENSOR_THRESHOLD) n++;
    }
    return n;
}

static bool sevenOfEightActive()  { return activeSensorCount() >= 7; }
static bool wideBarActive()       { return activeSensorCount() >= 4; }
static bool centerSensorsActive() { return g_lf->getAvgBit(3) >= SENSOR_THRESHOLD ||
                                           g_lf->getAvgBit(4) >= SENSOR_THRESHOLD; }
static bool smallLineActive()
{
    return (g_lf->getAvgBit(3) >= SENSOR_THRESHOLD &&
            g_lf->getAvgBit(4) >= SENSOR_THRESHOLD &&
            g_lf->getAvgBit(5) >= SENSOR_THRESHOLD) ||
           (g_lf->getAvgBit(2) >= SENSOR_THRESHOLD &&
            g_lf->getAvgBit(3) >= SENSOR_THRESHOLD &&
            g_lf->getAvgBit(4) >= SENSOR_THRESHOLD);
}

static float trayCurrentAngle()
{
    return g_servoTray ? g_servoTray->getCurrentAngle() : 0.0f;
}

static float depthForColor(int c)
{
    // ROT (3) and GRÜN (5): use GRÜN depth.  GELB (4) and BLAU (7): use BLAU depth.
    if (c == 7 || c == 4) return D2_DOWN_BLAU;
    return D2_DOWN_GRUEN;
}

// Compute centroid line-follower step using the mount-cross convention:
//   M10 (right) ← getLeftWheelVelocity()
//   M11 (left)  ← getRightWheelVelocity()
static void applyLineFollowSpeed(float scale)
{
    m_cmd_M10 = VEL_SIGN * g_lf->getLeftWheelVelocity()  * scale;
    m_cmd_M11 = VEL_SIGN * g_lf->getRightWheelVelocity() * scale;
    g_M10->setVelocity(m_cmd_M10);
    g_M11->setVelocity(m_cmd_M11);
}

// Drive both motors at a single speed.
static void driveStraight(float spd)
{
    g_M10->setVelocity(VEL_SIGN * spd);
    g_M11->setVelocity(VEL_SIGN * spd);
    m_cmd_M10 = m_cmd_M11 = VEL_SIGN * spd;
}

static void stopMotors()
{
    g_M10->setVelocity(0.0f);
    g_M11->setVelocity(0.0f);
    m_cmd_M10 = m_cmd_M11 = 0.0f;
}

// Check the colour sensor while driving and trigger m_slowing when stable.
// Returns the latest action colour (3/4/5/7) when stable_count is met, else 0.
static void pollColorWhileDriving()
{
    if (m_slowing || m_color_delay_ctr > 0) {
        if (m_color_delay_ctr > 0) m_color_delay_ctr--;
        m_color_stable  = 0;
        m_color_pending = 0;
        return;
    }
    int c = g_cs->getColor();
    if (isActionColor(c) && c == m_color_pending) {
        m_color_stable++;
    } else if (isActionColor(c)) {
        m_color_pending  = c;
        m_color_stable   = 1;
        m_color_fallback = c;
    } else {
        m_color_pending  = 0;
        m_color_stable   = 0;
    }
    if (m_color_stable >= COLOR_STABLE_CNT) {
        m_slowing  = true;
        m_slow_ctr = 0;
    }
}

// 1.0 → SLOW_FACTOR slow-ramp once m_slowing is true.
static float currentSlowRamp()
{
    if (!m_slowing) return 1.0f;
    float t = (m_slow_ctr < SLOWDOWN_LOOPS)
                  ? static_cast<float>(m_slow_ctr) / static_cast<float>(SLOWDOWN_LOOPS)
                  : 1.0f;
    if (m_slow_ctr < SLOWDOWN_LOOPS) m_slow_ctr++;
    return 1.0f - t * (1.0f - SLOW_FACTOR);
}

// Read colour at standstill — confirm m_action_color.
static void pollColorAtStandstill()
{
    int c = g_cs->getColor();
    if (isActionColor(c) && c == m_color_pending) {
        m_color_stable++;
    } else if (isActionColor(c)) {
        m_color_pending = c;
        m_color_stable  = 1;
    }
    if (m_color_stable >= COLOR_STOP_STABLE_CNT && m_action_color == 0) {
        m_action_color = m_color_pending;
    }
}

// Set a new tray target and start the P-controller (mirrors TEST_PARALLAX_360
// MOVING state). Use this everywhere instead of g_servoTray->moveToAngle()
// so serviceTray() knows to call update() instead of stop().
static void trayMoveTo(float deg)
{
    g_servoTray->moveToAngle(deg);
    m_tray_moving = true;
}

// Set tray target_angle + dispatch flags from the (just-confirmed) action colour.
static void dispatchOnColor(int color, bool small_line)
{
    int slot = colorToSlot(color);
    if (slot >= 0) {
        m_target_slot   = slot;
        m_target_angle  = COLOR_ANGLE[slot];
        trayMoveTo(m_target_angle);
    }
    m_needs_drive_past = (color == 3 || color == 4); // ROT or GELB
    (void)small_line;
}

// NeoPixel — quick visual mirror of detected colour (dimmed).
static void updateNeoPixel(int color)
{
    uint8_t r = 0, g = 0, b = 0;
    switch (color) {
        case 3: r = 60;            break;       // ROT
        case 4: r = 60; g = 25;    break;       // GELB
        case 5: g = 60;            break;       // GRÜN
        case 7: b = 60;            break;       // BLAU
        default: break;
    }
    g_np->setRGB(r, g, b);
    g_np->show();
}

// LED blink-code: fast blink when no signal, N short pulses per 2 s otherwise.
static void updateUserLed(DigitalOut& led)
{
    m_led_ctr = (m_led_ctr + 1) % 100;
    if (m_led_color == 0) {
        led = !led;
    } else {
        int n = (m_led_color == 3) ? 1 :
                (m_led_color == 4) ? 2 :
                (m_led_color == 5) ? 3 :
                (m_led_color == 7) ? 4 : 0;
        int pos = m_led_ctr;
        led = (n > 0 && pos < n * 16 && (pos % 16) < 8) ? 1 : 0;
    }
}

// Drive the tray feedback loop — mirrors TEST_PARALLAX_360's warmup/moving pattern:
//   warmup: stop() until feedback valid (no premature movement).
//   moving: update() until isAtTarget() → stop() and wait (no jiggle at rest).
//   stopped: stop() every loop; trayMoveTo() restarts movement.
static void serviceTray()
{
    if (!g_servoTray) return;
    if (!m_sf360_ready) {
        g_servoTray->stop();
        m_sf360_warmup_ctr++;
        if (m_sf360_warmup_ctr >= SF360_WARMUP_LOOPS &&
            g_servoTray->isFeedbackValid()) {
            m_sf360_ready = true;
        }
        return;
    }
    if (!m_tray_moving) {
        g_servoTray->stop();
        return;
    }
    g_servoTray->update();
    if (g_servoTray->isAtTarget()) {
        g_servoTray->stop();
        m_tray_moving = false;
    }
}

// ===========================================================================
// Public API
// ===========================================================================
void roboter_v27_init(int /*loops_per_second*/)
{
    // Motors — pin assignment per Drahtzugliste V10.
    static DCMotor motor_M10(PB_13, PA_6, PC_7, GEAR_RATIO, KN, VOLTAGE_MAX);
    static DCMotor motor_M11(PA_9,  PB_6, PB_7, GEAR_RATIO, KN, VOLTAGE_MAX);
    static DigitalOut enable_motors(PB_15);
    static LineFollower lineFollower(PB_9, PB_8, BAR_DIST, D_WHEEL, B_WHEEL,
                                     motor_M10.getMaxPhysicalVelocity());
    static ColorSensor colorSensor(PB_3, PC_3, PA_4, PB_0, PC_1, PC_0);
    static Servo servoVert(PB_2);    // M21
    static Servo servoHoriz(PC_8);   // M20
    static ServoFeedback360 servoTray(PB_12, PC_2,
                                      SF360_KP, SF360_TOL_DEG,
                                      SF360_MIN_SPEED, SF360_OFFSET);
    static NeoPixel neopixel(PC_5);

    g_M10        = &motor_M10;
    g_M11        = &motor_M11;
    g_en         = &enable_motors;
    g_lf         = &lineFollower;
    g_cs         = &colorSensor;
    g_servoVert  = &servoVert;
    g_servoHoriz = &servoHoriz;
    g_servoTray  = &servoTray;
    g_np         = &neopixel;

    g_np->clear();
    g_cs->setCalibration();
    g_cs->switchLed(ON);
    g_servoVert ->calibratePulseMinMax(0.0200f, 0.1310f);
    g_servoHoriz->calibratePulseMinMax(0.0500f, 0.1050f);

    g_lf->setRotationalVelocityControllerGains(KP, KP_NL);
    g_lf->setMaxWheelVelocity(MAX_SPEED);

    TIM1->BDTR |= TIM_BDTR_MOE;

    *g_en = 0;

    // Reset all runtime state.
    m_state              = STATE_BLIND;
    m_sf360_warmup_ctr   = 0;
    m_sf360_ready        = false;
    for (int i = 0; i < 4; i++) m_slots[i] = false;

    m_straight_ctr       = 0;
    m_follow_ctr         = 0;
    m_brake_ctr          = 0;
    m_pause_ctr          = 0;
    m_backward_ctr       = 0;
    m_guard_ctr          = 0;
    m_approach_ctr       = 0;
    m_accel_ctr          = 0;
    m_crossing_ctr       = 0;
    m_small_crossing_ctr = 0;
    m_drive_past_ctr     = 0;
    m_tray_timeout_ctr   = 0;
    m_tray_moving        = false;
    m_phase_ctr          = 0;
    m_phase_idx          = 0;
    m_crossings_left     = 0;
    m_small_crossings_left = 0;
    m_brake_start_M10 = m_brake_start_M11 = 0.0f;
    m_cmd_M10 = m_cmd_M11 = 0.0f;

    m_current_color   = 0;
    m_color_pending   = 0;
    m_color_stable    = 0;
    m_color_delay_ctr = 0;
    m_slowing         = false;
    m_slow_ctr        = 0;
    m_action_color    = 0;
    m_color_fallback  = 0;
    m_assumption_set   = false;
    m_assumption_color = 0;
    m_is_delivering    = false;
    m_target_angle     = 0.0f;
    m_target_slot      = -1;
    m_needs_drive_past = false;

    m_led_ctr   = 0;
    m_led_color = 0;

    g_M10->setVelocity(0.0f);
    g_M11->setVelocity(0.0f);
}

// ---------------------------------------------------------------------------
// Pickup phase implementation — five sequential phases.
// Drives the tray to target_angle ± offsets, the vertical arm down/up and
// the horizontal arm in 2-D jiggle. Returns true when the entire sequence
// has finished.
// ---------------------------------------------------------------------------
static bool runPickupPhase()
{
    // Phase 0 — extend horizontal arm fully, tray to +50°.
    if (m_phase_idx == 0) {
        if (m_phase_ctr == 0) {
            g_servoHoriz->enable(D1_EXTENDED);
            g_servoVert ->enable(D2_UP);
            trayMoveTo(m_target_angle + PICKUP_OFFSET_DEG);
        }
        m_phase_ctr++;
        if (m_phase_ctr >= PICKUP_PHASE1_LOOPS) {
            m_phase_idx = 1;
            m_phase_ctr = 0;
        }
        return false;
    }

    // Phase 1 — D2 partial down (~10 mm).
    if (m_phase_idx == 1) {
        if (m_phase_ctr == 0) {
            g_servoVert->setPulseWidth(D2_PARTIAL_DOWN);
        }
        m_phase_ctr++;
        if (m_phase_ctr >= PICKUP_PHASE2_LOOPS) {
            m_phase_idx = 2;
            m_phase_ctr = 0;
        }
        return false;
    }

    // Phase 2 — tray back to target_angle.
    if (m_phase_idx == 2) {
        if (m_phase_ctr == 0) {
            trayMoveTo(m_target_angle);
        }
        m_phase_ctr++;
        if (m_phase_ctr >= PICKUP_PHASE3_LOOPS) {
            m_phase_idx = 3;
            m_phase_ctr = 0;
        }
        return false;
    }

    // Phase 3 — D2 full down to pickup depth.
    if (m_phase_idx == 3) {
        if (m_phase_ctr == 0) {
            g_servoVert->setPulseWidth(depthForColor(m_action_color));
        }
        m_phase_ctr++;
        if (m_phase_ctr >= PICKUP_PHASE4_LOOPS) {
            m_phase_idx = 4;
            m_phase_ctr = 0;
        }
        return false;
    }

    // Phase 4 — 2-D jiggle: tray ±5°, horizontal ±5%.
    if (m_phase_idx == 4) {
        const int q = JIGGLE_LOOPS / 4;
        int qi = m_phase_ctr / q;
        if (m_phase_ctr % q == 0) {
            if (qi == 0) {
                trayMoveTo(m_target_angle + JIGGLE_AMPL_DEG);
                g_servoHoriz->setPulseWidth(D1_JIGGLE_OUT);
            } else if (qi == 1) {
                trayMoveTo(m_target_angle - JIGGLE_AMPL_DEG);
                g_servoHoriz->setPulseWidth(D1_JIGGLE_IN);
            } else if (qi == 2) {
                trayMoveTo(m_target_angle + JIGGLE_AMPL_DEG);
                g_servoHoriz->setPulseWidth(D1_JIGGLE_OUT);
            } else {
                trayMoveTo(m_target_angle);
                g_servoHoriz->setPulseWidth(D1_EXTENDED);
            }
        }
        m_phase_ctr++;
        if (m_phase_ctr >= JIGGLE_LOOPS) {
            m_phase_idx = 5;
            m_phase_ctr = 0;
        }
        return false;
    }

    // Phase 5 — raise: D2 up, retract horizontal.
    if (m_phase_idx == 5) {
        if (m_phase_ctr == 0) {
            g_servoVert ->setPulseWidth(D2_UP);
            g_servoHoriz->setPulseWidth(D1_RETRACTED);
        }
        m_phase_ctr++;
        if (m_phase_ctr >= RAISE_LOOPS) {
            g_servoVert ->disable();
            g_servoHoriz->disable();
            return true;
        }
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Deliver phase — three internal phases (no +50° offset; just down + jiggle + up).
// ---------------------------------------------------------------------------
static bool runDeliverPhase()
{
    // Phase 0 — extend horizontal, lower vertical to deliver depth.
    if (m_phase_idx == 0) {
        if (m_phase_ctr == 0) {
            g_servoHoriz->enable(D1_EXTENDED);
            g_servoVert ->enable(depthForColor(m_action_color));
            trayMoveTo(m_target_angle);
        }
        m_phase_ctr++;
        if (m_phase_ctr >= PICKUP_PHASE4_LOOPS) {
            m_phase_idx = 1;
            m_phase_ctr = 0;
        }
        return false;
    }

    // Phase 1 — 2-D jiggle (identical to pickup).
    if (m_phase_idx == 1) {
        const int q = JIGGLE_LOOPS / 4;
        int qi = m_phase_ctr / q;
        if (m_phase_ctr % q == 0) {
            if (qi == 0) {
                trayMoveTo(m_target_angle + JIGGLE_AMPL_DEG);
                g_servoHoriz->setPulseWidth(D1_JIGGLE_OUT);
            } else if (qi == 1) {
                trayMoveTo(m_target_angle - JIGGLE_AMPL_DEG);
                g_servoHoriz->setPulseWidth(D1_JIGGLE_IN);
            } else if (qi == 2) {
                trayMoveTo(m_target_angle + JIGGLE_AMPL_DEG);
                g_servoHoriz->setPulseWidth(D1_JIGGLE_OUT);
            } else {
                trayMoveTo(m_target_angle);
                g_servoHoriz->setPulseWidth(D1_EXTENDED);
            }
        }
        m_phase_ctr++;
        if (m_phase_ctr >= JIGGLE_LOOPS) {
            m_phase_idx = 2;
            m_phase_ctr = 0;
        }
        return false;
    }

    // Phase 2 — raise + retract.
    if (m_phase_idx == 2) {
        if (m_phase_ctr == 0) {
            g_servoVert ->setPulseWidth(D2_UP);
            g_servoHoriz->setPulseWidth(D1_RETRACTED);
        }
        m_phase_ctr++;
        if (m_phase_ctr >= RAISE_LOOPS) {
            g_servoVert ->disable();
            g_servoHoriz->disable();
            return true;
        }
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Main task — 50 Hz tick.
// ---------------------------------------------------------------------------
void roboter_v27_task(DigitalOut& led)
{
    // 1. Sample colour, update NeoPixel + LED bookkeeping.
    m_current_color = g_cs->getColor();
    if (isActionColor(m_current_color)) m_led_color = m_current_color;
    updateNeoPixel(m_current_color);
    updateUserLed(led);

    // 2. Service the tray feedback loop every loop.
    serviceTray();

    *g_en = 1;

    switch (m_state) {

        // ------------------------------------------------------------------
        // Phase 0 — INTRO sequence
        // ------------------------------------------------------------------
        case STATE_BLIND:
            driveStraight(BLIND_SPEED);
            // Wait for bar AND tray feedback (like TEST_PARALLAX_360 explicit warmup).
            // m_sf360_ready becomes true after 25 loops of stop() + isFeedbackValid().
            if (sevenOfEightActive() && m_sf360_ready) {
                g_servoHoriz->enable(D1_RETRACTED);
                g_servoVert ->enable(D2_UP);
                m_straight_ctr = STRAIGHT_LOOPS;
                m_state = STATE_STRAIGHT;
            }
            break;

        case STATE_STRAIGHT:
            driveStraight(STRAIGHT_SPEED);
            m_straight_ctr--;
            if (m_straight_ctr <= 0) {
                stopMotors();
                m_state = STATE_TURN_SPOT;
            }
            break;

        case STATE_TURN_SPOT:
            g_M10->setVelocity(VEL_SIGN * -TURN_SPEED);
            g_M11->setVelocity(VEL_SIGN *  TURN_SPEED);
            if (centerSensorsActive()) {
                g_lf->setRotationalVelocityControllerGains(KP_FOLLOW, KP_NL_FOLLOW);
                m_guard_ctr = START_GUARD;
                m_state = STATE_FOLLOW;
            }
            break;

        case STATE_FOLLOW:
            applyLineFollowSpeed(1.0f);
            if (m_guard_ctr > 0) {
                m_guard_ctr--;
            } else if (sevenOfEightActive()) {
                m_follow_ctr = FOLLOW_1S_LOOPS;
                m_state = STATE_FOLLOW_1S;
            }
            break;

        case STATE_FOLLOW_1S:
            applyLineFollowSpeed(1.0f);
            m_follow_ctr--;
            if (m_follow_ctr <= 0) {
                m_brake_start_M10 = m_cmd_M10;
                m_brake_start_M11 = m_cmd_M11;
                m_brake_ctr = BRAKE_LOOPS;
                m_state = STATE_BRAKE;
            }
            break;

        case STATE_BRAKE: {
            float t = static_cast<float>(m_brake_ctr) / static_cast<float>(BRAKE_LOOPS);
            g_M10->setVelocity(m_brake_start_M10 * t);
            g_M11->setVelocity(m_brake_start_M11 * t);
            m_brake_ctr--;
            if (m_brake_ctr <= 0) {
                stopMotors();
                m_pause_ctr = PAUSE_LOOPS;
                m_state = STATE_PAUSE;
            }
            break;
        }

        case STATE_PAUSE:
            stopMotors();
            m_pause_ctr--;
            if (m_pause_ctr <= 0) {
                m_backward_ctr = BACKWARD_LOOPS;
                m_state = STATE_BACKWARD;
            }
            break;

        case STATE_BACKWARD: {
            int   elapsed   = BACKWARD_LOOPS - m_backward_ctr;
            float ramp_up   = (elapsed < ACCEL_LOOPS)
                                  ? static_cast<float>(elapsed) / static_cast<float>(ACCEL_LOOPS)
                                  : 1.0f;
            float ramp_down = (m_backward_ctr < BRAKE_LOOPS)
                                  ? static_cast<float>(m_backward_ctr) / static_cast<float>(BRAKE_LOOPS)
                                  : 1.0f;
            float ramp = (ramp_up < ramp_down) ? ramp_up : ramp_down;
            float spd  = -VEL_SIGN * BACKWARD_SPEED * ramp;
            g_M10->setVelocity(spd);
            g_M11->setVelocity(spd);
            m_backward_ctr--;
            if (m_backward_ctr <= 0) {
                stopMotors();
                m_pause_ctr = PAUSE_LOOPS;
                m_state = STATE_START_PAUSE;
            }
            break;
        }

        case STATE_START_PAUSE:
            stopMotors();
            m_pause_ctr--;
            if (m_pause_ctr <= 0) {
                m_crossings_left = TOTAL_CROSSINGS;
                m_approach_ctr   = 0;
                m_is_delivering  = false;
                m_state = STATE_APPROACH;
            }
            break;

        // ------------------------------------------------------------------
        // Phase 1 — PICKUP cycle (4 wide bars)
        // ------------------------------------------------------------------
        case STATE_APPROACH: {
            float ramp = (m_approach_ctr < ACCEL_LOOPS)
                              ? static_cast<float>(m_approach_ctr) / static_cast<float>(ACCEL_LOOPS)
                              : 1.0f;
            driveStraight(APPROACH_SPEED * ramp);
            m_approach_ctr++;

            if (m_approach_ctr > ACCEL_LOOPS) {
                int c = g_cs->getColor();
                if (isActionColor(c)) m_color_fallback = c;
            }
            if (m_approach_ctr > ACCEL_LOOPS && wideBarActive()) {
                stopMotors();
                m_crossings_left--;
                m_crossing_ctr   = CROSSING_STOP_LOOPS;
                m_action_color   = 0;
                m_color_pending  = 0;
                m_color_stable   = 0;
                m_assumption_set = false;
                m_state = STATE_CROSSING_STOP;
            }
            break;
        }

        case STATE_LINE_FOLLOW: {
            g_lf->setMaxWheelVelocity(MAX_SPEED);
            g_lf->setRotationalVelocityControllerGains(KP, KP_NL);

            if (m_accel_ctr < RESTART_ACCEL_LOOPS) {
                float ramp = static_cast<float>(m_accel_ctr + 1) /
                             static_cast<float>(RESTART_ACCEL_LOOPS);
                driveStraight(MAX_SPEED * ramp);
                m_accel_ctr++;
            } else {
                pollColorWhileDriving();
                applyLineFollowSpeed(currentSlowRamp());
            }

            if (m_guard_ctr > 0) {
                m_guard_ctr--;
            } else if (m_color_stable >= COLOR_STABLE_CNT && !m_assumption_set) {
                // colour spotted → enter COLOUR_ASSUMPTION
                m_assumption_color = m_color_pending;
                int slot = colorToSlot(m_assumption_color);
                if (slot >= 0) {
                    trayMoveTo(COLOR_ANGLE[slot]);
                }
                m_assumption_set = true;
                m_state = STATE_COLOUR_ASSUMPTION;
            } else if (wideBarActive()) {
                stopMotors();
                m_crossings_left--;
                m_crossing_ctr   = CROSSING_STOP_LOOPS;
                m_action_color   = 0;
                m_color_pending  = 0;
                m_color_stable   = 0;
                m_assumption_set = false;
                m_state = STATE_CROSSING_STOP;
            }
            break;
        }

        case STATE_COLOUR_ASSUMPTION: {
            // Continue line-following while tray pre-rotates.
            applyLineFollowSpeed(currentSlowRamp());

            if (wideBarActive()) {
                stopMotors();
                m_crossings_left--;
                m_crossing_ctr   = CROSSING_STOP_LOOPS;
                m_action_color   = 0;
                m_color_pending  = 0;
                m_color_stable   = 0;
                m_state = STATE_CROSSING_STOP;
            } else if (g_servoTray->isAtTarget()) {
                // tray has reached predicted angle → drop back to LINE_FOLLOW
                m_assumption_set = false;
                m_state = STATE_LINE_FOLLOW;
            }
            break;
        }

        case STATE_CROSSING_STOP:
            stopMotors();

            if (m_crossing_ctr > CROSSING_STOP_LOOPS - COLOR_READ_PHASE) {
                pollColorAtStandstill();
            } else if (m_crossing_ctr == CROSSING_STOP_LOOPS - COLOR_READ_PHASE) {
                if (m_action_color == 0) m_action_color = m_color_fallback;
                dispatchOnColor(m_action_color, false);
                m_tray_timeout_ctr = 0;
            }

            m_crossing_ctr--;

            // Wait for tray to reach target (mirrors TEST_PARALLAX_360 MOVING state).
            // SF360_TIMEOUT_LOOPS (5 s) guards against infinite hang if servo stalls.
            if (m_crossing_ctr <= 0) {
                m_tray_timeout_ctr++;
                bool tray_ok = g_servoTray->isAtTarget() ||
                               m_tray_timeout_ctr >= SF360_TIMEOUT_LOOPS;
                if (tray_ok) {
                    m_slowing         = false;
                    m_slow_ctr        = 0;
                    m_color_delay_ctr = COLOR_READ_DELAY;
                    m_color_pending   = 0;
                    m_color_stable    = 0;
                    m_phase_idx       = 0;
                    m_phase_ctr       = 0;
                    m_tray_timeout_ctr = 0;

                    if (m_needs_drive_past) {
                        m_drive_past_ctr = 0;
                        g_lf->setRotationalVelocityControllerGains(KP, KP_NL);
                        g_lf->setMaxWheelVelocity(MAX_SPEED);
                        m_state = STATE_COLOUR_DRIVE_PAST;
                    } else {
                        m_state = STATE_PICKUP;
                    }
                }
            }
            break;

        case STATE_COLOUR_DRIVE_PAST: {
            int loops_done = m_drive_past_ctr;
            int loops_left = ROT_GELB_DRIVE_LOOPS - loops_done;
            float ramp;
            if (loops_done < ROT_GELB_ACCEL_LOOPS) {
                ramp = static_cast<float>(loops_done + 1) /
                       static_cast<float>(ROT_GELB_ACCEL_LOOPS);
            } else if (loops_left <= ROT_GELB_BRAKE_LOOPS) {
                ramp = static_cast<float>(loops_left) /
                       static_cast<float>(ROT_GELB_BRAKE_LOOPS);
            } else {
                ramp = 1.0f;
            }
            applyLineFollowSpeed(ramp);
            m_drive_past_ctr++;
            if (m_drive_past_ctr >= ROT_GELB_DRIVE_LOOPS) {
                stopMotors();
                m_phase_idx = 0;
                m_phase_ctr = 0;
                m_state = m_is_delivering ? STATE_DELIVER : STATE_PICKUP;
            }
            break;
        }

        case STATE_PICKUP:
            stopMotors();
            if (runPickupPhase()) {
                int slot = m_target_slot;
                if (slot >= 0) m_slots[slot] = true;
                m_color_delay_ctr = COLOR_READ_DELAY;
                m_color_fallback  = 0;
                m_action_color    = 0;
                m_phase_idx = m_phase_ctr = 0;

                if (m_crossings_left > 0) {
                    m_guard_ctr = STOP_GUARD;
                    m_accel_ctr = 0;
                    m_state = STATE_LINE_FOLLOW;
                } else {
                    m_small_crossings_left = TOTAL_SMALL_CROSSINGS;
                    m_guard_ctr            = SMALL_FOLLOW_START_GUARD;
                    m_accel_ctr            = 0;
                    m_is_delivering        = true;
                    m_state                = STATE_SMALL_FOLLOW;
                }
            }
            break;

        // ------------------------------------------------------------------
        // Phase 2 — DELIVER cycle (4 narrow lines)
        // ------------------------------------------------------------------
        case STATE_SMALL_FOLLOW: {
            g_lf->setMaxWheelVelocity(MAX_SPEED);
            g_lf->setRotationalVelocityControllerGains(KP, KP_NL);

            if (m_accel_ctr < RESTART_ACCEL_LOOPS) {
                float ramp = static_cast<float>(m_accel_ctr + 1) /
                             static_cast<float>(RESTART_ACCEL_LOOPS);
                driveStraight(MAX_SPEED * ramp);
                m_accel_ctr++;
            } else {
                pollColorWhileDriving();
                applyLineFollowSpeed(currentSlowRamp());
            }

            if (m_guard_ctr > 0) {
                m_guard_ctr--;
            } else if (m_color_stable >= COLOR_STABLE_CNT && !m_assumption_set) {
                m_assumption_color = m_color_pending;
                int slot = colorToSlot(m_assumption_color);
                if (slot >= 0) {
                    trayMoveTo(COLOR_ANGLE[slot]);
                }
                m_assumption_set = true;
                m_state = STATE_SMALL_COLOUR_ASSUMPTION;
            } else if (smallLineActive()) {
                stopMotors();
                m_small_crossings_left--;
                m_small_crossing_ctr = CROSSING_STOP_LOOPS;
                m_action_color   = 0;
                m_color_pending  = 0;
                m_color_stable   = 0;
                m_assumption_set = false;
                m_state = STATE_SMALL_CROSSING_STOP;
            }
            break;
        }

        case STATE_SMALL_COLOUR_ASSUMPTION: {
            applyLineFollowSpeed(currentSlowRamp());

            if (smallLineActive()) {
                stopMotors();
                m_small_crossings_left--;
                m_small_crossing_ctr = CROSSING_STOP_LOOPS;
                m_action_color   = 0;
                m_color_pending  = 0;
                m_color_stable   = 0;
                m_state = STATE_SMALL_CROSSING_STOP;
            } else if (g_servoTray->isAtTarget()) {
                m_assumption_set = false;
                m_state = STATE_SMALL_FOLLOW;
            }
            break;
        }

        case STATE_SMALL_CROSSING_STOP:
            stopMotors();

            if (m_small_crossing_ctr > CROSSING_STOP_LOOPS - COLOR_READ_PHASE) {
                pollColorAtStandstill();
            } else if (m_small_crossing_ctr == CROSSING_STOP_LOOPS - COLOR_READ_PHASE) {
                if (m_action_color == 0) m_action_color = m_color_fallback;
                dispatchOnColor(m_action_color, true);
            }

            m_small_crossing_ctr--;
            if (m_small_crossing_ctr <= 0 && g_servoTray->isAtTarget()) {
                m_slowing       = false;
                m_slow_ctr      = 0;
                m_color_delay_ctr = COLOR_READ_DELAY;
                m_color_pending = 0;
                m_color_stable  = 0;
                m_phase_idx     = 0;
                m_phase_ctr     = 0;

                if (m_needs_drive_past) {
                    m_drive_past_ctr = 0;
                    g_lf->setRotationalVelocityControllerGains(KP, KP_NL);
                    g_lf->setMaxWheelVelocity(MAX_SPEED);
                    m_state = STATE_COLOUR_DRIVE_PAST;
                } else {
                    m_state = STATE_DELIVER;
                }
            }
            break;

        case STATE_DELIVER:
            stopMotors();
            if (runDeliverPhase()) {
                int slot = m_target_slot;
                if (slot >= 0) m_slots[slot] = false;
                m_color_delay_ctr = COLOR_READ_DELAY;
                m_color_fallback  = 0;
                m_action_color    = 0;
                m_phase_idx = m_phase_ctr = 0;

                if (m_small_crossings_left > 0) {
                    m_guard_ctr = SMALL_REENTRY_GUARD;
                    m_accel_ctr = 0;
                    m_state = STATE_SMALL_FOLLOW;
                } else {
                    m_state = STATE_FINAL_HALT;
                }
            }
            break;

        case STATE_FINAL_HALT:
            stopMotors();
            break;
    }
}

void roboter_v27_reset(DigitalOut& led)
{
    if (g_en) *g_en = 0;
    if (g_M10) g_M10->setVelocity(0.0f);
    if (g_M11) g_M11->setVelocity(0.0f);
    if (g_servoVert)  g_servoVert ->disable();
    if (g_servoHoriz) g_servoHoriz->disable();
    if (g_servoTray)  g_servoTray ->stop();
    if (g_np)         g_np->clear();

    m_state              = STATE_BLIND;
    m_sf360_warmup_ctr   = 0;
    m_sf360_ready        = false;
    for (int i = 0; i < 4; i++) m_slots[i] = false;

    m_straight_ctr = m_follow_ctr = m_brake_ctr = m_pause_ctr = m_backward_ctr = 0;
    m_guard_ctr = m_approach_ctr = m_accel_ctr = 0;
    m_crossing_ctr = m_small_crossing_ctr = m_drive_past_ctr = 0;
    m_phase_ctr = m_phase_idx = 0;
    m_crossings_left = m_small_crossings_left = 0;
    m_brake_start_M10 = m_brake_start_M11 = 0.0f;
    m_cmd_M10 = m_cmd_M11 = 0.0f;

    m_current_color = m_color_pending = m_color_stable = m_color_delay_ctr = 0;
    m_slowing = false;
    m_slow_ctr = 0;
    m_action_color = 0;
    m_color_fallback = 0;
    m_assumption_set = false;
    m_assumption_color = 0;
    m_is_delivering = false;
    m_target_angle = 0.0f;
    m_target_slot = -1;
    m_needs_drive_past = false;

    m_led_ctr = 0;
    m_led_color = 0;
    led = 0;
}

void roboter_v27_print()
{
    static int print_ctr = 0;
    if (++print_ctr < 10) return;   // ~5 Hz
    print_ctr = 0;

    const char* state_name = "?";
    switch (m_state) {
        case STATE_BLIND:                    state_name = "BLIND";              break;
        case STATE_STRAIGHT:                 state_name = "STRAIGHT";           break;
        case STATE_TURN_SPOT:                state_name = "TURN_SPOT";          break;
        case STATE_FOLLOW:                   state_name = "FOLLOW";             break;
        case STATE_FOLLOW_1S:                state_name = "FOLLOW_1S";          break;
        case STATE_BRAKE:                    state_name = "BRAKE";              break;
        case STATE_PAUSE:                    state_name = "PAUSE";              break;
        case STATE_BACKWARD:                 state_name = "BACKWARD";           break;
        case STATE_START_PAUSE:              state_name = "START_PAUSE";        break;
        case STATE_APPROACH:                 state_name = "APPROACH";           break;
        case STATE_LINE_FOLLOW:              state_name = "LINE_FOLLOW";        break;
        case STATE_COLOUR_ASSUMPTION:        state_name = "COL_ASSUME";         break;
        case STATE_CROSSING_STOP:            state_name = "X_STOP";             break;
        case STATE_COLOUR_DRIVE_PAST:        state_name = "DRIVE_PAST";         break;
        case STATE_PICKUP:                   state_name = "PICKUP";             break;
        case STATE_SMALL_FOLLOW:             state_name = "SMALL_FOLLOW";       break;
        case STATE_SMALL_COLOUR_ASSUMPTION:  state_name = "SMALL_COL_ASSUME";   break;
        case STATE_SMALL_CROSSING_STOP:      state_name = "SMALL_X_STOP";       break;
        case STATE_DELIVER:                  state_name = "DELIVER";            break;
        case STATE_FINAL_HALT:               state_name = "HALT";               break;
    }

    printf("%-16s | col=%d | tray=%5.1f tgt=%5.1f | slots=[%c%c%c%c] | wL=%d nL=%d\n",
           state_name,
           m_current_color,
           trayCurrentAngle(),
           m_target_angle,
           m_slots[0] ? 'R' : '-',
           m_slots[1] ? 'G' : '-',
           m_slots[2] ? 'B' : '-',
           m_slots[3] ? 'Y' : '-',
           m_crossings_left,
           m_small_crossings_left);
}

#endif // PROTOTYPE_03_V27
