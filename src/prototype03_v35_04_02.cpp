// CargoSweep — PROTOTYPE_03_V35_04_02
// Per-Farbe-States: jede Farbe hat eigenen State mit hard-coded Tray-Winkel.
// CROSSING_STOP und SMALL_CROSSING_STOP sind reine Dispatch-States.
// STATE_ROT_GELB_DRIVE und STATE_ROT_GELB_PAUSE entfallen — Logik in
// STATE_ARM_ROT / STATE_ARM_GELB integriert (Vorwärtsfahrt + Arm).
#include "test_config.h"

#ifdef PROTOTYPE_03_V35_04_02

#include "prototype03_v35_04_02.h"
#include <cmath>
#include "ColorSensor.h"
#include "DCMotor.h"
#include "LineFollower.h"
#include "NeoPixel.h"
#include "Servo.h"
#include "ServoFeedback360.h"
#include "PESBoardPinMap.h"

// ---------------------------------------------------------------------------
// Motor / Geometrie
// ---------------------------------------------------------------------------
static const float GEAR_RATIO  = 100.0f;
static const float KN          = 140.0f / 12.0f;
static const float VOLTAGE_MAX = 12.0f;
static const float VEL_SIGN    = 1.0f;

static const float D_WHEEL  = 0.0291f;
static const float B_WHEEL  = 0.1493f;
static const float BAR_DIST = 0.182f;

// ---------------------------------------------------------------------------
// Linienfolger PID
// ---------------------------------------------------------------------------
static const float KP           = 2.8f;
static const float KP_NL        = 4.55f;
static const float KP_FOLLOW    = 1.1f;
static const float KP_NL_FOLLOW = 0.7f;
static const float MAX_SPEED    = 1.6f;
static const float SPEED_SCALE  = 1.0f / MAX_SPEED;

static const float BLIND_SPEED    = 1.0f;
static const float STRAIGHT_SPEED = 1.0f;
static const float TURN_SPEED     = 0.5f;
static const float BACKWARD_SPEED = 1.0f;
static const float APPROACH_SPEED = 0.5f;

static const int STRAIGHT_LOOPS       = 70;
static const int FOLLOW_1S_LOOPS      = static_cast<int>(50 * SPEED_SCALE);
static const int BRAKE_LOOPS          = 12;
static const int PAUSE_LOOPS          = 20;
static const int BACKWARD_LOOPS       = 222;
static const int ACCEL_LOOPS          = 12;
static const int RESTART_ACCEL_LOOPS  = 50;
static const int REAL_APPROACH_GUARD  = 20;  // verhindert Startbalken-Trigger
static const int STOP_GUARD           = static_cast<int>(75  * SPEED_SCALE);
static const int START_GUARD          = static_cast<int>(300 * SPEED_SCALE);
static const int SLOWDOWN_LOOPS       = 13;
static const float SLOW_FACTOR        = 0.2f;
static const int COLOR_READ_DELAY     = 50;
static const int COLOR_STABLE_CNT     = 3;
static const int COLOR_READ_PHASE     = 25;
static const int COLOR_STOP_STABLE_CNT = 1;

// ---------------------------------------------------------------------------
// Programm-Konstanten
// ---------------------------------------------------------------------------
static const int   TOTAL_CROSSINGS       = 4;
static const int   CROSSING_STOP_LOOPS   = 250;  // war 150 — mehr Zeit fÃ¼r color reading bei wide bars
static const int   SMALL_FOLLOW_START_GUARD   = static_cast<int>(781 * SPEED_SCALE);
static const int   SMALL_REENTRY_GUARD        = static_cast<int>(100 * SPEED_SCALE);
static const int   TOTAL_SMALL_CROSSINGS      = 4;
static const int   SMALL_CROSSING_STOP_LOOPS  = 100;

// 180° Servo-Positionen
static const float D1_RETRACTED       = 0.1f;
static const float D1_EXTENDED        = 0.80f;
static const float D2_UP              = 1.0f;
static const float D2_PARTIAL_DOWN    = 0.50f;
static const float D2_DOWN_BLAU       = 0.24f;
static const float D2_DOWN_GRUEN      = 0.07f;

// PICKUP / DELIVER Timing
static const int   PICKUP_PAUSE_LOOPS  = 50;
static const int   JIGGLE_LOOPS        = 50;
static const int   RAISE_LOOPS         = 50;
static const int   RETRACT_LOOPS       = 120;
static const float JIGGLE_AMPL_DEG    = 6.0f;
static const float D1_JIGGLE_OFFSET   = 0.05f;
static const int   SF360_TIMEOUT_LOOPS = 150;  // war 60 — 3s fÃ¼r 270Â° GELB tray drehung

static const int   ROT_GELB_DRIVE_LOOPS  = 51;
static const int   ROT_GELB_ACCEL_LOOPS  = 10;
static const int   ROT_GELB_BRAKE_LOOPS  = 12;

static const float SENSOR_THRESHOLD = 0.40f;

// ---------------------------------------------------------------------------
// ServoFeedback360 Parameter
// ---------------------------------------------------------------------------
static const float SF360_KP          = 0.005f;
static const float SF360_TOL_DEG     = 2.1f;
static const float SF360_MIN_SPEED   = 0.01f;
static const float SF360_OFFSET      = 110.0f;
static const int   SF360_WARMUP_LOOPS = 25;
static const float JIGGLE_DEG        = 5.0f;

// Hard-coded Tray-Winkel pro Farbe
static const float ANGLE_ROT   = 0.0f;
static const float ANGLE_GRUEN = 90.0f;
static const float ANGLE_BLAU  = 180.0f;
static const float ANGLE_GELB  = 270.0f;

// ---------------------------------------------------------------------------
// States
// ---------------------------------------------------------------------------
enum State {
    STATE_BLIND               = 1,
    STATE_STRAIGHT            = 2,
    STATE_TURN_SPOT           = 3,
    STATE_FOLLOW              = 4,
    STATE_FOLLOW_1S           = 5,
    STATE_BRAKE               = 6,
    STATE_PAUSE               = 7,
    STATE_BACKWARD            = 8,
    STATE_REAL_START_PAUSE    = 14,
    STATE_REAL_APPROACH       = 15,
    STATE_REAL_FOLLOW         = 9,
    STATE_CROSSING_STOP       = 10,
    STATE_SMALL_FOLLOW        = 12,
    STATE_SMALL_CROSSING_STOP = 13,
    STATE_FINAL_HALT          = 11,
    STATE_RED                 = 20,
    STATE_GREEN               = 21,
    STATE_BLUE                = 22,
    STATE_YELLOW              = 23
};

// ---------------------------------------------------------------------------
// Hardware-Pointer
// ---------------------------------------------------------------------------
static DCMotor*          g_M1        = nullptr;
static DCMotor*          g_M2        = nullptr;
static DigitalOut*       g_en        = nullptr;
static LineFollower*     g_lf        = nullptr;
static ColorSensor*      g_cs        = nullptr;
static Servo*            g_servo_D1  = nullptr;
static Servo*            g_servo_D2  = nullptr;
static ServoFeedback360* g_servoTray = nullptr;
static NeoPixel*         g_np        = nullptr;

// ---------------------------------------------------------------------------
// Laufzeit-Zustand
// ---------------------------------------------------------------------------
static State m_state = STATE_BLIND;

static int   m_sf360_warmup_ctr = 0;
static bool  m_sf360_ready      = false;
static float m_target_angle     = 0.0f;

static int   m_straight_ctr        = 0;
static int   m_follow_ctr          = 0;
static int   m_brake_ctr           = 0;
static int   m_pause_ctr           = 0;
static int   m_backward_ctr        = 0;
static int   m_guard_ctr           = 0;
static int   m_crossing_ctr        = 0;
static int   m_crossings_left      = 0;
static int   m_tray_timeout_ctr    = 0;
static int   m_small_crossing_ctr  = 0;
static int   m_small_crossings_left = 0;
static int   m_approach_ctr        = 0;
static int   m_small_accel_ctr     = 0;
static int   m_real_accel_ctr      = 0;
static int   m_arm_retract_ctr     = 0;
static int   m_phase_idx           = 0;
static int   m_phase_ctr           = 0;
static int   m_drive_ctr           = 0;  // ROT/GELB Vorwärtsfahrt-Zähler
static float m_brake_start_M1      = 0.0f;
static float m_brake_start_M2      = 0.0f;
static float g_cmd_M1              = 0.0f;
static float g_cmd_M2              = 0.0f;

static int   m_current_color       = 0;
static int   m_prev_color          = 0;
static int   m_led_color           = 0;
static int   m_led_ctr             = 0;
static int   m_color_log[8]        = {};
static int   m_color_log_ctr       = 0;
static int   m_action_color        = 0;
static bool  m_slowing             = false;
static int   m_slow_ctr            = 0;
static int   m_color_delay_ctr     = 0;
static int   m_color_stable_ctr    = 0;
static int   m_color_pending       = 0;
static int   m_color_fallback      = 0;
static int   m_approach_fallback   = 0;

static bool  m_tray_moving         = false;
static bool  m_small_line_mode     = false;
static int   m_line_count          = 0;   // counts all 8 lines (4 wide + 4 small)
static bool  m_counting            = false; // starts after alignment
static int   m_home_timer          = 0;

// ---------------------------------------------------------------------------
// Hilfsfunktionen
// ---------------------------------------------------------------------------
static bool all_sensors_active()
{
    int active = 0;
    for (int i = 0; i < 8; i++) {
        if (g_lf->getAvgBit(i) >= SENSOR_THRESHOLD) active++;
    }
    return active >= 7;
}

static bool center_sensors_active()
{
    return g_lf->getAvgBit(3) >= SENSOR_THRESHOLD ||
           g_lf->getAvgBit(4) >= SENSOR_THRESHOLD;
}

static bool wide_bar_active()
{
    int count = 0;
    for (int i = 0; i < 8; i++) {
        if (g_lf->getAvgBit(i) >= SENSOR_THRESHOLD) count++;
    }
    return count >= 4;
}

static bool small_line_active()
{
    return (g_lf->getAvgBit(3) >= SENSOR_THRESHOLD &&
            g_lf->getAvgBit(4) >= SENSOR_THRESHOLD &&
            g_lf->getAvgBit(5) >= SENSOR_THRESHOLD) ||
           (g_lf->getAvgBit(2) >= SENSOR_THRESHOLD &&
            g_lf->getAvgBit(3) >= SENSOR_THRESHOLD &&
            g_lf->getAvgBit(4) >= SENSOR_THRESHOLD);
}

static float depthForColor(int c)
{
    if (c == 7 || c == 4) return D2_DOWN_BLAU;
    return D2_DOWN_GRUEN;
}

static bool trayNearTarget(float tol = 20.0f)
{
    float err = g_servoTray->getCurrentAngle() - m_target_angle;
    if (err >  180.0f) err -= 360.0f;
    if (err < -180.0f) err += 360.0f;
    return fabsf(err) < tol;
}

static float angleForColor(int c)
{
    switch (c) {
        case 3: return ANGLE_ROT;
        case 4: return ANGLE_GELB;
        case 5: return ANGLE_GRUEN;
        case 7: return ANGLE_BLAU;
        default: return m_target_angle;
    }
}

static float extensionForColor(int /*c*/)
{
    return D1_EXTENDED;
}

static void setNeoColor(int color)
{
    if (!g_np) return;
    switch (color) {
        case 3: g_np->setRGB(255,   0,   0); break;
        case 4: g_np->setRGB(255, 200,   0); break;
        case 5: g_np->setRGB(  0, 255,   0); break;
        case 7: g_np->setRGB(  0,   0, 255); break;
        default: g_np->setRGB(5, 5, 5);      break;
    }
    g_np->show();
}

static void trayMoveTo(float deg)
{
    m_target_angle = deg;  // serviceTray() jiggle picks this up every loop
}

static void trayStop()
{
    // No-op: servo stays enabled, P-controller continues via serviceTray().
    // FINAL_HALT stops/disables directly — all other callers just leave servo running.
}

static void applyJiggleTick(int tick)
{
    const int ph1 = JIGGLE_LOOPS / 3;
    const int ph2 = 2 * JIGGLE_LOOPS / 3;
    int jp = tick % JIGGLE_LOOPS;
    if      (jp == 0)    g_servoTray->moveToAngle(m_target_angle - JIGGLE_AMPL_DEG);
    else if (jp == ph1)  g_servoTray->moveToAngle(m_target_angle + JIGGLE_AMPL_DEG);
    else if (jp == ph2)  g_servoTray->moveToAngle(m_target_angle);
}

// ---------------------------------------------------------------------------
// Ablege-Sequenz (runDeliverPhase)
// Phase 0: Tray → target (warten)
// Phase 1: D1+D2 runter
// Phase 2: Jiggle
// Phase 3: D2+D1 hoch → return true
// ---------------------------------------------------------------------------
static bool runDeliverPhase()
{
    // Phase 0: lower arm
    if (m_phase_idx == 0) {
        if (m_phase_ctr == 0) {
            g_servo_D1->enable(extensionForColor(m_action_color));
            g_servo_D2->enable(depthForColor(m_action_color));
        }
        m_phase_ctr++;
        if (m_phase_ctr >= PICKUP_PAUSE_LOOPS) { m_phase_idx = 1; m_phase_ctr = 0; }
        return false;
    }
    // Phase 1: arm down + D1 jiggle, tray jiggle runs in serviceTray()
    if (m_phase_idx == 1) {
        const float base = extensionForColor(m_action_color);
        const int   ph1  = JIGGLE_LOOPS / 3;
        const int   ph2  = 2 * JIGGLE_LOOPS / 3;
        if      (m_phase_ctr == 0)   g_servo_D1->setPulseWidth(base + D1_JIGGLE_OFFSET);
        else if (m_phase_ctr == ph1) g_servo_D1->setPulseWidth(base - D1_JIGGLE_OFFSET);
        else if (m_phase_ctr == ph2) g_servo_D1->setPulseWidth(base);
        m_phase_ctr++;
        if (m_phase_ctr >= JIGGLE_LOOPS) { m_phase_idx = 2; m_phase_ctr = 0; }
        return false;
    }
    // Phase 2: raise arm
    if (m_phase_idx == 2) {
        if (m_phase_ctr == 0) {
            g_servo_D2->setPulseWidth(D2_UP);
            g_servo_D1->setMaxAcceleration(1.0e6f);
            g_servo_D1->setPulseWidth(D1_RETRACTED);
        }
        m_phase_ctr++;
        if (m_phase_ctr >= RAISE_LOOPS) {
            g_servo_D1->disable();
            g_servo_D2->disable();
            return true;
        }
        return false;
    }
    return true;
}

static void serviceTray()
{
    if (!g_servoTray) return;
    g_servoTray->moveToAngle(m_target_angle);  // keep P-controller on current target
    g_servoTray->update();
    // Jiggle overlaid AFTER P-controller — square wave, never zero, can't be stopped
    if (m_counting && m_state != STATE_FINAL_HALT) {
        static int s_jiggle_ctr = 0;
        float jiggle = ((s_jiggle_ctr % JIGGLE_LOOPS) < JIGGLE_LOOPS / 2)
                           ? 0.12f : -0.12f;
        g_servoTray->addSpeed(jiggle);
        s_jiggle_ctr++;
    }
}

static void advanceTray()
{
    m_target_angle = fmodf(m_target_angle + 90.0f, 360.0f);
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
void roboter_v35_04_02_init(int /*loops_per_second*/)
{
    static DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1,
                             GEAR_RATIO, KN, VOLTAGE_MAX);
    static DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2,
                             GEAR_RATIO, KN, VOLTAGE_MAX);
    static DigitalOut enable_motors(PB_ENABLE_DCMOTORS);
    static LineFollower lineFollower(PB_9, PB_8, BAR_DIST, D_WHEEL, B_WHEEL,
                                     motor_M1.getMaxPhysicalVelocity());
    static ColorSensor colorSensor(PB_3, PC_3, PA_4, PB_0, PC_1, PC_0);
    static Servo servo_D1(PC_8);
    static Servo servo_D2(PB_2);
    static ServoFeedback360 servoTray(PB_12, PC_2,
                                      SF360_KP, SF360_TOL_DEG,
                                      SF360_MIN_SPEED, SF360_OFFSET);
    static NeoPixel neopixel(PC_5);

    g_M1        = &motor_M1;
    g_M2        = &motor_M2;
    g_en        = &enable_motors;
    g_lf        = &lineFollower;
    g_cs        = &colorSensor;
    g_servo_D1  = &servo_D1;
    g_servo_D2  = &servo_D2;
    g_servoTray = &servoTray;
    g_np        = &neopixel;

    g_np->clear();
    g_cs->setCalibration();
    g_cs->switchLed(ON);
    g_servo_D1->calibratePulseMinMax(0.0500f, 0.1050f);
    g_servo_D1->setMaxAcceleration(0.3f);
    g_servo_D2->calibratePulseMinMax(0.0200f, 0.1310f);
    g_servo_D1->enable(D1_RETRACTED);
    g_servo_D2->enable(D2_UP);

    g_lf->setRotationalVelocityControllerGains(KP, KP_NL);
    g_lf->setMaxWheelVelocity(MAX_SPEED);

    TIM1->BDTR |= TIM_BDTR_MOE;

    *g_en = 0;

    m_state               = STATE_BLIND;
    m_sf360_warmup_ctr    = 0;
    m_sf360_ready         = false;
    m_target_angle        = 0.0f;
    m_straight_ctr        = 0;
    m_follow_ctr          = 0;
    m_brake_ctr           = 0;
    m_pause_ctr           = 0;
    m_backward_ctr        = 0;
    m_guard_ctr           = 0;
    m_crossing_ctr        = 0;
    m_crossings_left      = 0;
    m_small_crossing_ctr  = 0;
    m_small_crossings_left = 0;
    m_approach_ctr        = 0;
    m_small_accel_ctr     = 0;
    m_real_accel_ctr      = 0;
    m_arm_retract_ctr     = 0;
    m_phase_idx           = 0;
    m_phase_ctr           = 0;
    m_drive_ctr           = 0;
    m_tray_timeout_ctr    = 0;
    m_brake_start_M1      = 0.0f;
    m_brake_start_M2      = 0.0f;
    m_current_color       = 0;
    m_prev_color          = 0;
    m_led_color           = 0;
    m_led_ctr             = 0;
    m_color_log_ctr       = 0;
    m_action_color        = 0;
    m_slowing             = false;
    m_slow_ctr            = 0;
    m_color_delay_ctr     = 0;
    m_color_stable_ctr    = 0;
    m_color_pending       = 0;
    m_color_fallback      = 0;
    m_approach_fallback   = 0;
    for (int i = 0; i < 8; i++) m_color_log[i] = 0;
    g_M1->setVelocity(0.0f);
    g_M2->setVelocity(0.0f);
}

// ---------------------------------------------------------------------------
// Hilfsfunktionen für State-Übergänge
// ---------------------------------------------------------------------------

// Reset gemeinsamer Variablen am Ende einer Arm-Sequenz
static void resetArmExitVars()
{
    m_slowing          = false;
    m_slow_ctr         = 0;
    m_action_color     = 0;
    m_color_delay_ctr  = COLOR_READ_DELAY;
    m_color_stable_ctr = 0;
    m_color_pending    = 0;
    m_color_fallback   = 0;
    m_approach_fallback = 0;
    m_arm_retract_ctr  = 0;
    m_phase_idx        = 0;
    m_phase_ctr        = 0;
    m_drive_ctr        = 0;
    m_tray_timeout_ctr = 0;
}

// Exit aus breiten Balken — zurück zu REAL_FOLLOW oder Übergang zu SMALL_FOLLOW
static void exitWideBar()
{
    if (m_line_count >= 4) {
        m_guard_ctr       = SMALL_FOLLOW_START_GUARD;
        m_small_accel_ctr = 0;
        m_state           = STATE_SMALL_FOLLOW;
    } else {
        m_guard_ctr      = STOP_GUARD;
        m_real_accel_ctr = 0;
        m_state          = STATE_REAL_FOLLOW;
    }
}

static void exitSmallLine()
{
    if (m_line_count >= 8) {
        m_state = STATE_FINAL_HALT;
    } else {
        m_guard_ctr       = SMALL_REENTRY_GUARD;
        m_small_accel_ctr = 0;
        m_state           = STATE_SMALL_FOLLOW;
    }
}

// ---------------------------------------------------------------------------
// Task (50 Hz)
// ---------------------------------------------------------------------------
void roboter_v35_04_02_task(DigitalOut& led)
{
    // Enable 360° servo immediately on button press
    if (!m_sf360_ready && g_servoTray) {
        g_servoTray->enable(0.5f);
        m_sf360_ready = true;
    }

    m_current_color = g_cs->getColor();

    // Independent pre-adjustment: update angle while navigating (not during arm sequence)
    // Pre-adjust only when actually approaching bars — not during backward or other nav
    bool can_preadjust = (m_state == STATE_REAL_APPROACH ||
                          m_state == STATE_REAL_FOLLOW   ||
                          m_state == STATE_SMALL_FOLLOW);
    if (m_sf360_ready && can_preadjust) {
        int col = m_current_color;
        if (col == 3 || col == 4 || col == 5 || col == 7) {
            m_target_angle   = angleForColor(col);
            m_color_fallback = col;
        }
    }

    {
        uint8_t pr = 0, pg = 0, pb = 0;
        switch (m_current_color) {
            case 3: pr = 60;            break;
            case 4: pr = 60; pg = 25;   break;
            case 5: pg = 60;            break;
            case 7: pb = 60;            break;
            default: break;
        }
        g_np->setRGB(pr, pg, pb);
        g_np->show();
    }

    bool is_sig    = (m_current_color == 3 || m_current_color == 4 ||
                      m_current_color == 5 || m_current_color == 7);
    bool was_neutral = (m_prev_color == 0 || m_prev_color == 1 || m_prev_color == 2);
    if (is_sig && was_neutral) {
        if (m_color_log_ctr < 8) m_color_log[m_color_log_ctr++] = m_current_color;
        m_led_color = m_current_color;
    }
    m_prev_color = m_current_color;

    m_led_ctr = (m_led_ctr + 1) % 100;
    if (m_led_color == 0) {
        led = !led;
    } else {
        int n  = (m_led_color == 3) ? 1 :
                 (m_led_color == 4) ? 2 :
                 (m_led_color == 5) ? 3 :
                 (m_led_color == 7) ? 4 : 0;
        int pos = m_led_ctr;
        led     = (n > 0 && pos < n * 16 && (pos % 16) < 8) ? 1 : 0;
    }

    *g_en = 1;

    serviceTray();

    if (m_home_timer > 0) {
        if (--m_home_timer == 0) trayStop();
    }

    switch (m_state) {

        case STATE_BLIND:
            g_M1->setVelocity(VEL_SIGN * BLIND_SPEED);
            g_M2->setVelocity(VEL_SIGN * BLIND_SPEED);
            if (all_sensors_active() && m_sf360_ready) {
                trayMoveTo(0.0f);
                m_home_timer   = 150;
                m_straight_ctr = STRAIGHT_LOOPS;
                m_counting     = true;   // start counting lines after alignment
                m_line_count   = 0;
                m_state        = STATE_STRAIGHT;
            }
            break;

        case STATE_STRAIGHT:
            g_M1->setVelocity(VEL_SIGN * STRAIGHT_SPEED);
            g_M2->setVelocity(VEL_SIGN * STRAIGHT_SPEED);
            m_straight_ctr--;
            if (m_straight_ctr <= 0) {
                g_M1->setVelocity(0.0f);
                g_M2->setVelocity(0.0f);
                m_state = STATE_TURN_SPOT;
            }
            break;

        case STATE_TURN_SPOT:
            g_M1->setVelocity(VEL_SIGN * -TURN_SPEED);
            g_M2->setVelocity(VEL_SIGN *  TURN_SPEED);
            if (center_sensors_active()) {
                g_lf->setRotationalVelocityControllerGains(KP_FOLLOW, KP_NL_FOLLOW);
                m_guard_ctr = START_GUARD;
                m_state     = STATE_FOLLOW;
            }
            break;

        case STATE_FOLLOW:
            g_cmd_M1 = VEL_SIGN * g_lf->getLeftWheelVelocity();
            g_cmd_M2 = VEL_SIGN * g_lf->getRightWheelVelocity();
            g_M1->setVelocity(g_cmd_M1);
            g_M2->setVelocity(g_cmd_M2);
            if (m_guard_ctr > 0) {
                m_guard_ctr--;
            } else if (all_sensors_active()) {
                m_follow_ctr = FOLLOW_1S_LOOPS;
                m_state      = STATE_FOLLOW_1S;
            }
            break;

        case STATE_FOLLOW_1S:
            g_cmd_M1 = VEL_SIGN * g_lf->getLeftWheelVelocity();
            g_cmd_M2 = VEL_SIGN * g_lf->getRightWheelVelocity();
            g_M1->setVelocity(g_cmd_M1);
            g_M2->setVelocity(g_cmd_M2);
            m_follow_ctr--;
            if (m_follow_ctr <= 0) {
                m_brake_start_M1 = g_cmd_M1;
                m_brake_start_M2 = g_cmd_M2;
                m_brake_ctr      = BRAKE_LOOPS;
                m_state          = STATE_BRAKE;
            }
            break;

        case STATE_BRAKE: {
            float t = static_cast<float>(m_brake_ctr) / static_cast<float>(BRAKE_LOOPS);
            g_M1->setVelocity(m_brake_start_M1 * t);
            g_M2->setVelocity(m_brake_start_M2 * t);
            m_brake_ctr--;
            if (m_brake_ctr <= 0) {
                g_M1->setVelocity(0.0f);
                g_M2->setVelocity(0.0f);
                m_pause_ctr = PAUSE_LOOPS;
                m_state     = STATE_PAUSE;
            }
            break;
        }
          
        case STATE_PAUSE:
            g_M1->setVelocity(0.0f);
            g_M2->setVelocity(0.0f);
            m_pause_ctr--;
            if (m_pause_ctr <= 0) {
                m_backward_ctr = BACKWARD_LOOPS;
                m_state        = STATE_BACKWARD;
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
            g_M1->setVelocity(spd);
            g_M2->setVelocity(spd);
            m_backward_ctr--;
            if (m_backward_ctr <= 0) {
                g_M1->setVelocity(0.0f);
                g_M2->setVelocity(0.0f);
                m_color_fallback    = 0;  // discard any colours read during backward
                m_approach_fallback = 0;
                m_pause_ctr = PAUSE_LOOPS;
                m_state     = STATE_REAL_START_PAUSE;
            }
            break;
        }

        // ================================================================
        // REAL PROGRAM
        // ================================================================

        case STATE_REAL_START_PAUSE:
            g_M1->setVelocity(0.0f);
            g_M2->setVelocity(0.0f);
            {   // Farbsensor könnte während Pause über Balken 1 stehen → vormerken
                int col_p = m_current_color;
                if (col_p == 3 || col_p == 4 || col_p == 5 || col_p == 7)
                    m_approach_fallback = col_p;
            }
            m_pause_ctr--;
            if (m_pause_ctr <= 0) {
                m_crossings_left = TOTAL_CROSSINGS;
                m_approach_ctr   = 0;
                m_state          = STATE_REAL_APPROACH;
            }
            break;

        case STATE_REAL_APPROACH: {
            float ramp = (m_approach_ctr < ACCEL_LOOPS)
                             ? static_cast<float>(m_approach_ctr) / static_cast<float>(ACCEL_LOOPS)
                             : 1.0f;
            g_M1->setVelocity(VEL_SIGN * APPROACH_SPEED * ramp);
            g_M2->setVelocity(VEL_SIGN * APPROACH_SPEED * ramp);
            m_approach_ctr++;

            {
                int col_a = m_current_color;
                if (col_a == 3 || col_a == 4 || col_a == 5 || col_a == 7) {
                    m_color_fallback = col_a;
                    m_target_angle   = angleForColor(col_a);
                }
            }

            if (m_approach_ctr > REAL_APPROACH_GUARD && wide_bar_active()) {
                g_M1->setVelocity(0.0f);
                g_M2->setVelocity(0.0f);
                if (m_counting) m_line_count++;
                m_crossing_ctr     = CROSSING_STOP_LOOPS;
                m_action_color     = 0;
                m_color_stable_ctr = 0;
                m_color_pending    = 0;
                m_arm_retract_ctr  = 0;
                if (m_color_fallback == 0) m_color_fallback = m_approach_fallback;
                if (m_color_fallback != 0) m_action_color = m_color_fallback;
                m_state            = STATE_CROSSING_STOP;
            }
            break;
        }

        case STATE_REAL_FOLLOW: {
            g_lf->setMaxWheelVelocity(MAX_SPEED);
            g_lf->setRotationalVelocityControllerGains(KP, KP_NL);

            if (m_real_accel_ctr < RESTART_ACCEL_LOOPS) {
                float ramp = static_cast<float>(m_real_accel_ctr + 1) /
                             static_cast<float>(RESTART_ACCEL_LOOPS);
                g_cmd_M1 = g_cmd_M2 = VEL_SIGN * MAX_SPEED * ramp;
                m_real_accel_ctr++;
            } else {
                if (m_color_delay_ctr > 0) {
                    m_color_delay_ctr--;
                    m_color_stable_ctr = 0;
                    m_color_pending    = 0;
                } else if (!m_slowing) {
                    int col_now = m_current_color;
                    bool is_action = (col_now == 3 || col_now == 4 ||
                                      col_now == 5 || col_now == 7);
                    if (is_action && col_now == m_color_pending) {
                        m_color_stable_ctr++;
                    } else if (is_action) {
                        m_color_pending    = col_now;
                        m_color_stable_ctr = 1;
                        m_color_fallback   = col_now;
                        m_target_angle     = angleForColor(col_now);
                    } else {
                        m_color_pending    = 0;
                        m_color_stable_ctr = 0;
                    }
                    if (m_color_stable_ctr >= (m_color_pending == 5 || m_color_pending == 7 ? 1 : COLOR_STABLE_CNT)) {
                        m_slowing  = true;
                        m_slow_ctr = 0;
                    }
                }
                float slow_ramp = 1.0f;
                if (m_slowing) {
                    float t = (m_slow_ctr < SLOWDOWN_LOOPS)
                                  ? static_cast<float>(m_slow_ctr) / static_cast<float>(SLOWDOWN_LOOPS)
                                  : 1.0f;
                    slow_ramp = 1.0f - t * (1.0f - SLOW_FACTOR);
                    if (m_slow_ctr < SLOWDOWN_LOOPS) m_slow_ctr++;
                }
                g_cmd_M1 = VEL_SIGN * g_lf->getLeftWheelVelocity()  * slow_ramp;
                g_cmd_M2 = VEL_SIGN * g_lf->getRightWheelVelocity() * slow_ramp;
            }
            g_M1->setVelocity(g_cmd_M1);
            g_M2->setVelocity(g_cmd_M2);

            if (m_guard_ctr > 0) {
                m_guard_ctr--;
            } else if (wide_bar_active()) {
                g_M1->setVelocity(0.0f);
                g_M2->setVelocity(0.0f);
                if (m_counting) m_line_count++;
                m_crossing_ctr     = CROSSING_STOP_LOOPS;
                m_action_color     = 0;
                m_color_stable_ctr = 0;
                m_color_pending    = 0;
                m_arm_retract_ctr  = 0;
                if (m_color_fallback != 0) m_action_color = m_color_fallback;
                m_state            = STATE_CROSSING_STOP;
            }
            break;
        }

        // ----------------------------------------------------------------
        // CROSSING_STOP — Dispatch-State (liest Farbe, geht zu STATE_ARM_*)
        // ----------------------------------------------------------------
        case STATE_CROSSING_STOP: {
            g_M1->setVelocity(0.0f);
            g_M2->setVelocity(0.0f);

            // Live Farbe lesen wenn noch keine bekannt
            if (m_action_color == 0) {
                int col_now = m_current_color;
                bool is_action = (col_now == 3 || col_now == 4 ||
                                  col_now == 5 || col_now == 7);
                if (is_action && col_now == m_color_pending) {
                    m_color_stable_ctr++;
                } else if (is_action) {
                    m_color_pending    = col_now;
                    m_color_stable_ctr = 1;
                }
                if (m_color_stable_ctr >= COLOR_STOP_STABLE_CNT) {
                    m_action_color = m_color_pending;
                    setNeoColor(m_action_color);
                }
            }

            if (m_crossing_ctr > 0) m_crossing_ctr--;

            // Fallback bei Timeout (nach Dekrement, damit der letzte Loop greift)
            if (m_crossing_ctr <= 0 && m_action_color == 0) {
                m_action_color = m_color_fallback;
                if (m_action_color != 0) setNeoColor(m_action_color);
            }

            if (m_action_color != 0 || m_crossing_ctr <= 0) {
                m_phase_idx        = 0;
                m_phase_ctr        = 0;
                m_drive_ctr        = 0;
                m_color_stable_ctr = 0;
                m_color_pending    = 0;
                m_color_fallback   = 0;
                m_tray_timeout_ctr = 0;
                m_small_line_mode  = false;
                switch (m_action_color) {
                    case 3: m_state = STATE_RED;    break;
                    case 4: m_state = STATE_YELLOW; break;
                    case 5: m_state = STATE_GREEN;  break;
                    case 7: m_state = STATE_BLUE;   break;
                    default:
                        m_slowing = false; m_slow_ctr = 0; m_action_color = 0;
                        m_color_delay_ctr = COLOR_READ_DELAY;
                        exitWideBar();
                        break;
                }
            }
            break;
        }

        // ================================================================
        // SMALL LINES
        // ================================================================

        case STATE_SMALL_FOLLOW: {
            g_lf->setMaxWheelVelocity(MAX_SPEED);
            g_lf->setRotationalVelocityControllerGains(KP, KP_NL);

            if (m_small_accel_ctr < RESTART_ACCEL_LOOPS) {
                float ramp = static_cast<float>(m_small_accel_ctr + 1) /
                             static_cast<float>(RESTART_ACCEL_LOOPS);
                g_cmd_M1 = g_cmd_M2 = VEL_SIGN * MAX_SPEED * ramp;
                m_small_accel_ctr++;
            } else {
                if (m_guard_ctr > 0 || m_color_delay_ctr > 0) {
                    if (m_color_delay_ctr > 0) m_color_delay_ctr--;
                    m_color_stable_ctr = 0;
                    m_color_pending    = 0;
                } else if (!m_slowing) {
                    int col_now = m_current_color;
                    bool is_action = (col_now == 3 || col_now == 4 ||
                                      col_now == 5 || col_now == 7);
                    if (is_action && col_now == m_color_pending) {
                        m_color_stable_ctr++;
                    } else if (is_action) {
                        m_color_pending    = col_now;
                        m_color_stable_ctr = 1;
                        m_color_fallback   = col_now;
                        m_target_angle     = angleForColor(col_now);
                    } else {
                        m_color_pending    = 0;
                        m_color_stable_ctr = 0;
                    }
                    if (m_color_stable_ctr >= (m_color_pending == 5 || m_color_pending == 7 ? 1 : COLOR_STABLE_CNT)) {
                        m_slowing  = true;
                        m_slow_ctr = 0;
                    }
                }
                float slow_ramp = 1.0f;
                if (m_slowing) {
                    float t = (m_slow_ctr < SLOWDOWN_LOOPS)
                                  ? static_cast<float>(m_slow_ctr) / static_cast<float>(SLOWDOWN_LOOPS)
                                  : 1.0f;
                    slow_ramp = 1.0f - t * (1.0f - SLOW_FACTOR);
                    if (m_slow_ctr < SLOWDOWN_LOOPS) m_slow_ctr++;
                }
                g_cmd_M1 = VEL_SIGN * g_lf->getLeftWheelVelocity()  * slow_ramp;
                g_cmd_M2 = VEL_SIGN * g_lf->getRightWheelVelocity() * slow_ramp;
            }
            g_M1->setVelocity(g_cmd_M1);
            g_M2->setVelocity(g_cmd_M2);

            if (m_guard_ctr > 0) {
                m_guard_ctr--;
            } else if (small_line_active()) {
                g_M1->setVelocity(0.0f);
                g_M2->setVelocity(0.0f);
                if (m_counting) m_line_count++;
                m_small_crossing_ctr = SMALL_CROSSING_STOP_LOOPS;
                m_action_color       = 0;
                m_color_stable_ctr   = 0;
                m_color_pending      = 0;
                m_arm_retract_ctr    = 0;
                if (m_color_fallback != 0) m_action_color = m_color_fallback;
                m_state              = STATE_SMALL_CROSSING_STOP;
            }
            break;
        }

        // ----------------------------------------------------------------
        // SMALL_CROSSING_STOP — Dispatch-State (liest Farbe, geht zu STATE_SMALL_ARM_*)
        // ----------------------------------------------------------------
        case STATE_SMALL_CROSSING_STOP: {
            g_M1->setVelocity(0.0f);
            g_M2->setVelocity(0.0f);

            if (m_action_color == 0) {
                int col_now = m_current_color;
                bool is_action = (col_now == 3 || col_now == 4 ||
                                  col_now == 5 || col_now == 7);
                if (is_action && col_now == m_color_pending) {
                    m_color_stable_ctr++;
                } else if (is_action) {
                    m_color_pending    = col_now;
                    m_color_stable_ctr = 1;
                }
                if (m_color_stable_ctr >= COLOR_STOP_STABLE_CNT) {
                    m_action_color = m_color_pending;
                    setNeoColor(m_action_color);
                }
            }

            if (m_small_crossing_ctr > 0) m_small_crossing_ctr--;

            // Fallback bei Timeout (nach Dekrement, damit der letzte Loop greift)
            if (m_small_crossing_ctr <= 0 && m_action_color == 0) {
                m_action_color = m_color_fallback;
                if (m_action_color != 0) setNeoColor(m_action_color);
            }

            if (m_action_color != 0 || m_small_crossing_ctr <= 0) {
                m_phase_idx        = 0;
                m_phase_ctr        = 0;
                m_drive_ctr        = 0;
                m_color_stable_ctr = 0;
                m_color_pending    = 0;
                m_color_fallback   = 0;
                m_tray_timeout_ctr = 0;
                m_small_line_mode  = true;
                switch (m_action_color) {
                    case 3: m_state = STATE_RED;    break;
                    case 4: m_state = STATE_YELLOW; break;
                    case 5: m_state = STATE_GREEN;  break;
                    case 7: m_state = STATE_BLUE;   break;
                    default:
                        m_slowing = false; m_slow_ctr = 0; m_action_color = 0;
                        m_color_delay_ctr = COLOR_READ_DELAY;
                        exitSmallLine();
                        break;
                }
            }
            break;
        }

        // ----------------------------------------------------------------
        // 4 Farb-States — identische Logik, m_action_color bestimmt Details
        // ----------------------------------------------------------------
        case STATE_RED:
        case STATE_GREEN:
        case STATE_BLUE:
        case STATE_YELLOW: {
            bool needs_drive = (m_action_color == 3 || m_action_color == 4);
            if (needs_drive && m_drive_ctr < ROT_GELB_DRIVE_LOOPS) {
                g_lf->setMaxWheelVelocity(MAX_SPEED);
                g_lf->setRotationalVelocityControllerGains(KP, KP_NL);
                int   ld = m_drive_ctr;
                int   ll = ROT_GELB_DRIVE_LOOPS - ld;
                float ramp;
                if      (ld < ROT_GELB_ACCEL_LOOPS) ramp = static_cast<float>(ld + 1) / ROT_GELB_ACCEL_LOOPS;
                else if (ll <= ROT_GELB_BRAKE_LOOPS) ramp = static_cast<float>(ll)     / ROT_GELB_BRAKE_LOOPS;
                else                                  ramp = 1.0f;
                g_cmd_M1 = VEL_SIGN * g_lf->getLeftWheelVelocity()  * ramp;
                g_cmd_M2 = VEL_SIGN * g_lf->getRightWheelVelocity() * ramp;
                g_M1->setVelocity(g_cmd_M1);
                g_M2->setVelocity(g_cmd_M2);
                m_drive_ctr++;
            } else {
                g_M1->setVelocity(0.0f);
                g_M2->setVelocity(0.0f);
                // Always count up; after 50 loops check angle; force at 100 loops (2s)
                m_tray_timeout_ctr++;
                bool tray_ready = (m_tray_timeout_ctr >= 100) ||
                                  (m_tray_timeout_ctr >= 50 && trayNearTarget());
                if (tray_ready) {
                    if (runDeliverPhase()) {
                        resetArmExitVars();
                        if (m_small_line_mode) exitSmallLine(); else exitWideBar();
                    }
                }
            }
            break;
        }

        case STATE_FINAL_HALT:
            g_M1->setVelocity(0.0f);
            g_M2->setVelocity(0.0f);
            g_servoTray->stop();
            g_servoTray->disable();
            break;
    }
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------
void roboter_v35_04_02_reset(DigitalOut& led)
{
    *g_en = 0;
    g_M1->setVelocity(0.0f);
    g_M2->setVelocity(0.0f);
    g_cmd_M1 = g_cmd_M2 = 0.0f;
    if (g_servo_D1)  g_servo_D1->disable();
    if (g_servo_D2)  g_servo_D2->disable();
    if (g_servoTray) { g_servoTray->stop(); g_servoTray->disable(); }
    if (g_np)        g_np->clear();

    m_state               = STATE_BLIND;
    m_sf360_warmup_ctr    = 0;
    m_sf360_ready         = false;
    m_tray_moving         = false;
    m_target_angle        = 0.0f;
    m_straight_ctr        = 0;
    m_follow_ctr          = 0;
    m_brake_ctr           = 0;
    m_pause_ctr           = 0;
    m_backward_ctr        = 0;
    m_guard_ctr           = 0;
    m_crossing_ctr        = 0;
    m_crossings_left      = 0;
    m_small_crossing_ctr  = 0;
    m_small_crossings_left = 0;
    m_approach_ctr        = 0;
    m_small_accel_ctr     = 0;
    m_real_accel_ctr      = 0;
    m_arm_retract_ctr     = 0;
    m_phase_idx           = 0;
    m_phase_ctr           = 0;
    m_drive_ctr           = 0;
    m_tray_timeout_ctr    = 0;
    m_brake_start_M1      = 0.0f;
    m_brake_start_M2      = 0.0f;
    m_current_color       = 0;
    m_prev_color          = 0;
    m_led_color           = 0;
    m_led_ctr             = 0;
    m_color_log_ctr       = 0;
    m_action_color        = 0;
    m_slowing             = false;
    m_slow_ctr            = 0;
    m_color_delay_ctr     = 0;
    m_color_stable_ctr    = 0;
    m_color_pending       = 0;
    m_color_fallback      = 0;
    m_approach_fallback   = 0;
    m_home_timer          = 0;
    for (int i = 0; i < 8; i++) m_color_log[i] = 0;
    led = 0;
}

// ---------------------------------------------------------------------------
// Print (~5 Hz)
// ---------------------------------------------------------------------------
void roboter_v35_04_02_print()
{
    const char* s =
        (m_state == STATE_BLIND)               ? "BLIND       " :
        (m_state == STATE_STRAIGHT)            ? "STRAIGHT    " :
        (m_state == STATE_TURN_SPOT)           ? "TURN_SPOT   " :
        (m_state == STATE_FOLLOW)              ? "FOLLOW      " :
        (m_state == STATE_FOLLOW_1S)           ? "FOLLOW_1S   " :
        (m_state == STATE_BRAKE)               ? "BRAKE       " :
        (m_state == STATE_PAUSE)               ? "PAUSE       " :
        (m_state == STATE_BACKWARD)            ? "BACKWARD    " :
        (m_state == STATE_REAL_START_PAUSE)    ? "START_PAUSE " :
        (m_state == STATE_REAL_APPROACH)       ? "APPROACH    " :
        (m_state == STATE_REAL_FOLLOW)         ? "REAL_FOLLOW " :
        (m_state == STATE_CROSSING_STOP)       ? "CROSS_STOP  " :
        (m_state == STATE_SMALL_FOLLOW)        ? "SMALL_FOLLOW" :
        (m_state == STATE_SMALL_CROSSING_STOP) ? "SMALL_STOP  " :
        (m_state == STATE_RED)                 ? "RED         " :
        (m_state == STATE_GREEN)               ? "GREEN       " :
        (m_state == STATE_BLUE)                ? "BLUE        " :
        (m_state == STATE_YELLOW)              ? "YELLOW      " : "FINAL_HALT  ";

    float tray_angle = g_servoTray ? g_servoTray->getCurrentAngle() : 0.0f;
    printf("State: %s ph=%d dr=%d | wide=%d small=%d | tray=%.1f/%.1f | col=%-5s act=%-5s\n",
           s,
           m_phase_idx,
           m_drive_ctr,
           m_crossings_left,
           m_small_crossings_left,
           tray_angle,
           m_target_angle,
           ColorSensor::getColorString(m_current_color),
           ColorSensor::getColorString(m_action_color));
}

#endif // PROTOTYPE_03_V35_04_02
