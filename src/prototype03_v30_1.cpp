// CargoSweep — PROTOTYPE_03_V30_1
// Kopie von PROTOTYPE_03_V23_2 mit ServoFeedback360 (Parallax Feedback 360°)
// anstelle des alten Endschalter-basierten 360°-Servos.
// Endschalter-ISR, Click-Counting und 5-Click-Logik wurden vollständig entfernt.
// Der Drehteller wird jetzt per absolutem Winkel gesteuert: jede Farbe hat einen
// festen Slot-Winkel (ROT=0°, GRÜN=90°, BLAU=180°, GELB=270°).
// Nach jeder Aufnahme rückt der Drehteller um +90° weiter.
#include "test_config.h"

#ifdef PROTOTYPE_03_V30_1

#include "prototype03_v30_1.h"
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
// Linienfolger PID (aus v23)
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
static const int STOP_GUARD           = static_cast<int>(75  * SPEED_SCALE);
static const int START_GUARD          = static_cast<int>(300 * SPEED_SCALE);
static const int SLOWDOWN_LOOPS       = 13;
static const float SLOW_FACTOR        = 0.2f;
static const int COLOR_READ_DELAY     = 50;
static const int COLOR_STABLE_CNT     = 3;
static const int COLOR_READ_PHASE     = 25;
static const int COLOR_STOP_STABLE_CNT = 3;

// ---------------------------------------------------------------------------
// Programm-Konstanten
// ---------------------------------------------------------------------------
static const int   TOTAL_CROSSINGS       = 4;
static const int   CROSSING_STOP_LOOPS   = 100;
static const int   SMALL_FOLLOW_START_GUARD   = static_cast<int>(781 * SPEED_SCALE);
static const int   SMALL_REENTRY_GUARD        = static_cast<int>(100 * SPEED_SCALE);
static const int   TOTAL_SMALL_CROSSINGS      = 4;
static const int   SMALL_CROSSING_STOP_LOOPS  = 100;

// 180° Servo-Positionen (aus v29, TEST_ARM_SEQUENCE validiert 2026-04-30)
static const float D1_RETRACTED       = 0.1f;
static const float D1_EXTENDED        = 0.80f;
static const float D2_UP              = 1.0f;
static const float D2_PARTIAL_DOWN    = 0.50f;
static const float D2_DOWN_BLAU       = 0.22f;
static const float D2_DOWN_GRUEN      = 0.05f;

// PICKUP / DELIVER Timing (aus v29)
static const int   PICKUP_PAUSE_LOOPS  = 50;
static const int   JIGGLE_LOOPS        = 50;
static const int   RAISE_LOOPS         = 50;   // D2 hoch (keine Beschleunigungsrampe)
static const int   RETRACT_LOOPS       = 120;  // D1 einfahren: 0.65→0.0 bei 0.3f/s ≈ 2.2 s
static const float JIGGLE_AMPL_DEG    = 10.0f;
static const float D1_JIGGLE_OFFSET   = 0.05f;
static const int   SF360_TIMEOUT_LOOPS = 250;

static const int   ROT_GELB_DRIVE_LOOPS  = 55;
static const int   ROT_GELB_ACCEL_LOOPS  = 10;
static const int   ROT_GELB_BRAKE_LOOPS  = 12;

static const float SENSOR_THRESHOLD = 0.40f;

// ---------------------------------------------------------------------------
// ServoFeedback360 Parameter (Parallax Feedback 360°, PB_12/PC_2)
// ---------------------------------------------------------------------------
static const float SF360_KP          = 0.005f;
static const float SF360_TOL_DEG     = 2.1f;
static const float SF360_MIN_SPEED   = 0.01f;
static const float SF360_OFFSET      = 110.0f;
static const int   SF360_WARMUP_LOOPS = 25;
static const float JIGGLE_DEG        = 5.0f;   // ±5° Jiggle-Amplitude

// Drehteller-Winkel für jede Farbe (aus v29)
static const float COLOR_ANGLE[4] = {0.0f, 90.0f, 180.0f, 270.0f};
// Mapping: colorToSlot(3)=0 (ROT→0°), colorToSlot(5)=1 (GRÜN→90°),
//          colorToSlot(7)=2 (BLAU→180°), colorToSlot(4)=3 (GELB→270°)

// ---------------------------------------------------------------------------
// States (identisch zu v23)
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
    STATE_ROT_GELB_DRIVE      = 16,
    STATE_ROT_GELB_PAUSE      = 17
};

// ---------------------------------------------------------------------------
// Hardware-Pointer
// ---------------------------------------------------------------------------
static DCMotor*          g_M1        = nullptr;
static DCMotor*          g_M2        = nullptr;
static DigitalOut*       g_en        = nullptr;
static LineFollower*     g_lf        = nullptr;
static ColorSensor*      g_cs        = nullptr;
static Servo*            g_servo_D1  = nullptr;   // 180° Arm horizontal — PC_8 (D1)
static Servo*            g_servo_D2  = nullptr;   // 180° Arm vertikal  — PB_2  (D0)
static ServoFeedback360* g_servoTray = nullptr;   // Parallax 360° — PB_12/PC_2
static NeoPixel*         g_np        = nullptr;

// ---------------------------------------------------------------------------
// Laufzeit-Zustand
// ---------------------------------------------------------------------------
static State m_state = STATE_BLIND;

// Drehteller (ServoFeedback360)
static int   m_sf360_warmup_ctr = 0;
static bool  m_sf360_ready      = false;
static float m_target_angle     = 0.0f;   // aktueller Slot-Winkel des Drehtellers

// Zähler
static int   m_straight_ctr        = 0;
static int   m_follow_ctr          = 0;
static int   m_brake_ctr           = 0;
static int   m_pause_ctr           = 0;
static int   m_backward_ctr        = 0;
static int   m_guard_ctr           = 0;
static int   m_crossing_ctr        = 0;
static int   m_crossings_left      = 0;
static int   m_tray_timeout_ctr    = 0;  // Schutz falls isAtTarget() nie true wird
static int   m_small_crossing_ctr  = 0;
static int   m_small_crossings_left = 0;
static int   m_approach_ctr        = 0;
static int   m_small_accel_ctr     = 0;
static int   m_real_accel_ctr      = 0;
static int   m_arm_retract_ctr     = 0;  // 1 = Arm-Sequenz läuft, 0 = fertig
static int   m_phase_idx           = 0;  // Phasen-Index für runPickupPhase/runDeliverPhase
static int   m_phase_ctr           = 0;  // Phasen-Zähler
static float m_brake_start_M1      = 0.0f;
static float m_brake_start_M2      = 0.0f;
static float g_cmd_M1              = 0.0f;
static float g_cmd_M2              = 0.0f;

// Farbe
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
static bool  m_rot_gelb_is_small   = false;
static int   m_rot_gelb_color      = 0;
static int   m_color_fallback      = 0;

// Drehteller-Bewegung
static bool  m_tray_moving         = false;  // true während Servo aktiv sein soll
static int   m_home_timer          = 0;      // 3s Ausrichtungs-Timer nach all_sensors_active()

// ---------------------------------------------------------------------------
// Hilfsfunktionen (identisch zu v23)
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

static float extensionForColor(int /*c*/)
{
    return D1_EXTENDED;
}

static int colorToSlot(int c)
{
    switch (c) {
        case 3: return 0; // ROT   → 0°
        case 5: return 1; // GRÜN  → 90°
        case 7: return 2; // BLAU  → 180°
        case 4: return 3; // GELB  → 270°
        default: return -1;
    }
}

static void trayMoveTo(float deg)
{
    if (!m_tray_moving) {
        g_servoTray->enable(0.5f);  // nur einschalten wenn bisher deaktiviert
        m_tray_moving = true;
    }
    g_servoTray->moveToAngle(deg);
}

static void trayStop()
{
    g_servoTray->stop();
    g_servoTray->disable();
    m_tray_moving = false;
}

// ---------------------------------------------------------------------------
// Jiggle-Hilfsfunktion: ruft moveToAngle() im ±AMPL-Muster auf.
// tick = laufender Zähler (phasen-übergreifend). Nur tray, kein enable().
// ---------------------------------------------------------------------------
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
// Arm-Sequenz Pickup
// Phase 0: Tray → target (warten)
// Phase 1: D2 partial + Jiggle startet sofort
// Phase 2: D1 extend  + Jiggle weiter
// Phase 3: D2 voll    + Jiggle weiter
// Phase 4: D2 hoch    + Jiggle stoppt (→ target zentrieren)
// Phase 5: D1 einfahren → return true
// ---------------------------------------------------------------------------
static bool runPickupPhase()
{
    static int s_jiggle_tick = 0;

    // Phase 0: Tray zum Zielwinkel fahren (trayMoveTo sichert enable)
    if (m_phase_idx == 0) {
        if (m_phase_ctr == 0)
            trayMoveTo(m_target_angle);
        m_phase_ctr++;
        if (g_servoTray->isAtTarget() || m_phase_ctr >= SF360_TIMEOUT_LOOPS) {
            m_phase_idx   = 1;
            m_phase_ctr   = 0;
            s_jiggle_tick = 0;
        }
        return false;
    }
    // Phase 1: D2 partial down — Jiggle startet sofort (nur Tray, D1 noch eingefahren)
    if (m_phase_idx == 1) {
        if (m_phase_ctr == 0) {
            g_servo_D1->enable(D1_RETRACTED);
            g_servo_D2->enable(D2_PARTIAL_DOWN);
        }
        applyJiggleTick(s_jiggle_tick);
        s_jiggle_tick++;
        m_phase_ctr++;
        if (m_phase_ctr >= PICKUP_PAUSE_LOOPS) { m_phase_idx = 2; m_phase_ctr = 0; }
        return false;
    }
    // Phase 2: D1 ausfahren — Jiggle weiter + D1 synchron mitbewegen
    if (m_phase_idx == 2) {
        const float base = extensionForColor(m_action_color);
        if (m_phase_ctr == 0)
            g_servo_D1->setPulseWidth(base);
        applyJiggleTick(s_jiggle_tick);
        const int jp2 = s_jiggle_tick % JIGGLE_LOOPS;
        if      (jp2 == 0)                  g_servo_D1->setPulseWidth(base + D1_JIGGLE_OFFSET);
        else if (jp2 == JIGGLE_LOOPS / 3)   g_servo_D1->setPulseWidth(base - D1_JIGGLE_OFFSET);
        else if (jp2 == 2*JIGGLE_LOOPS/3)   g_servo_D1->setPulseWidth(base);
        s_jiggle_tick++;
        m_phase_ctr++;
        if (m_phase_ctr >= PICKUP_PAUSE_LOOPS) { m_phase_idx = 3; m_phase_ctr = 0; }
        return false;
    }
    // Phase 3: D2 voll runter — Jiggle weiter
    if (m_phase_idx == 3) {
        const float base = extensionForColor(m_action_color);
        if (m_phase_ctr == 0)
            g_servo_D2->setPulseWidth(depthForColor(m_action_color));
        applyJiggleTick(s_jiggle_tick);
        const int jp3 = s_jiggle_tick % JIGGLE_LOOPS;
        if      (jp3 == 0)                  g_servo_D1->setPulseWidth(base + D1_JIGGLE_OFFSET);
        else if (jp3 == JIGGLE_LOOPS / 3)   g_servo_D1->setPulseWidth(base - D1_JIGGLE_OFFSET);
        else if (jp3 == 2*JIGGLE_LOOPS/3)   g_servo_D1->setPulseWidth(base);
        s_jiggle_tick++;
        m_phase_ctr++;
        if (m_phase_ctr >= PICKUP_PAUSE_LOOPS) { m_phase_idx = 4; m_phase_ctr = 0; }
        return false;
    }
    // Phase 4: D2 hoch — Jiggle stoppt, Tray zurück zur Mitte
    if (m_phase_idx == 4) {
        if (m_phase_ctr == 0) {
            g_servo_D2->setPulseWidth(D2_UP);
            g_servoTray->moveToAngle(m_target_angle);  // Jiggle beenden
        }
        m_phase_ctr++;
        if (m_phase_ctr >= PICKUP_PAUSE_LOOPS) { m_phase_idx = 5; m_phase_ctr = 0; }
        return false;
    }
    // Phase 5: D1 einfahren
    if (m_phase_idx == 5) {
        if (m_phase_ctr == 0) {
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

// ---------------------------------------------------------------------------
// Arm-Sequenz Deliver (aus v29)
// Phase 0: Tray warten → Phase 1: D1+D2 runter → Phase 2: Jiggle → Phase 3: D1+D2 hoch
// ---------------------------------------------------------------------------
static bool runDeliverPhase()
{
    if (m_phase_idx == 0) {
        if (m_phase_ctr == 0)
            trayMoveTo(m_target_angle);   // sichert enable falls Servo war aus
        m_phase_ctr++;
        if (g_servoTray->isAtTarget() || m_phase_ctr >= SF360_TIMEOUT_LOOPS) {
            m_phase_idx = 1; m_phase_ctr = 0;
        }
        return false;
    }
    if (m_phase_idx == 1) {
        if (m_phase_ctr == 0) {
            g_servo_D1->enable(extensionForColor(m_action_color));
            g_servo_D2->enable(depthForColor(m_action_color));
        }
        m_phase_ctr++;
        if (m_phase_ctr >= PICKUP_PAUSE_LOOPS) { m_phase_idx = 2; m_phase_ctr = 0; }
        return false;
    }
    if (m_phase_idx == 2) {
        const float base = extensionForColor(m_action_color);
        const int   ph1  = JIGGLE_LOOPS / 3;
        const int   ph2  = 2 * JIGGLE_LOOPS / 3;
        if (m_phase_ctr == 0) {
            trayMoveTo(m_target_angle - JIGGLE_AMPL_DEG);
            g_servo_D1->setPulseWidth(base + D1_JIGGLE_OFFSET);
        } else if (m_phase_ctr == ph1) {
            trayMoveTo(m_target_angle + JIGGLE_AMPL_DEG);
            g_servo_D1->setPulseWidth(base - D1_JIGGLE_OFFSET);
        } else if (m_phase_ctr == ph2) {
            trayMoveTo(m_target_angle);
            g_servo_D1->setPulseWidth(base);
        }
        m_phase_ctr++;
        if (m_phase_ctr >= JIGGLE_LOOPS) { m_phase_idx = 3; m_phase_ctr = 0; }
        return false;
    }
    if (m_phase_idx == 3) {
        if (m_phase_ctr == 0) {
            g_servo_D2->setPulseWidth(D2_UP);
            g_servo_D1->setMaxAcceleration(1.0e6f); // Rampe aufheben → sofort einfahren
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

// Drehteller-Service: Warmup abwarten, dann update() aufrufen wenn m_tray_moving=true.
// Kein auto-disable — trayStop() wird explizit aufgerufen wenn fertig.
static void serviceTray()
{
    if (!g_servoTray) return;

    // Warmup-Phase: Feedback kalibrieren (stop() → kein Bewegungsbefehl)
    if (!m_sf360_ready) {
        g_servoTray->stop();
        m_sf360_warmup_ctr++;
        if (m_sf360_warmup_ctr >= SF360_WARMUP_LOOPS &&
            g_servoTray->isFeedbackValid()) {
            m_sf360_ready = true;
            g_servoTray->disable();   // nach Warmup still halten bis all_sensors_active()
        }
        return;
    }

    // P-Regler nur wenn explizit aktiviert
    if (!m_tray_moving) return;

    g_servoTray->update();
}

// Drehteller um 90° weiterschalten und m_target_angle aktualisieren.
static void advanceTray()
{
    m_target_angle = fmodf(m_target_angle + 90.0f, 360.0f);
    trayMoveTo(m_target_angle);
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
void roboter_v30_1_init(int /*loops_per_second*/)
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
    g_servo_D1->calibratePulseMinMax(0.0500f, 0.1050f);   // M20 horizontal
    g_servo_D1->setMaxAcceleration(0.3f);
    g_servo_D2->calibratePulseMinMax(0.0200f, 0.1310f);   // M21 vertikal
    g_servo_D1->enable(D1_RETRACTED);   // Arm von Anfang an eingefahren
    g_servo_D2->enable(D2_UP);          // Arm von Anfang an oben

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
    m_rot_gelb_is_small   = false;
    m_rot_gelb_color      = 0;
    m_color_fallback      = 0;
    for (int i = 0; i < 8; i++) m_color_log[i] = 0;
    g_M1->setVelocity(0.0f);
    g_M2->setVelocity(0.0f);
}

// ---------------------------------------------------------------------------
// Task (50 Hz)
// ---------------------------------------------------------------------------
void roboter_v30_1_task(DigitalOut& led)
{
    // --- Farbsensor ---
    m_current_color = g_cs->getColor();

    // --- NeoPixel ---
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

    // --- LED Blinkcode ---
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

    // Drehteller-Regelkreis jeden Loop bedienen.
    serviceTray();

    // Ausrichtungs-Timer: 3s nach 0°-Ausrichtung → Servo abschalten
    if (m_home_timer > 0) {
        if (--m_home_timer == 0) trayStop();
    }

    switch (m_state) {

        // ----------------------------------------------------------------
        case STATE_BLIND:
            g_M1->setVelocity(VEL_SIGN * BLIND_SPEED);
            g_M2->setVelocity(VEL_SIGN * BLIND_SPEED);
            if (all_sensors_active() && m_sf360_ready) {
                trayMoveTo(0.0f);       // Tray zu 0° (ROT-Slot)
                m_home_timer   = 150;   // 3 s ausgerichtet halten
                m_straight_ctr = STRAIGHT_LOOPS;
                m_state        = STATE_STRAIGHT;
            }
            break;

        // ----------------------------------------------------------------
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

        // ----------------------------------------------------------------
        case STATE_TURN_SPOT:
            g_M1->setVelocity(VEL_SIGN * -TURN_SPEED);
            g_M2->setVelocity(VEL_SIGN *  TURN_SPEED);
            if (center_sensors_active()) {
                g_lf->setRotationalVelocityControllerGains(KP_FOLLOW, KP_NL_FOLLOW);
                m_guard_ctr = START_GUARD;
                m_state     = STATE_FOLLOW;
            }
            break;

        // ----------------------------------------------------------------
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

        // ----------------------------------------------------------------
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

        // ----------------------------------------------------------------
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

        // ----------------------------------------------------------------
        case STATE_PAUSE:
            g_M1->setVelocity(0.0f);
            g_M2->setVelocity(0.0f);
            m_pause_ctr--;
            if (m_pause_ctr <= 0) {
                m_backward_ctr = BACKWARD_LOOPS;
                m_state        = STATE_BACKWARD;
            }
            break;

        // ----------------------------------------------------------------
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
            m_pause_ctr--;
            if (m_pause_ctr <= 0) {
                m_crossings_left = TOTAL_CROSSINGS;
                m_approach_ctr   = 0;
                m_state          = STATE_REAL_APPROACH;
            }
            break;

        // ----------------------------------------------------------------
        case STATE_REAL_APPROACH: {
            float ramp = (m_approach_ctr < ACCEL_LOOPS)
                             ? static_cast<float>(m_approach_ctr) / static_cast<float>(ACCEL_LOOPS)
                             : 1.0f;
            g_M1->setVelocity(VEL_SIGN * APPROACH_SPEED * ramp);
            g_M2->setVelocity(VEL_SIGN * APPROACH_SPEED * ramp);
            m_approach_ctr++;

            if (m_approach_ctr > ACCEL_LOOPS) {
                int col_now = g_cs->getColor();
                bool is_action = (col_now == 3 || col_now == 4 ||
                                  col_now == 5 || col_now == 7);
                if (is_action && col_now == m_color_pending) {
                    m_color_stable_ctr++;
                } else if (is_action) {
                    m_color_pending    = col_now;
                    m_color_stable_ctr = 1;
                    m_color_fallback   = col_now;
                } else {
                    m_color_pending    = 0;
                    m_color_stable_ctr = 0;
                }
            }

            if (m_approach_ctr > ACCEL_LOOPS && wide_bar_active()) {
                g_M1->setVelocity(0.0f);
                g_M2->setVelocity(0.0f);
                m_crossings_left--;
                m_crossing_ctr     = CROSSING_STOP_LOOPS;
                m_action_color     = 0;
                m_color_stable_ctr = 0;
                m_color_pending    = 0;
                m_arm_retract_ctr  = 0;
                m_state            = STATE_CROSSING_STOP;
            }
            break;
        }

        // ----------------------------------------------------------------
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
                    int col_now = g_cs->getColor();
                    bool is_action = (col_now == 3 || col_now == 4 ||
                                      col_now == 5 || col_now == 7);
                    if (is_action && col_now == m_color_pending) {
                        m_color_stable_ctr++;
                    } else if (is_action) {
                        m_color_pending    = col_now;
                        m_color_stable_ctr = 1;
                        m_color_fallback   = col_now;
                    } else {
                        m_color_pending    = 0;
                        m_color_stable_ctr = 0;
                    }
                    if (m_color_stable_ctr >= COLOR_STABLE_CNT) {
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
                m_crossings_left--;
                m_crossing_ctr     = CROSSING_STOP_LOOPS;
                m_action_color     = 0;
                m_color_stable_ctr = 0;
                m_color_pending    = 0;
                m_arm_retract_ctr  = 0;
                m_state            = STATE_CROSSING_STOP;
            }
            break;
        }

        // ----------------------------------------------------------------
        // CROSSING_STOP
        // ----------------------------------------------------------------
        case STATE_CROSSING_STOP: {
            g_M1->setVelocity(0.0f);
            g_M2->setVelocity(0.0f);

            // Farbe lesen (erste 0.5 s)
            if (m_crossing_ctr > CROSSING_STOP_LOOPS - COLOR_READ_PHASE) {
                int col_now = g_cs->getColor();
                bool is_action = (col_now == 3 || col_now == 4 ||
                                  col_now == 5 || col_now == 7);
                if (is_action && col_now == m_color_pending) {
                    m_color_stable_ctr++;
                } else if (is_action) {
                    m_color_pending    = col_now;
                    m_color_stable_ctr = 1;
                }
                if (m_color_stable_ctr >= COLOR_STOP_STABLE_CNT && m_action_color == 0)
                    m_action_color = m_color_pending;
            }

            // Ende Farblese-Phase: Farbe fixieren + Drehteller auf Winkel + Arm starten
            if (m_crossing_ctr == CROSSING_STOP_LOOPS - COLOR_READ_PHASE) {
                if (m_action_color == 0) m_action_color = m_color_fallback;
                // Drehteller zum Farb-spezifischen Winkel (aus v29)
                int slot = colorToSlot(m_action_color);
                if (slot >= 0) {
                    m_target_angle = COLOR_ANGLE[slot];
                    trayMoveTo(m_target_angle);
                }
                if (m_action_color == 5 || m_action_color == 7) {
                    m_phase_idx       = 0;
                    m_phase_ctr       = 0;
                    m_arm_retract_ctr = 1;  // Arm-Sequenz aktiv
                }
            }

            // Arm-Sequenz ausführen (GRÜN / BLAU) — läuft bis runPickupPhase() true zurückgibt
            if (m_arm_retract_ctr == 1) {
                if (runPickupPhase()) {
                    m_arm_retract_ctr = 0;
                    advanceTray();  // +90° zum nächsten Slot
                }
            }

            if (m_crossing_ctr > 0) m_crossing_ctr--;

            // Exit: Arm fertig + Drehteller am Ziel (oder Timeout nach 5 s)
            if (m_crossing_ctr <= 0 && m_arm_retract_ctr == 0) {
                m_tray_timeout_ctr++;
                bool tray_ok = g_servoTray->isAtTarget() ||
                               m_tray_timeout_ctr >= SF360_TIMEOUT_LOOPS;
                if (tray_ok) {
                int exit_color     = m_action_color;
                trayStop();            // Servo stoppen + deaktivieren
                m_slowing          = false;
                m_slow_ctr         = 0;
                m_action_color     = 0;
                m_color_delay_ctr  = COLOR_READ_DELAY;
                m_color_stable_ctr = 0;
                m_color_pending    = 0;
                m_arm_retract_ctr  = 0;
                m_phase_idx        = 0;
                m_phase_ctr        = 0;
                m_tray_timeout_ctr = 0;
                if (exit_color == 3 || exit_color == 4) {
                    m_rot_gelb_color    = exit_color;
                    m_rot_gelb_is_small = false;
                    m_approach_ctr      = 0;
                    g_lf->setRotationalVelocityControllerGains(KP, KP_NL);
                    g_lf->setMaxWheelVelocity(MAX_SPEED);
                    m_state = STATE_ROT_GELB_DRIVE;
                } else if (m_crossings_left == 0) {
                    m_small_crossings_left = TOTAL_SMALL_CROSSINGS;
                    m_guard_ctr            = SMALL_FOLLOW_START_GUARD;
                    m_small_accel_ctr      = 0;
                    m_state                = STATE_SMALL_FOLLOW;
                } else {
                    m_guard_ctr      = STOP_GUARD;
                    m_real_accel_ctr = 0;
                    m_state          = STATE_REAL_FOLLOW;
                }
                } // tray_ok
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
                    int col_now = g_cs->getColor();
                    bool is_action = (col_now == 3 || col_now == 4 ||
                                      col_now == 5 || col_now == 7);
                    if (is_action && col_now == m_color_pending) {
                        m_color_stable_ctr++;
                    } else if (is_action) {
                        m_color_pending    = col_now;
                        m_color_stable_ctr = 1;
                        m_color_fallback   = col_now;
                    } else {
                        m_color_pending    = 0;
                        m_color_stable_ctr = 0;
                    }
                    if (m_color_stable_ctr >= COLOR_STABLE_CNT) {
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
                m_small_crossings_left--;
                m_small_crossing_ctr = SMALL_CROSSING_STOP_LOOPS;
                m_color_fallback     = 0;
                m_action_color       = 0;
                m_color_stable_ctr   = 0;
                m_color_pending      = 0;
                m_arm_retract_ctr    = 0;
                m_state              = STATE_SMALL_CROSSING_STOP;
            }
            break;
        }

        // ----------------------------------------------------------------
        case STATE_SMALL_CROSSING_STOP: {
            g_M1->setVelocity(0.0f);
            g_M2->setVelocity(0.0f);

            // Farbe lesen
            if (m_small_crossing_ctr > SMALL_CROSSING_STOP_LOOPS - COLOR_READ_PHASE) {
                int col_now = g_cs->getColor();
                bool is_action = (col_now == 3 || col_now == 4 ||
                                  col_now == 5 || col_now == 7);
                if (is_action && col_now == m_color_pending) {
                    m_color_stable_ctr++;
                } else if (is_action) {
                    m_color_pending    = col_now;
                    m_color_stable_ctr = 1;
                }
                if (m_color_stable_ctr >= COLOR_STOP_STABLE_CNT && m_action_color == 0)
                    m_action_color = m_color_pending;
            }

            // Ende Farblese-Phase: Drehteller auf Winkel + Arm starten
            if (m_small_crossing_ctr == SMALL_CROSSING_STOP_LOOPS - COLOR_READ_PHASE) {
                if (m_action_color == 0) m_action_color = m_color_fallback;
                // Drehteller zum Farb-spezifischen Winkel (aus v29)
                int slot = colorToSlot(m_action_color);
                if (slot >= 0) {
                    m_target_angle = COLOR_ANGLE[slot];
                    trayMoveTo(m_target_angle);
                }
                if (m_action_color == 5 || m_action_color == 7) {
                    m_phase_idx       = 0;
                    m_phase_ctr       = 0;
                    m_arm_retract_ctr = 1;
                }
            }

            // Deliver-Sequenz ausführen (GRÜN / BLAU)
            if (m_arm_retract_ctr == 1) {
                if (runDeliverPhase()) {
                    m_arm_retract_ctr = 0;
                    advanceTray();  // +90° zum nächsten Slot
                }
            }

            if (m_small_crossing_ctr > 0) m_small_crossing_ctr--;

            if (m_small_crossing_ctr <= 0 && m_arm_retract_ctr == 0) {
                m_tray_timeout_ctr++;
                bool tray_ok = g_servoTray->isAtTarget() ||
                               m_tray_timeout_ctr >= SF360_TIMEOUT_LOOPS;
                if (tray_ok) {
                int exit_color     = m_action_color;
                trayStop();            // Servo stoppen + deaktivieren
                m_slowing          = false;
                m_slow_ctr         = 0;
                m_action_color     = 0;
                m_color_delay_ctr  = COLOR_READ_DELAY;
                m_color_stable_ctr = 0;
                m_color_pending    = 0;
                m_arm_retract_ctr  = 0;
                m_phase_idx        = 0;
                m_phase_ctr        = 0;
                m_tray_timeout_ctr = 0;
                if (exit_color == 3 || exit_color == 4) {
                    m_rot_gelb_color    = exit_color;
                    m_rot_gelb_is_small = true;
                    m_approach_ctr      = 0;
                    g_lf->setRotationalVelocityControllerGains(KP, KP_NL);
                    g_lf->setMaxWheelVelocity(MAX_SPEED);
                    m_state = STATE_ROT_GELB_DRIVE;
                } else if (m_small_crossings_left == 0) {
                    m_state = STATE_FINAL_HALT;
                } else {
                    m_guard_ctr       = SMALL_REENTRY_GUARD;
                    m_small_accel_ctr = 0;
                    m_state           = STATE_SMALL_FOLLOW;
                }
                } // tray_ok
            }
            break;
        }

        // ----------------------------------------------------------------
        case STATE_ROT_GELB_DRIVE: {
            g_lf->setMaxWheelVelocity(MAX_SPEED);
            g_lf->setRotationalVelocityControllerGains(KP, KP_NL);
            int   loops_done = m_approach_ctr;
            int   loops_left = ROT_GELB_DRIVE_LOOPS - loops_done;
            float ramp;
            if (loops_done < ROT_GELB_ACCEL_LOOPS) {
                ramp = static_cast<float>(loops_done + 1) / static_cast<float>(ROT_GELB_ACCEL_LOOPS);
            } else if (loops_left <= ROT_GELB_BRAKE_LOOPS) {
                ramp = static_cast<float>(loops_left) / static_cast<float>(ROT_GELB_BRAKE_LOOPS);
            } else {
                ramp = 1.0f;
            }
            g_cmd_M1 = VEL_SIGN * g_lf->getLeftWheelVelocity()  * ramp;
            g_cmd_M2 = VEL_SIGN * g_lf->getRightWheelVelocity() * ramp;
            g_M1->setVelocity(g_cmd_M1);
            g_M2->setVelocity(g_cmd_M2);
            m_approach_ctr++;
            if (m_approach_ctr >= ROT_GELB_DRIVE_LOOPS) {
                g_M1->setVelocity(0.0f);
                g_M2->setVelocity(0.0f);
                m_phase_idx       = 0;
                m_phase_ctr       = 0;
                m_arm_retract_ctr = 1;  // Arm-Sequenz aktiv
                m_state           = STATE_ROT_GELB_PAUSE;
            }
            break;
        }

        // ----------------------------------------------------------------
        // ROT_GELB_PAUSE — v29 Arm-Sequenz via runPickupPhase()
        // ----------------------------------------------------------------
        case STATE_ROT_GELB_PAUSE: {
            static State prev_state = STATE_BLIND;  // verschieden von ROT_GELB_PAUSE → Entry-Code läuft beim ersten Eintritt
            if (prev_state != m_state) {
                trayMoveTo(m_target_angle);  // Servo aktivieren + auf Farbwinkel
                m_phase_idx       = 0;
                m_phase_ctr       = 0;
                m_arm_retract_ctr = 1;
                prev_state        = m_state;
            }
            g_M1->setVelocity(0.0f);
            g_M2->setVelocity(0.0f);

            if (m_arm_retract_ctr == 1) {
                if (runPickupPhase()) {
                    m_arm_retract_ctr = 0;
                    advanceTray();  // +90° zum nächsten Slot
                }
            }

            // Exit: Arm fertig + Drehteller am Ziel (oder Timeout nach 5 s)
            if (m_arm_retract_ctr == 0) {
                m_tray_timeout_ctr++;
                bool tray_ok = g_servoTray->isAtTarget() ||
                               m_tray_timeout_ctr >= SF360_TIMEOUT_LOOPS;
            if (tray_ok) {
                trayStop();            // Servo stoppen + deaktivieren
                m_tray_timeout_ctr = 0;
                m_color_delay_ctr = COLOR_READ_DELAY;
                if (m_rot_gelb_is_small) {
                    if (m_small_crossings_left == 0) {
                        m_state = STATE_FINAL_HALT;
                    } else {
                        m_guard_ctr       = SMALL_REENTRY_GUARD;
                        m_small_accel_ctr = 0;
                        m_state           = STATE_SMALL_FOLLOW;
                    }
                } else {
                    if (m_crossings_left == 0) {
                        m_small_crossings_left = TOTAL_SMALL_CROSSINGS;
                        m_guard_ctr            = SMALL_FOLLOW_START_GUARD;
                        m_small_accel_ctr      = 0;
                        m_state                = STATE_SMALL_FOLLOW;
                    } else {
                        m_guard_ctr      = STOP_GUARD;
                        m_real_accel_ctr = 0;
                        m_state          = STATE_REAL_FOLLOW;
                    }
                }
            } // tray_ok
            }
            break;
        }

        // ----------------------------------------------------------------
        case STATE_FINAL_HALT:
            g_M1->setVelocity(0.0f);
            g_M2->setVelocity(0.0f);
            break;
    }
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------
void roboter_v30_1_reset(DigitalOut& led)
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
    m_rot_gelb_is_small   = false;
    m_rot_gelb_color      = 0;
    m_color_fallback      = 0;
    m_tray_moving         = false;
    m_home_timer          = 0;
    for (int i = 0; i < 8; i++) m_color_log[i] = 0;
    led = 0;
}

// ---------------------------------------------------------------------------
// Print (~5 Hz)
// ---------------------------------------------------------------------------
void roboter_v30_1_print()
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
        (m_state == STATE_ROT_GELB_DRIVE)      ? "RG_DRIVE    " :
        (m_state == STATE_ROT_GELB_PAUSE)      ? "RG_PAUSE    " : "FINAL_HALT  ";

    float tray_angle = g_servoTray ? g_servoTray->getCurrentAngle() : 0.0f;
    printf("State: %s | wide=%d small=%d | tray=%.1f/%.1f | col=%-5s act=%-5s\n",
           s,
           m_crossings_left,
           m_small_crossings_left,
           tray_angle,
           m_target_angle,
           ColorSensor::getColorString(m_current_color),
           ColorSensor::getColorString(m_action_color));
}

#endif // PROTOTYPE_03_V30_1
