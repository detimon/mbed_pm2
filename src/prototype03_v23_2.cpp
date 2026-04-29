// CargoSweep — PROTOTYPE_03_V23_2
// Kopie von PROTOTYPE_02_V23 mit ServoFeedback360 (Parallax Feedback 360°)
// anstelle des alten Endschalter-basierten 360°-Servos.
// Endschalter-ISR, Click-Counting und 5-Click-Logik wurden vollständig entfernt.
// Der Drehteller wird jetzt per absolutem Winkel gesteuert: jede Farbe hat einen
// festen Slot-Winkel (ROT=0°, GRÜN=90°, BLAU=180°, GELB=270°).
// Nach jeder Aufnahme rückt der Drehteller um +90° weiter.
#include "test_config.h"

#ifdef PROTOTYPE_03_V23_2

#include "prototype03_v23_2.h"
#include "Servo.h"
#include "NeoPixel.h"
#include "ServoFeedback360.h"

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

static const float SERVO_D2_BLAU_DOWN    = 0.38f;
static const float SERVO_D2_GRUEN_DOWN   = 0.22f;
static const float SERVO_D2_PARTIAL_DOWN = 0.85f;

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
static const float SF360_OFFSET      = 70.0f;
static const int   SF360_WARMUP_LOOPS = 25;
static const float JIGGLE_DEG        = 5.0f;   // ±5° Jiggle-Amplitude

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
static int   m_small_crossing_ctr  = 0;
static int   m_small_crossings_left = 0;
static int   m_approach_ctr        = 0;
static int   m_small_accel_ctr     = 0;
static int   m_real_accel_ctr      = 0;
static int   m_arm_retract_ctr     = 0;
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

// Drehteller-Service: Warmup abwarten, dann jeden Loop update() aufrufen.
static void serviceTray()
{
    if (!g_servoTray) return;
    if (!m_sf360_ready) {
        g_servoTray->stop();
        m_sf360_warmup_ctr++;
        if (m_sf360_warmup_ctr >= SF360_WARMUP_LOOPS &&
            g_servoTray->isFeedbackValid()) {
            m_sf360_ready = true;
            g_servoTray->moveToAngle(0.0f); // Heim: Slot 0 (ROT=0°)
        }
        return;
    }
    g_servoTray->update();
}

// Drehteller um 90° weiterschalten und m_target_angle aktualisieren.
static void advanceTray()
{
    m_target_angle = fmodf(m_target_angle + 90.0f, 360.0f);
    g_servoTray->moveToAngle(m_target_angle);
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
void roboter_v23_2_init(int /*loops_per_second*/)
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
    g_servo_D1->calibratePulseMinMax(0.050f, 0.1050f);
    g_servo_D2->calibratePulseMinMax(0.0200f, 0.1310f);

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
void roboter_v23_2_task(DigitalOut& led)
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

    switch (m_state) {

        // ----------------------------------------------------------------
        case STATE_BLIND:
            g_M1->setVelocity(VEL_SIGN * BLIND_SPEED);
            g_M2->setVelocity(VEL_SIGN * BLIND_SPEED);
            if (all_sensors_active()) {
                g_servo_D1->enable(0.0f);   // D1 eingefahren
                g_servo_D2->enable(1.0f);   // D2 oben
                // Drehteller: serviceTray() homed bereits auf 0° sobald Warmup abgeschlossen.
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
        case STATE_CROSSING_STOP:
            g_M1->setVelocity(0.0f);
            g_M2->setVelocity(0.0f);

            // Phase 1: Farbe lesen (erste 0.5 s)
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

            // Phase 2: D1 ausfahren für GRÜN/BLAU
            if (m_crossing_ctr == CROSSING_STOP_LOOPS - COLOR_READ_PHASE) {
                if (m_action_color == 0) m_action_color = m_color_fallback;
                switch (m_action_color) {
                    case 3:  break;
                    case 4:  break;
                    case 5:  g_servo_D1->enable(0.95f); break;
                    case 7:  g_servo_D1->enable(0.95f); break;
                    default: break;
                }
            }

            // D2 absenken + Arm-Sequenz starten
            if (m_action_color == 5) {
                if (m_crossing_ctr == 60) g_servo_D2->enable(m_crossings_left <= 2 ? SERVO_D2_PARTIAL_DOWN : SERVO_D2_GRUEN_DOWN);
                if (m_crossing_ctr == 45) m_arm_retract_ctr = (m_crossings_left <= 2) ? 165 : 108;
            }
            if (m_action_color == 7) {
                if (m_crossing_ctr == 60) g_servo_D2->enable(m_crossings_left <= 2 ? SERVO_D2_PARTIAL_DOWN : SERVO_D2_BLAU_DOWN);
                if (m_crossing_ctr == 45) m_arm_retract_ctr = (m_crossings_left <= 2) ? 165 : 108;
            }

            // Arm-Sequenz (GRÜN / BLAU)
            if ((m_action_color == 5 || m_action_color == 7) && m_arm_retract_ctr > 0) {
                const bool rev = (m_crossings_left <= 2);
                // Balken 2/3/4: D2 voll runter (da zuvor PARTIAL_DOWN war)
                if (rev && m_arm_retract_ctr == 112) {
                    if (m_action_color == 5) g_servo_D2->setPulseWidth(SERVO_D2_GRUEN_DOWN);
                    if (m_action_color == 7) g_servo_D2->setPulseWidth(SERVO_D2_BLAU_DOWN);
                }
                // Jiggle Richtung 1
                if (m_arm_retract_ctr == 108)
                    g_servoTray->moveToAngle(m_target_angle + (rev ? -JIGGLE_DEG : +JIGGLE_DEG));
                if (m_arm_retract_ctr == 101)
                    g_servoTray->moveToAngle(m_target_angle);  // zurück
                // D1/D2 Korrekturen
                if (m_arm_retract_ctr == 82) g_servo_D1->setPulseWidth(0.97f);
                if (m_arm_retract_ctr == 78) g_servo_D1->setPulseWidth(0.95f);
                if (m_arm_retract_ctr == 72) {
                    if (m_action_color == 5) g_servo_D2->setPulseWidth(SERVO_D2_GRUEN_DOWN - 0.02f);
                    if (m_action_color == 7) g_servo_D2->setPulseWidth(SERVO_D2_BLAU_DOWN  - 0.02f);
                }
                if (m_arm_retract_ctr == 68) {
                    if (m_action_color == 5) g_servo_D2->setPulseWidth(SERVO_D2_GRUEN_DOWN);
                    if (m_action_color == 7) g_servo_D2->setPulseWidth(SERVO_D2_BLAU_DOWN);
                }
                // Jiggle Richtung 2 (Doppelimpuls)
                if (m_arm_retract_ctr == 52)
                    g_servoTray->moveToAngle(m_target_angle + (rev ? +JIGGLE_DEG : -JIGGLE_DEG));
                if (m_arm_retract_ctr == 46)
                    g_servoTray->moveToAngle(m_target_angle);
                if (m_arm_retract_ctr == 43)
                    g_servoTray->moveToAngle(m_target_angle + (rev ? +JIGGLE_DEG : -JIGGLE_DEG));
                if (m_arm_retract_ctr == 37) {
                    g_servoTray->moveToAngle(m_target_angle);
                    g_servo_D2->setPulseWidth(1.0f);  // D2 hoch
                }
                if (m_arm_retract_ctr == 22) g_servo_D1->setPulseWidth(0.0f);
                if (m_arm_retract_ctr == 7)  g_servo_D2->disable();
                m_arm_retract_ctr--;
                if (m_arm_retract_ctr == 0) {
                    g_servo_D1->disable();
                    advanceTray();  // +90° zum nächsten Slot
                }
            }

            m_crossing_ctr--;
            if (m_crossing_ctr <= 0 && m_arm_retract_ctr == 0 &&
                g_servoTray->isAtTarget()) {
                int exit_color     = m_action_color;
                m_slowing          = false;
                m_slow_ctr         = 0;
                m_action_color     = 0;
                m_color_delay_ctr  = COLOR_READ_DELAY;
                m_color_stable_ctr = 0;
                m_color_pending    = 0;
                m_arm_retract_ctr  = 0;
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
            }
            break;

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
        case STATE_SMALL_CROSSING_STOP:
            g_M1->setVelocity(0.0f);
            g_M2->setVelocity(0.0f);

            // Phase 1: Farbe lesen
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

            // Phase 2: D1 ausfahren für GRÜN/BLAU
            if (m_small_crossing_ctr == SMALL_CROSSING_STOP_LOOPS - COLOR_READ_PHASE) {
                if (m_action_color == 0) m_action_color = m_color_fallback;
                switch (m_action_color) {
                    case 3:  break;
                    case 4:  break;
                    case 5:  g_servo_D1->enable(0.95f); break;
                    case 7:  g_servo_D1->enable(0.95f); break;
                    default: break;
                }
            }
            if (m_action_color == 5 && m_small_crossing_ctr == 60)
                g_servo_D2->enable(SERVO_D2_GRUEN_DOWN);
            if (m_action_color == 7 && m_small_crossing_ctr == 60)
                g_servo_D2->enable(SERVO_D2_BLAU_DOWN);
            if ((m_action_color == 5 || m_action_color == 7) &&
                m_small_crossing_ctr == 45)
                m_arm_retract_ctr = 108;

            // Arm-Sequenz (GRÜN / BLAU) mit einfachem Jiggle
            if ((m_action_color == 5 || m_action_color == 7) && m_arm_retract_ctr > 0) {
                if (m_arm_retract_ctr == 108) g_servoTray->moveToAngle(m_target_angle + JIGGLE_DEG);
                if (m_arm_retract_ctr == 92)  g_servoTray->moveToAngle(m_target_angle);
                if (m_arm_retract_ctr == 52)  g_servoTray->moveToAngle(m_target_angle - JIGGLE_DEG);
                if (m_arm_retract_ctr == 44) {
                    g_servoTray->moveToAngle(m_target_angle);
                    g_servo_D2->setPulseWidth(1.0f);  // D2 hoch
                }
                if (m_arm_retract_ctr == 29) g_servo_D1->setPulseWidth(0.0f);
                if (m_arm_retract_ctr == 14) g_servo_D2->disable();
                m_arm_retract_ctr--;
                if (m_arm_retract_ctr == 0) {
                    g_servo_D1->disable();
                    advanceTray();  // +90° zum nächsten Slot
                }
            }

            m_small_crossing_ctr--;
            if (m_small_crossing_ctr <= 0 && m_arm_retract_ctr == 0 &&
                g_servoTray->isAtTarget()) {
                int exit_color     = m_action_color;
                m_slowing          = false;
                m_slow_ctr         = 0;
                m_action_color     = 0;
                m_color_delay_ctr  = COLOR_READ_DELAY;
                m_color_stable_ctr = 0;
                m_color_pending    = 0;
                m_arm_retract_ctr  = 0;
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
            }
            break;

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
                m_arm_retract_ctr = (m_crossings_left <= 2 && !m_rot_gelb_is_small) ? 180 : 138;
                m_state           = STATE_ROT_GELB_PAUSE;
            }
            break;
        }

        // ----------------------------------------------------------------
        // ROT_GELB_PAUSE — Arm-Sequenz (identisch zu v23, 360°-Befehle ersetzt)
        // ----------------------------------------------------------------
        case STATE_ROT_GELB_PAUSE: {
            g_M1->setVelocity(0.0f);
            g_M2->setVelocity(0.0f);

            if (m_arm_retract_ctr > 0) {
                float d2_depth = (m_rot_gelb_color == 4) ? SERVO_D2_BLAU_DOWN : SERVO_D2_GRUEN_DOWN;
                const bool rev = (m_crossings_left <= 2 && !m_rot_gelb_is_small);
                const bool jiggle_rev = rev ^ (m_rot_gelb_color == 3 && !m_rot_gelb_is_small);

                // D1/D2 ausfahren (Timing identisch zu v23)
                if (m_arm_retract_ctr == 180 &&  rev) g_servo_D1->enable(0.95f);
                if (m_arm_retract_ctr == 165 &&  rev) g_servo_D2->enable(SERVO_D2_PARTIAL_DOWN);
                if (m_arm_retract_ctr == 138 && !rev) g_servo_D1->enable(0.95f);
                if (m_arm_retract_ctr == 123 && !rev) g_servo_D2->enable(d2_depth);
                // Balken 2/3/4: D2 voll runter nach PARTIAL_DOWN
                if (rev && m_arm_retract_ctr == 112) g_servo_D2->setPulseWidth(d2_depth);

                // Jiggle Richtung 1
                if (m_arm_retract_ctr == 108)
                    g_servoTray->moveToAngle(m_target_angle + (jiggle_rev ? -JIGGLE_DEG : +JIGGLE_DEG));
                if (m_arm_retract_ctr == 101)
                    g_servoTray->moveToAngle(m_target_angle);

                // D1/D2 Korrekturen (nur breite Balken)
                if (!m_rot_gelb_is_small) {
                    if (m_arm_retract_ctr == 82) g_servo_D1->setPulseWidth(0.97f);
                    if (m_arm_retract_ctr == 78) g_servo_D1->setPulseWidth(0.95f);
                    if (m_arm_retract_ctr == 72) g_servo_D2->setPulseWidth(d2_depth - 0.02f);
                    if (m_arm_retract_ctr == 68) g_servo_D2->setPulseWidth(d2_depth);
                }

                // Jiggle Richtung 2
                if (m_arm_retract_ctr == 52)
                    g_servoTray->moveToAngle(m_target_angle + (jiggle_rev ? +JIGGLE_DEG : -JIGGLE_DEG));
                if (m_arm_retract_ctr == 40) {
                    g_servoTray->moveToAngle(m_target_angle);
                    g_servo_D2->setPulseWidth(1.0f);  // D2 hoch
                }

                // D1 einfahren + disable
                if (m_arm_retract_ctr == 22) g_servo_D1->setPulseWidth(0.0f);
                if (m_arm_retract_ctr == 7)  g_servo_D2->disable();
                m_arm_retract_ctr--;
                if (m_arm_retract_ctr == 0) {
                    g_servo_D1->disable();
                    advanceTray();  // +90° zum nächsten Slot
                }
            }

            // Exit: Arm fertig + Drehteller am Ziel
            if (m_arm_retract_ctr == 0 && g_servoTray->isAtTarget()) {
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
void roboter_v23_2_reset(DigitalOut& led)
{
    *g_en = 0;
    g_M1->setVelocity(0.0f);
    g_M2->setVelocity(0.0f);
    g_cmd_M1 = g_cmd_M2 = 0.0f;
    if (g_servo_D1)  g_servo_D1->disable();
    if (g_servo_D2)  g_servo_D2->disable();
    if (g_servoTray) g_servoTray->stop();
    if (g_np)        g_np->clear();

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
    led = 0;
}

// ---------------------------------------------------------------------------
// Print (~5 Hz)
// ---------------------------------------------------------------------------
void roboter_v23_2_print()
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

#endif // PROTOTYPE_03_V23_2
