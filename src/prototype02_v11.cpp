// CargoSweep — PROTOTYPE_02_V11
#include "test_config.h"

#ifdef PROTOTYPE_02_V11

#include "prototype02_v11.h"
#include "Servo.h"
#include "NeoPixel.h"

// ---------------------------------------------------------------------------
// Motor / geometry parameters (Gear 100:1, KN = 140/12)
// ---------------------------------------------------------------------------
static const float GEAR_RATIO  = 100.0f;
static const float KN          = 140.0f / 12.0f;
static const float VOLTAGE_MAX = 12.0f;
static const float VEL_SIGN    = 1.0f;  // v8: -1.0f

static const float D_WHEEL  = 0.0291f;
static const float B_WHEEL  = 0.1493f;
static const float BAR_DIST = 0.182f;

// ---------------------------------------------------------------------------
// Tunable constants
// ---------------------------------------------------------------------------
static const float KP        = 2.8f;   // v9-start: 2.8f
static const float KP_NL     = 4.55f;  // v9-start: 4.55f
static const float KP_FOLLOW    = 1.1f;   // low gains — prevent backward-wheel spin on FOLLOW entry
static const float KP_NL_FOLLOW = 0.7f;
static const float MAX_SPEED = 1.6f;

// SPEED_SCALE: skaliert zeit-basierte Guards, die WÄHREND Linienfolgen bei MAX_SPEED laufen.
// Baseline MAX_SPEED war 1.0f — bei Erhöhung werden die Guards automatisch kürzer.
// ACHTUNG: Nur auf Loops anwenden, die eine Distanz darstellen (Roboter fährt während des Guards).
// NICHT skalieren: Servo-Zeiten, Standstill-Pausen, Motor-Ramps, Intro-Zeiten mit eigenem Speed.
// Falls Probleme: Suche nach Kommentar "// [SPEED_SCALE]" — dort wird skaliert.
static const float SPEED_SCALE = 1.0f / MAX_SPEED;

static const float BLIND_SPEED    = 1.0f;
static const float STRAIGHT_SPEED = 1.0f;
static const float TURN_SPEED     = 0.5f;
static const float BACKWARD_SPEED  = 1.0f;
static const float APPROACH_SPEED  = 0.5f;  // langsamer → getAvgBit hat Zeit aufzubauen

static const int   STRAIGHT_LOOPS  = 70;   // 1.4 s at 50 Hz
static const int   FOLLOW_1S_LOOPS = static_cast<int>(50 * SPEED_SCALE);   // [SPEED_SCALE] 1 s line-follow after crossing detected @ MAX_SPEED=1.0
static const int   BRAKE_LOOPS     = 12;   // 0.24 s smooth brake to 0
static const int   PAUSE_LOOPS     = 20;   // 0.4 s standstill before backwards
static const int   BACKWARD_LOOPS  = 222;  // 4.44 s backwards drive
static const int   ACCEL_LOOPS          = 12;  // 0.24 s smooth accel at start of backwards
static const int   FOLLOW_ACCEL_LOOPS   = 5;   // 0.1 s micro-ramp for motor protection on line follow
static const int   RESTART_ACCEL_LOOPS  = 50;  // 1.0 s straight drive with ramp, then line follower
static const int   STOP_GUARD      = static_cast<int>(75 * SPEED_SCALE);   // [SPEED_SCALE] 1.5 s guard @ MAX_SPEED=1.0 — prevents immediate retrigger
static const int   SLOWDOWN_LOOPS  = 13;    // 0.26 s ramp from 100% → SLOW_FACTOR when colour detected
static const float SLOW_FACTOR     = 0.2f;  // target speed during approach (20% of current command)
static const int   COLOR_READ_DELAY = 50;   // 1.0 s after restart before colour is polled again (avoids re-triggering on old card)
static const int   COLOR_STABLE_CNT = 3;    // consecutive matching reads during driving to trigger slowdown (60 ms)
static const int   COLOR_READ_PHASE = 25;   // 0.5 s colour reading at standstill before servo fires
static const int   COLOR_STOP_STABLE_CNT = 3; // consecutive matching reads at standstill (sensor stable)

// ---------------------------------------------------------------------------
// Real program constants
// ---------------------------------------------------------------------------
static const int   TOTAL_CROSSINGS      = 4;    // 4 wide angled bars to process
static const int   CROSSING_STOP_LOOPS  = 100;  // 2 s total

static const int   SMALL_FOLLOW_START_GUARD  = static_cast<int>(706 * SPEED_SCALE); // [SPEED_SCALE] 14.12 s @ MAX_SPEED=1.0 — ignoriert b3/b4/b5 nach 4. Querbalken
static const int   SMALL_REENTRY_GUARD       = static_cast<int>(100 * SPEED_SCALE); // [SPEED_SCALE] 2.0 s @ MAX_SPEED=1.0 — skip past colour card after each small stop
static const int   TOTAL_SMALL_CROSSINGS     = 4;   // 4 small lines after wide bars
static const int   SMALL_CROSSING_STOP_LOOPS = 100; // 2 s total

static const int   SERVO_ROT_LOOPS        = 75;  // 1.5 s spin for ROT
static const int   SERVO_GELB_LOOPS       = 50;  // 1.0 s spin for GELB
static const int   SERVO_1S_LOOPS         = 50;  // 1 s for 180° servo extend/retract phase

static const float SENSOR_THRESHOLD  = 0.40f;  // v9-start: 0.5f

// ---------------------------------------------------------------------------
// States
// ---------------------------------------------------------------------------
enum State {
    // --- intro sequence ---
    STATE_BLIND      = 1,
    STATE_STRAIGHT   = 2,
    STATE_TURN_SPOT  = 3,
    STATE_FOLLOW     = 4,  // line follow until crossing
    STATE_FOLLOW_1S  = 5,  // keep following for 1 s after crossing
    STATE_BRAKE      = 6,  // smooth decel to 0
    STATE_PAUSE      = 7,  // 0.4 s standstill
    STATE_BACKWARD         = 8,  // 3.9 s backwards (smooth accel + decel)
    STATE_REAL_START_PAUSE = 14, // 0.4 s standstill before real program
    STATE_REAL_APPROACH    = 15, // smooth straight drive to first bar (no line follower)
    // --- real program: wide bars ---
    STATE_REAL_FOLLOW         = 9,  // main line follower (wide bars)
    STATE_CROSSING_STOP       = 10, // 5 s stop at each wide bar (PLACEHOLDER)
    // --- real program: small lines ---
    STATE_SMALL_FOLLOW        = 12, // line follower after wide bars
    STATE_SMALL_CROSSING_STOP = 13, // 3 s stop at each small line (PLACEHOLDER)
    STATE_FINAL_HALT          = 11  // stop forever after 4th small line
};

// ---------------------------------------------------------------------------
// Hardware objects
// ---------------------------------------------------------------------------
static DCMotor*      g_M1 = nullptr;
static DCMotor*      g_M2 = nullptr;
static DigitalOut*   g_en = nullptr;
static LineFollower* g_lf = nullptr;
static ColorSensor*  g_cs       = nullptr;
static Servo*        g_servo    = nullptr;  // 360° servo — D3 (PB_12)
static Servo*        g_servo_D1 = nullptr;  // 180° servo A — D1 (PC_8)
static Servo*        g_servo_D2 = nullptr;  // 180° servo B — D2 (PC_6)
static NeoPixel*     g_np       = nullptr;  // WS2812B colour-debug LED — PB_2 (D0)
static DigitalIn*    g_endstop  = nullptr;  // Endschalter 360°-Servo — A2 (PC_5)

static State m_state           = STATE_BLIND;
static bool  m_servo360_aligning = false;   // true while 360° servo spins to endstop
static int   m_straight_ctr    = 0;
static int   m_follow_ctr      = 0;
static int   m_brake_ctr       = 0;
static int   m_pause_ctr       = 0;
static int   m_backward_ctr    = 0;
static int   m_guard_ctr       = 0;
static int   m_crossing_ctr        = 0;  // countdown for current wide crossing stop
static int   m_crossings_left      = 0;  // wide crossings still to go
static int   m_small_crossing_ctr  = 0;  // countdown for current small line stop
static int   m_small_crossings_left = 0; // small lines still to go
static int   m_approach_ctr       = 0;  // elapsed loops in REAL_APPROACH (for ramp-up)
static int   m_small_accel_ctr    = 0;  // elapsed loops in SMALL_FOLLOW (for ramp-up)
static int   m_real_accel_ctr     = 0;  // elapsed loops in REAL_FOLLOW (for ramp-up)
static float m_brake_start_M1  = 0.0f;
static float m_brake_start_M2  = 0.0f;
static float g_cmd_M1          = 0.0f;
static float g_cmd_M2          = 0.0f;

static int   m_current_color        = 0;  // gecachte Farbe (für print)
static int   m_prev_color           = 0;  // vorherige Farbe (rising-edge, für LED)
static int   m_led_color            = 0;  // letzte signifikante Farbe (LED-Blinkcode)
static int   m_led_ctr              = 0;  // 0–99, 2-Sekunden-Periode (50 Hz)
static int   m_color_log[8]         = {0, 0, 0, 0, 0, 0, 0, 0};
static int   m_color_log_ctr        = 0;
static int   m_action_color         = 0;    // colour read at bar/line detection moment
static bool  m_slowing              = false; // true while slowing down after colour detected
static int   m_slow_ctr             = 0;     // loops since slowdown started
static int   m_color_delay_ctr      = 0;     // loops remaining before colour polling is re-enabled
static int   m_color_stable_ctr     = 0;     // consecutive matching reads of the same action colour
static int   m_color_pending        = 0;     // the action colour currently being confirmed

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static bool all_sensors_active()
{
    for (int i = 0; i < 8; i++) {
        if (g_lf->getAvgBit(i) < SENSOR_THRESHOLD)
            return false;
    }
    return true;
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
        if (g_lf->getAvgBit(i) >= SENSOR_THRESHOLD)
            count++;
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

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void roboter_v11_init(int loops_per_second)
{
    static DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1,
                             GEAR_RATIO, KN, VOLTAGE_MAX);
    static DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2,
                             GEAR_RATIO, KN, VOLTAGE_MAX);
    static DigitalOut enable_motors(PB_ENABLE_DCMOTORS);
    static LineFollower lineFollower(PB_9, PB_8, BAR_DIST, D_WHEEL, B_WHEEL,
                                     motor_M1.getMaxPhysicalVelocity());
    static ColorSensor colorSensor(PB_3, PC_3, PA_4, PB_0, PC_1, PC_0);
    static Servo servo_360(PB_12);
    static Servo servo_D1(PC_8);
    static Servo servo_D2(PC_6);
    static NeoPixel neopixel(PB_2);
    static DigitalIn endstop(PC_5, PullUp);

    g_M1       = &motor_M1;
    g_M2       = &motor_M2;
    g_en       = &enable_motors;
    g_lf       = &lineFollower;
    g_cs       = &colorSensor;
    g_servo    = &servo_360;
    g_servo_D1 = &servo_D1;
    g_servo_D2 = &servo_D2;
    g_np       = &neopixel;
    g_endstop  = &endstop;
    g_np->clear();
    g_cs->setCalibration();
    g_cs->switchLed(ON);
    g_servo->calibratePulseMinMax(0.050f, 0.1050f);
    g_servo_D1->calibratePulseMinMax(0.050f, 0.1050f);
    g_servo_D2->calibratePulseMinMax(0.050f, 0.1050f);

    g_lf->setRotationalVelocityControllerGains(KP, KP_NL);
    g_lf->setMaxWheelVelocity(MAX_SPEED);

    TIM1->BDTR |= TIM_BDTR_MOE;

    *g_en            = 0;
    m_state          = STATE_BLIND;
    m_straight_ctr   = 0;
    m_follow_ctr     = 0;
    m_brake_ctr      = 0;
    m_pause_ctr      = 0;
    m_backward_ctr   = 0;
    m_guard_ctr      = 0;
    m_crossing_ctr         = 0;
    m_crossings_left       = 0;
    m_small_crossing_ctr   = 0;
    m_small_crossings_left = 0;
    m_approach_ctr         = 0;
    m_small_accel_ctr      = 0;
    m_real_accel_ctr       = 0;
    m_brake_start_M1       = 0.0f;
    m_brake_start_M2       = 0.0f;
    m_current_color        = 0;
    m_prev_color           = 0;
    m_led_color            = 0;
    m_led_ctr              = 0;
    m_color_log_ctr        = 0;
    m_action_color         = 0;
    m_slowing              = false;
    m_slow_ctr             = 0;
    m_color_delay_ctr      = 0;
    m_color_stable_ctr     = 0;
    m_color_pending        = 0;
    for (int i = 0; i < 8; i++) m_color_log[i] = 0;
    g_M1->setVelocity(0.0f);
    g_M2->setVelocity(0.0f);
}

void roboter_v11_task(DigitalOut& led)
{
    // --- Farbsensor ---
    m_current_color  = g_cs->getColor();

    // --- NeoPixel Live-Spiegelung (gedimmt 60/255) — nur Aktions-Farben 3/4/5/7 ---
    {
        uint8_t pr = 0, pg = 0, pb = 0;
        switch (m_current_color) {
            case 3: pr = 60; pg =  0; pb =  0; break; // ROT
            case 4: pr = 60; pg = 25; pb =  0; break; // GELB
            case 5: pr =  0; pg = 60; pb =  0; break; // GRÜN
            case 7: pr =  0; pg =  0; pb = 60; break; // BLAU
            default: break;                            // AUS (UNKNOWN/BLACK/WHITE/CYAN/MAGENTA)
        }
        g_np->setRGB(pr, pg, pb);
        g_np->show();
    }

    bool is_sig      = (m_current_color == 3 || m_current_color == 4 ||
                        m_current_color == 5 || m_current_color == 7);
    bool was_neutral = (m_prev_color == 0 || m_prev_color == 1 || m_prev_color == 2);
    if (is_sig && was_neutral) {
        if (m_color_log_ctr < 8)
            m_color_log[m_color_log_ctr++] = m_current_color;
        m_led_color = m_current_color;
    }
    m_prev_color = m_current_color;

    // --- LED Blinkcode (2-Sekunden-Periode = 100 Loops bei 50 Hz) ---
    // Kein Signal: schnelles Blinken (alive). ROT=1 GELB=2 GRÜN=3 BLAU=4 Blinks/2s
    m_led_ctr = (m_led_ctr + 1) % 100;
    if (m_led_color == 0) {
        led = !led;
    } else {
        int n   = (m_led_color == 3) ? 1 :
                  (m_led_color == 4) ? 2 :
                  (m_led_color == 5) ? 3 :
                  (m_led_color == 7) ? 4 : 0;
        int pos = m_led_ctr;
        led     = (n > 0 && pos < n * 16 && (pos % 16) < 8) ? 1 : 0;
    }

    *g_en = 1;

    if (m_servo360_aligning && g_endstop->read() == 0) {
        g_servo->disable();
        m_servo360_aligning = false;
    }

    switch (m_state) {

        // ----------------------------------------------------------------
        // BLIND: drive straight until all 8 sensors see the sideline
        // ----------------------------------------------------------------
        case STATE_BLIND:
            g_M1->setVelocity(VEL_SIGN * BLIND_SPEED);
            g_M2->setVelocity(VEL_SIGN * BLIND_SPEED);
            if (all_sensors_active()) {
                g_servo_D1->enable(0.0f);   // D1 komplett eingefahren
                g_servo_D2->enable(1.0f);   // D2 komplett oben
                g_servo->enable(0.80f);     // 360° langsam drehen bis Endschalter
                m_servo360_aligning = true;
                m_straight_ctr = STRAIGHT_LOOPS;
                m_state        = STATE_STRAIGHT;
            }
            break;

        // ----------------------------------------------------------------
        // STRAIGHT: drive forward 1.7 s past the sideline
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
        // TURN_SPOT: pivot until center sensors pick up the line
        // ----------------------------------------------------------------
        case STATE_TURN_SPOT:
            g_M1->setVelocity(VEL_SIGN * -TURN_SPEED);
            g_M2->setVelocity(VEL_SIGN *  TURN_SPEED);
            if (center_sensors_active()) {
                g_lf->setRotationalVelocityControllerGains(KP_FOLLOW, KP_NL_FOLLOW);
                m_guard_ctr = STOP_GUARD;
                m_state     = STATE_FOLLOW;
            }
            break;

        // ----------------------------------------------------------------
        // FOLLOW: line follower until all 8 sensors active
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
        // FOLLOW_1S: keep line-following for 1 s after crossing detected
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
        // BRAKE: linear ramp from captured velocity to 0 over 0.24 s
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
        // PAUSE: stand still for 0.4 s
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
        // BACKWARD: 3.9 s — smooth accel (0.24 s), full speed, smooth decel (0.24 s)
        // ----------------------------------------------------------------
        case STATE_BACKWARD: {
            int   elapsed    = BACKWARD_LOOPS - m_backward_ctr;
            float ramp_up    = (elapsed < ACCEL_LOOPS)
                                   ? (static_cast<float>(elapsed) / static_cast<float>(ACCEL_LOOPS))
                                   : 1.0f;
            float ramp_down  = (m_backward_ctr < BRAKE_LOOPS)
                                   ? (static_cast<float>(m_backward_ctr) / static_cast<float>(BRAKE_LOOPS))
                                   : 1.0f;
            float ramp = (ramp_up < ramp_down) ? ramp_up : ramp_down;
            float spd = -VEL_SIGN * BACKWARD_SPEED * ramp;
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

        // ----------------------------------------------------------------
        // REAL_START_PAUSE: 0.4 s standstill after backwards before real program
        // ----------------------------------------------------------------
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
        // REAL_APPROACH: smooth straight drive to first wide bar (no line follower)
        // ----------------------------------------------------------------
        case STATE_REAL_APPROACH: {
            float ramp = (m_approach_ctr < ACCEL_LOOPS)
                             ? (static_cast<float>(m_approach_ctr) / static_cast<float>(ACCEL_LOOPS))
                             : 1.0f;
            float spd  = VEL_SIGN * APPROACH_SPEED * ramp;
            g_M1->setVelocity(spd);
            g_M2->setVelocity(spd);
            m_approach_ctr++;

            if (m_approach_ctr > ACCEL_LOOPS && wide_bar_active()) {
                g_M1->setVelocity(0.0f);
                g_M2->setVelocity(0.0f);
                m_crossings_left--;
                m_crossing_ctr     = CROSSING_STOP_LOOPS;
                m_action_color     = 0;
                m_color_stable_ctr = 0;
                m_color_pending    = 0;
                m_state            = STATE_CROSSING_STOP;
            }
            break;
        }

        // ----------------------------------------------------------------
        // REAL_FOLLOW: main line follower
        // Stops at each of the 4 angled bars
        // ----------------------------------------------------------------
        case STATE_REAL_FOLLOW: {
            g_lf->setMaxWheelVelocity(MAX_SPEED);
            g_lf->setRotationalVelocityControllerGains(KP, KP_NL);

            if (m_real_accel_ctr < RESTART_ACCEL_LOOPS) {
                float ramp = static_cast<float>(m_real_accel_ctr + 1) / static_cast<float>(RESTART_ACCEL_LOOPS);
                float spd = VEL_SIGN * MAX_SPEED * ramp;
                g_cmd_M1 = spd;
                g_cmd_M2 = spd;
                m_real_accel_ctr++;
            } else {
                if (m_color_delay_ctr > 0) {
                    m_color_delay_ctr--;
                    m_color_stable_ctr = 0;
                    m_color_pending    = 0;
                } else if (!m_slowing) {
                    int col_now = g_cs->getColor();
                    bool is_action = (col_now == 3 || col_now == 4 || col_now == 5 || col_now == 7);
                    if (is_action && col_now == m_color_pending) {
                        m_color_stable_ctr++;
                    } else if (is_action) {
                        m_color_pending    = col_now;
                        m_color_stable_ctr = 1;
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
                g_cmd_M1 = VEL_SIGN * g_lf->getLeftWheelVelocity() * slow_ramp;
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
                m_state            = STATE_CROSSING_STOP;
            }
            break;
        }

        // ----------------------------------------------------------------
        // CROSSING_STOP: stop → servo (colour already read at bar detection)
        // *** PLACEHOLDER — swap out CROSSING_STOP_LOOPS and this state's
        //     body later for arm extension, rotation, etc. ***
        // After the 4th wide bar → continue to small lines, else resume
        // ----------------------------------------------------------------
        case STATE_CROSSING_STOP:
            g_M1->setVelocity(0.0f);
            g_M2->setVelocity(0.0f);

            // Phase 1: Farbe lesen im Stillstand (erste 0.5 s = COLOR_READ_PHASE Loops)
            if (m_crossing_ctr > CROSSING_STOP_LOOPS - COLOR_READ_PHASE) {
                int col_now = g_cs->getColor();
                bool is_action = (col_now == 3 || col_now == 4 || col_now == 5 || col_now == 7);
                if (is_action && col_now == m_color_pending) {
                    m_color_stable_ctr++;
                } else if (is_action) {
                    m_color_pending    = col_now;
                    m_color_stable_ctr = 1;
                }
                if (m_color_stable_ctr >= COLOR_STOP_STABLE_CNT && m_action_color == 0) {
                    m_action_color = m_color_pending;
                }
            }

            // Phase 2: Servo feuern nach Lesephase
            if (m_crossing_ctr == CROSSING_STOP_LOOPS - COLOR_READ_PHASE) {
                switch (m_action_color) {
                    case 3:  g_servo->enable(0.90f);    break; // ROT  — D3 schnell
                    case 4:  g_servo->enable(0.80f);    break; // GELB — D3 langsam
                    case 5:  g_servo_D1->enable(1.0f);  break; // GRÜN — D1 ausfahren
                    case 7:  g_servo_D1->enable(0.2f);  break; // BLAU — D1 20% ausfahren (D1: 0.0=ein, 1.0=aus)
                    default: break;                             // kein Signal → kein Servo
                }
            }
            // Stop ROT servo after SERVO_ROT_LOOPS, GELB after SERVO_GELB_LOOPS (ab Servo-Start)
            if (m_crossing_ctr == CROSSING_STOP_LOOPS - COLOR_READ_PHASE - SERVO_GELB_LOOPS && m_action_color == 4) g_servo->disable();
            // Retract 180° servo after SERVO_1S_LOOPS (ab Servo-Start)
            if (m_crossing_ctr == CROSSING_STOP_LOOPS - COLOR_READ_PHASE - SERVO_1S_LOOPS) {
                if (m_action_color == 5) g_servo_D1->setPulseWidth(0.0f);
            }
            // BLAU Sequenz: ausfahren → senken → hoch → einfahren → 360° drehen
            if (m_action_color == 7) {
                if (m_crossing_ctr == 60) g_servo_D2->enable(0.8f);          // D2 senken 20% (D2: 1.0=oben, 0.0=unten)
                if (m_crossing_ctr == 45) g_servo_D2->setPulseWidth(1.0f);   // D2 komplett hoch
                if (m_crossing_ctr == 35) {
                    g_servo_D2->disable();
                    g_servo_D1->setPulseWidth(0.0f);                          // D1 komplett einfahren
                }
                if (m_crossing_ctr == 25) {
                    g_servo_D1->disable();
                    g_servo->enable(0.80f);                                   // 360° langsam
                }
                if (m_crossing_ctr == 15) g_servo->disable();                // 360° stopp
            }
            if (m_crossing_ctr == 1) {
                if (m_action_color == 3) g_servo->disable();
                if (m_action_color == 5) g_servo_D1->disable();
            }

            m_crossing_ctr--;
            if (m_crossing_ctr <= 0) {
                m_slowing          = false;
                m_slow_ctr         = 0;
                m_action_color     = 0;
                m_color_delay_ctr  = COLOR_READ_DELAY;
                m_color_stable_ctr = 0;
                m_color_pending    = 0;
                if (m_crossings_left == 0) {
                    // all 4 wide bars done → move on to small lines
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

        // ----------------------------------------------------------------
        // SMALL_FOLLOW: line follower between the 4 small lines
        // Triggered when sensors 2–5 all see the line
        // ----------------------------------------------------------------
        case STATE_SMALL_FOLLOW: {
            g_lf->setMaxWheelVelocity(MAX_SPEED);
            g_lf->setRotationalVelocityControllerGains(KP, KP_NL);

            if (m_small_accel_ctr < RESTART_ACCEL_LOOPS) {
                float ramp = static_cast<float>(m_small_accel_ctr + 1) / static_cast<float>(RESTART_ACCEL_LOOPS);
                float spd = VEL_SIGN * MAX_SPEED * ramp;
                g_cmd_M1 = spd;
                g_cmd_M2 = spd;
                m_small_accel_ctr++;
            } else {
                if (m_guard_ctr > 0 || m_color_delay_ctr > 0) {
                    if (m_color_delay_ctr > 0) m_color_delay_ctr--;
                    m_color_stable_ctr = 0;
                    m_color_pending    = 0;
                } else if (!m_slowing) {
                    int col_now = g_cs->getColor();
                    bool is_action = (col_now == 3 || col_now == 4 || col_now == 5 || col_now == 7);
                    if (is_action && col_now == m_color_pending) {
                        m_color_stable_ctr++;
                    } else if (is_action) {
                        m_color_pending    = col_now;
                        m_color_stable_ctr = 1;
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
                g_cmd_M1 = VEL_SIGN * g_lf->getLeftWheelVelocity() * slow_ramp;
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
                m_action_color       = 0;
                m_color_stable_ctr   = 0;
                m_color_pending      = 0;
                m_state              = STATE_SMALL_CROSSING_STOP;
            }
            break;
        }

        // ----------------------------------------------------------------
        // SMALL_CROSSING_STOP: stop → read colour (1.5 s) → servo → wait
        // After the 4th small line → FINAL_HALT, else resume following
        // ----------------------------------------------------------------
        case STATE_SMALL_CROSSING_STOP:
            g_M1->setVelocity(0.0f);
            g_M2->setVelocity(0.0f);

            // Phase 1: Farbe lesen im Stillstand (erste 0.5 s = COLOR_READ_PHASE Loops)
            if (m_small_crossing_ctr > SMALL_CROSSING_STOP_LOOPS - COLOR_READ_PHASE) {
                int col_now = g_cs->getColor();
                bool is_action = (col_now == 3 || col_now == 4 || col_now == 5 || col_now == 7);
                if (is_action && col_now == m_color_pending) {
                    m_color_stable_ctr++;
                } else if (is_action) {
                    m_color_pending    = col_now;
                    m_color_stable_ctr = 1;
                }
                if (m_color_stable_ctr >= COLOR_STOP_STABLE_CNT && m_action_color == 0) {
                    m_action_color = m_color_pending;
                }
            }

            // Phase 2: Servo feuern nach Lesephase
            if (m_small_crossing_ctr == SMALL_CROSSING_STOP_LOOPS - COLOR_READ_PHASE) {
                switch (m_action_color) {
                    case 3:  g_servo->enable(0.90f);    break; // ROT  — D3 schnell
                    case 4:  g_servo->enable(0.80f);    break; // GELB — D3 langsam
                    case 5:  g_servo_D1->enable(1.0f);  break; // GRÜN — D1 ausfahren
                    case 7:  g_servo_D1->enable(0.2f);  break; // BLAU — D1 20% ausfahren (D1: 0.0=ein, 1.0=aus)
                    default: break;                             // kein Signal → kein Servo
                }
            }
            // Stop ROT servo after SERVO_ROT_LOOPS, GELB after SERVO_GELB_LOOPS (ab Servo-Start)
            if (m_small_crossing_ctr == SMALL_CROSSING_STOP_LOOPS - COLOR_READ_PHASE - SERVO_GELB_LOOPS && m_action_color == 4) g_servo->disable();
            // Retract 180° servo after SERVO_1S_LOOPS (ab Servo-Start)
            if (m_small_crossing_ctr == SMALL_CROSSING_STOP_LOOPS - COLOR_READ_PHASE - SERVO_1S_LOOPS) {
                if (m_action_color == 5) g_servo_D1->setPulseWidth(0.0f);
            }
            // BLAU Sequenz: ausfahren → senken → hoch → einfahren → 360° drehen
            if (m_action_color == 7) {
                if (m_small_crossing_ctr == 60) g_servo_D2->enable(0.8f);          // D2 senken 20% (D2: 1.0=oben, 0.0=unten)
                if (m_small_crossing_ctr == 45) g_servo_D2->setPulseWidth(1.0f);   // D2 komplett hoch
                if (m_small_crossing_ctr == 35) {
                    g_servo_D2->disable();
                    g_servo_D1->setPulseWidth(0.0f);                                // D1 komplett einfahren
                }
                if (m_small_crossing_ctr == 25) {
                    g_servo_D1->disable();
                    g_servo->enable(0.80f);                                         // 360° langsam
                }
                if (m_small_crossing_ctr == 15) g_servo->disable();                // 360° stopp
            }
            if (m_small_crossing_ctr == 1) {
                if (m_action_color == 3) g_servo->disable();
                if (m_action_color == 5) g_servo_D1->disable();
            }

            m_small_crossing_ctr--;
            if (m_small_crossing_ctr <= 0) {
                m_slowing          = false;
                m_slow_ctr         = 0;
                m_action_color     = 0;
                m_color_delay_ctr  = COLOR_READ_DELAY;
                m_color_stable_ctr = 0;
                m_color_pending    = 0;
                if (m_small_crossings_left == 0) {
                    m_state = STATE_FINAL_HALT;
                } else {
                    m_guard_ctr       = SMALL_REENTRY_GUARD;
                    m_small_accel_ctr = 0;
                    m_state           = STATE_SMALL_FOLLOW;
                }
            }
            break;

        // ----------------------------------------------------------------
        // FINAL_HALT: stop forever
        // ----------------------------------------------------------------
        case STATE_FINAL_HALT:
            g_M1->setVelocity(0.0f);
            g_M2->setVelocity(0.0f);
            break;
    }

}

void roboter_v11_reset(DigitalOut& led)
{
    *g_en            = 0;
    g_M1->setVelocity(0.0f);
    g_M2->setVelocity(0.0f);
    g_cmd_M1             = 0.0f;
    g_cmd_M2             = 0.0f;
    m_state              = STATE_BLIND;
    m_servo360_aligning  = false;
    m_straight_ctr       = 0;
    m_follow_ctr         = 0;
    m_brake_ctr      = 0;
    m_pause_ctr      = 0;
    m_backward_ctr   = 0;
    m_guard_ctr      = 0;
    m_crossing_ctr         = 0;
    m_crossings_left       = 0;
    m_small_crossing_ctr   = 0;
    m_small_crossings_left = 0;
    m_approach_ctr         = 0;
    m_small_accel_ctr      = 0;
    m_real_accel_ctr       = 0;
    m_brake_start_M1       = 0.0f;
    m_brake_start_M2       = 0.0f;
    m_current_color        = 0;
    m_prev_color           = 0;
    m_led_color            = 0;
    m_led_ctr              = 0;
    m_color_log_ctr        = 0;
    m_action_color         = 0;
    m_slowing              = false;
    m_slow_ctr             = 0;
    m_color_delay_ctr      = 0;
    m_color_stable_ctr     = 0;
    m_color_pending        = 0;
    for (int i = 0; i < 8; i++) m_color_log[i] = 0;
    g_servo->disable();
    g_servo_D1->disable();
    g_servo_D2->disable();
    if (g_np) g_np->clear();
    led                    = 0;
}

void roboter_v11_print()
{
    const char* s = (m_state == STATE_BLIND)               ? "BLIND       " :
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
                    (m_state == STATE_SMALL_CROSSING_STOP) ? "SMALL_STOP  " : "FINAL_HALT  ";
    printf("State: %s | wide=%d small=%d | a=%.3f | M1=%+.2f M2=%+.2f | all8=%d s2345=%d\n",
           s,
           m_crossings_left,
           m_small_crossings_left,
           g_lf->getAngleRadians(),
           g_M1->getVelocity(),
           g_M2->getVelocity(),
           all_sensors_active() ? 1 : 0,
           small_line_active() ? 1 : 0);

    // Compute hue from normalised RGB for threshold tuning
    const float* norm = g_cs->readColorNorm();
    float rn = norm[0], gn = norm[1], bn = norm[2];
    float cmax = fmaxf(rn, fmaxf(gn, bn));
    float cmin = fminf(rn, fminf(gn, bn));
    float delta = cmax - cmin;
    float hue = 0.0f;
    if (delta > 1e-6f) {
        if (cmax == rn)      hue = 60.0f * fmodf((gn - bn) / delta, 6.0f);
        else if (cmax == gn) hue = 60.0f * ((bn - rn) / delta + 2.0f);
        else                 hue = 60.0f * ((rn - gn) / delta + 4.0f);
        if (hue < 0.0f) hue += 360.0f;
    }

    const float* cal = g_cs->readColorCalib();
    float r0 = cal[0], g0 = cal[1], b0 = cal[2];
    printf("Color: %-8s hue=%5.1f delta=%.2f r=%.2f g=%.2f b=%.2f | action=%s\n",
           ColorSensor::getColorString(m_current_color),
           hue, delta, r0, g0, b0,
           ColorSensor::getColorString(m_action_color));
}

#endif // PROTOTYPE_02_V11
