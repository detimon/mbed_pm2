#include "test_config.h"

#ifdef TEST_ROBOTER_V2

#include "roboter_v2.h"

// ---------------------------------------------------------------------------
// Motor / geometry parameters (Gear 100:1, KN = 140/12)
// ---------------------------------------------------------------------------
static const float GEAR_RATIO  = 100.0f;
static const float KN          = 140.0f / 12.0f;
static const float VOLTAGE_MAX = 12.0f;
static const float VEL_SIGN    = -1.0f;

static const float D_WHEEL  = 0.0393f;
static const float B_WHEEL  = 0.179f;
static const float BAR_DIST = 0.1836f;

// ---------------------------------------------------------------------------
// Tunable constants
// ---------------------------------------------------------------------------
static const float KP        = 2.8f;
static const float KP_NL     = 4.55f;
static const float MAX_SPEED = 1.0f;

static const float BLIND_SPEED    = 1.0f;  // speed during blind drive to sideline
static const float STRAIGHT_SPEED = 1.0f;  // speed during 1.7 s straight after sideline
static const float TURN_SPEED     = 0.5f;  // pivot speed for in-place 90° turn

static const int   STRAIGHT_LOOPS = 85;    // 1.7 s at 50 Hz
static const int   STOP_5S_LOOPS  = 250;   // 5 s stop at 50 Hz
static const int   STOP_GUARD     = 75;    // 1.5 s guard after stop before retrigger allowed

// How many stops total:
//   stop 1 (m_stop_remaining==4): isLedActive() — handles angled first crossing
//   stops 2-4 (m_stop_remaining==3,2,1): all_sensors_active() — normal crossings
//   after that (m_stop_remaining==0): follow forever, no more stops
static const int   TOTAL_STOPS    = 4;     // 1 first (any sensor) + 3 normal (all sensors)

static const float SENSOR_THRESHOLD = 0.5f;

// ---------------------------------------------------------------------------
// States
// ---------------------------------------------------------------------------
enum State {
    STATE_BLIND      = 1, // drive straight until all 8 sensors see the sideline
    STATE_STRAIGHT   = 2, // drive straight for 1.7 s after sideline
    STATE_TURN_SPOT  = 3, // pivot in-place 90° until line is re-acquired
    STATE_FOLLOW     = 4, // pure line follower (same as LINE_FOLLOWER_FAST)
    STATE_STOP_5S    = 5  // 5 s stop on crossing, then back to FOLLOW
};

// ---------------------------------------------------------------------------
// Hardware objects
// ---------------------------------------------------------------------------
static DCMotor*      g_M1 = nullptr;
static DCMotor*      g_M2 = nullptr;
static DigitalOut*   g_en = nullptr;
static LineFollower* g_lf = nullptr;

static State m_state          = STATE_BLIND;
static int   m_straight_ctr   = 0;
static int   m_stop_ctr       = 0;
static int   m_guard_ctr      = 0;
static int   m_stop_remaining = 0; // counts down from TOTAL_STOPS to 0
static float g_cmd_M1         = 0.0f;
static float g_cmd_M2         = 0.0f;

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

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void roboter_v2_init(int loops_per_second)
{
    static DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1,
                             GEAR_RATIO, KN, VOLTAGE_MAX);
    static DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2,
                             GEAR_RATIO, KN, VOLTAGE_MAX);
    static DigitalOut enable_motors(PB_ENABLE_DCMOTORS);
    static LineFollower lineFollower(PB_9, PB_8, BAR_DIST, D_WHEEL, B_WHEEL,
                                     motor_M1.getMaxPhysicalVelocity());

    g_M1 = &motor_M1;
    g_M2 = &motor_M2;
    g_en = &enable_motors;
    g_lf = &lineFollower;

    g_lf->setRotationalVelocityControllerGains(KP, KP_NL);
    g_lf->setMaxWheelVelocity(MAX_SPEED);

    TIM1->BDTR |= TIM_BDTR_MOE;

    *g_en            = 0;
    m_state          = STATE_BLIND;
    m_straight_ctr   = 0;
    m_stop_ctr       = 0;
    m_guard_ctr      = 0;
    m_stop_remaining = 0;
    g_M1->setVelocity(0.0f);
    g_M2->setVelocity(0.0f);
}

void roboter_v2_task(DigitalOut& led)
{
    led = !led;
    *g_en = 1;

    switch (m_state) {

        // ----------------------------------------------------------------
        // BLIND: drive straight until the sideline lights up all 8 sensors
        // ----------------------------------------------------------------
        case STATE_BLIND:
            g_M1->setVelocity(VEL_SIGN * BLIND_SPEED);
            g_M2->setVelocity(VEL_SIGN * BLIND_SPEED);
            if (all_sensors_active()) {
                m_straight_ctr = STRAIGHT_LOOPS;
                m_state        = STATE_STRAIGHT;
            }
            break;

        // ----------------------------------------------------------------
        // STRAIGHT: drive forward for 1.7 s past the sideline
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
        // TURN_SPOT: pivot in-place (M1 forward, M2 backward) until the
        //            line sensor picks up the line again
        // ----------------------------------------------------------------
        case STATE_TURN_SPOT:
            g_M1->setVelocity(VEL_SIGN *  TURN_SPEED);
            g_M2->setVelocity(VEL_SIGN * -TURN_SPEED);
            if (center_sensors_active()) {
                // entering FOLLOW: arm all stops, guard so the turn-spot
                // line hit doesn't immediately retrigger stop
                m_stop_remaining = TOTAL_STOPS;
                m_guard_ctr      = STOP_GUARD;
                m_state          = STATE_FOLLOW;
            }
            break;

        // ----------------------------------------------------------------
        // FOLLOW: pure line follower — identical to LINE_FOLLOWER_FAST
        //
        // Stop logic:
        //   m_stop_remaining == TOTAL_STOPS (first crossing, may be angled):
        //       trigger on isLedActive() — any sensor sees the bar
        //   m_stop_remaining in [1, TOTAL_STOPS-1] (normal crossings):
        //       trigger on all_sensors_active()
        //   m_stop_remaining == 0:
        //       never stop, follow forever
        // ----------------------------------------------------------------
        case STATE_FOLLOW:
            g_cmd_M1 = VEL_SIGN * g_lf->getRightWheelVelocity();
            g_cmd_M2 = VEL_SIGN * g_lf->getLeftWheelVelocity();
            g_M1->setVelocity(g_cmd_M1);
            g_M2->setVelocity(g_cmd_M2);

            if (m_guard_ctr > 0) {
                m_guard_ctr--;
            } else if (m_stop_remaining > 0) {
                bool triggered = (m_stop_remaining == TOTAL_STOPS)
                                     ? g_lf->isLedActive()   // first: any sensor
                                     : all_sensors_active();  // others: all 8
                if (triggered) {
                    g_M1->setVelocity(0.0f);
                    g_M2->setVelocity(0.0f);
                    m_stop_ctr = STOP_5S_LOOPS;
                    m_state    = STATE_STOP_5S;
                }
            }
            break;

        // ----------------------------------------------------------------
        // STOP_5S: hold still for 5 s, then resume line following
        // ----------------------------------------------------------------
        case STATE_STOP_5S:
            g_M1->setVelocity(0.0f);
            g_M2->setVelocity(0.0f);
            m_stop_ctr--;
            if (m_stop_ctr <= 0) {
                m_stop_remaining--;
                m_guard_ctr = STOP_GUARD;
                m_state     = STATE_FOLLOW;
            }
            break;
    }
}

void roboter_v2_reset(DigitalOut& led)
{
    *g_en            = 0;
    g_M1->setVelocity(0.0f);
    g_M2->setVelocity(0.0f);
    g_cmd_M1         = 0.0f;
    g_cmd_M2         = 0.0f;
    m_state          = STATE_BLIND;
    m_straight_ctr   = 0;
    m_stop_ctr       = 0;
    m_guard_ctr      = 0;
    m_stop_remaining = 0;
    led              = 0;
}

void roboter_v2_print()
{
    const char* s = (m_state == STATE_BLIND)     ? "BLIND    " :
                    (m_state == STATE_STRAIGHT)  ? "STRAIGHT " :
                    (m_state == STATE_TURN_SPOT) ? "TURN_SPOT" :
                    (m_state == STATE_STOP_5S)   ? "STOP_5S  " : "FOLLOW   ";
    printf("State: %s | stops_left=%d | a=%.3f | M1=%+.2f M2=%+.2f | led=%d\n",
           s,
           m_stop_remaining,
           g_lf->getAngleRadians(),
           g_M1->getVelocity(),
           g_M2->getVelocity(),
           g_lf->isLedActive() ? 1 : 0);
}

#endif // TEST_ROBOTER_V2
