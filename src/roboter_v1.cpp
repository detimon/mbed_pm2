#include "test_config.h"

#ifdef TEST_ROBOTER_V1

#include "roboter_v1.h"

// ---------------------------------------------------------------------------
// Motor / geometry parameters (Gear 100:1, KN = 140/12)
// ---------------------------------------------------------------------------
static const float GEAR_RATIO  = 100.0f;
static const float KN          = 140.0f / 12.0f;
static const float VOLTAGE_MAX = 12.0f;
static const float VEL_SIGN    = -1.0f; // both motors mounted inverted

static const float D_WHEEL  = 0.0393f;
static const float B_WHEEL  = 0.179f;
static const float BAR_DIST = 0.1836f;

// ---------------------------------------------------------------------------
// Tunable constants
// ---------------------------------------------------------------------------
static const float KP           = 4.0f;
static const float KP_NL        = 15.0f;
static const float FOLLOW_SPEED = 0.8f;  // RPS — speed cap for LineFollower

static const float BLIND_SPEED     = 1.0f;  // RPS
static const float REACQUIRE_SPEED = 0.4f;  // RPS

static const float TURN_SPEED = 1.0f;
static const float TURN_INNER = 0.3f;
static const int   TURN_LOOPS = 60;

// Straight detection — entered from STATE_FOLLOW only
static const float STRAIGHT_SPEED       = 1.5f; // RPS on straight
// How long to stay in FOLLOW before we start looking for a straight (covers
// the right curve right after the turn). 75 loops = 1.5 s at 50 Hz.
static const int   FOLLOW_MIN_LOOPS     = 75;
// Angle must stay below this for STRAIGHT_ENTER_LOOPS to confirm straight
static const float STRAIGHT_ENTER_ANGLE = 0.20f; // rad — tune up if never triggers
static const int   STRAIGHT_ENTER_LOOPS = 8;     // consecutive loops below enter angle
// Angle must rise above this to leave the straight
static const float STRAIGHT_EXIT_ANGLE  = 0.30f; // rad — tune down if exits too late

static const float SENSOR_THRESHOLD = 0.5f;

// ---------------------------------------------------------------------------
// States
// ---------------------------------------------------------------------------
enum State {
    STATE_BLIND     = 1,
    STATE_TURNING   = 2,
    STATE_REACQUIRE = 3,
    STATE_FOLLOW    = 4, // LineFollower — handles all curves
    STATE_STRAIGHT  = 5  // both wheels equal — no correction on straight
};

// ---------------------------------------------------------------------------
// Hardware objects
// ---------------------------------------------------------------------------
static DCMotor*      g_M1 = nullptr;
static DCMotor*      g_M2 = nullptr;
static DigitalOut*   g_en = nullptr;
static LineFollower* g_lf = nullptr;

static int   m_lps              = 50;
static State m_state            = STATE_BLIND;
static int   m_turn_ctr         = 0;
static int   m_follow_ctr       = 0; // loops spent in STATE_FOLLOW this stint
static int   m_straight_ctr     = 0; // consecutive loops with small angle
static float g_cmd_M1           = 0.0f;
static float g_cmd_M2           = 0.0f;

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
    return g_lf->getAvgBit(3) >= SENSOR_THRESHOLD &&
           g_lf->getAvgBit(4) >= SENSOR_THRESHOLD;
}

static void drive(float right_rps, float left_rps)
{
    g_M1->setVelocity(VEL_SIGN * right_rps);
    g_M2->setVelocity(VEL_SIGN * left_rps);
    g_cmd_M1 = right_rps;
    g_cmd_M2 = left_rps;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void roboter_v1_init(int loops_per_second)
{
    m_lps = loops_per_second;

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
    g_lf->setMaxWheelVelocity(FOLLOW_SPEED);

    TIM1->BDTR |= TIM_BDTR_MOE;

    *g_en         = 0;
    m_state       = STATE_BLIND;
    m_turn_ctr    = 0;
    m_follow_ctr  = 0;
    m_straight_ctr = 0;
    drive(0.0f, 0.0f);
}

void roboter_v1_task(DigitalOut& led1)
{
    led1 = !led1;
    *g_en = 1;

    switch (m_state) {

        // -----------------------------------------------------------------
        // STATE 1: BLIND
        // -----------------------------------------------------------------
        case STATE_BLIND:
            drive(BLIND_SPEED, BLIND_SPEED);
            if (all_sensors_active()) {
                m_turn_ctr = TURN_LOOPS;
                m_state    = STATE_TURNING;
            }
            break;

        // -----------------------------------------------------------------
        // STATE 2: TURNING — curved left
        // -----------------------------------------------------------------
        case STATE_TURNING:
            drive(TURN_SPEED, TURN_INNER);
            m_turn_ctr--;
            if (m_turn_ctr <= 0)
                m_state = STATE_REACQUIRE;
            break;

        // -----------------------------------------------------------------
        // STATE 3: REACQUIRE — creep until b3+b4 catch the line
        // -----------------------------------------------------------------
        case STATE_REACQUIRE:
            drive(REACQUIRE_SPEED, REACQUIRE_SPEED);
            if (center_sensors_active()) {
                m_follow_ctr   = 0;
                m_straight_ctr = 0;
                m_state        = STATE_FOLLOW;
            }
            break;

        // -----------------------------------------------------------------
        // STATE 4: FOLLOW — LineFollower handles all curves.
        // Only checks for straight AFTER FOLLOW_MIN_LOOPS (right curve done).
        // -----------------------------------------------------------------
        case STATE_FOLLOW:
            drive(g_lf->getRightWheelVelocity(), g_lf->getLeftWheelVelocity());
            m_follow_ctr++;
            if (m_follow_ctr > FOLLOW_MIN_LOOPS) {
                if (fabsf(g_lf->getAngleRadians()) < STRAIGHT_ENTER_ANGLE) {
                    m_straight_ctr++;
                    if (m_straight_ctr >= STRAIGHT_ENTER_LOOPS)
                        m_state = STATE_STRAIGHT;
                } else {
                    m_straight_ctr = 0;
                }
            }
            break;

        // -----------------------------------------------------------------
        // STATE 5: STRAIGHT — both wheels equal, no wobble.
        // Returns to FOLLOW when curve detected. Resets follow counter so
        // next straight also waits FOLLOW_MIN_LOOPS first.
        // -----------------------------------------------------------------
        case STATE_STRAIGHT:
            drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
            if (fabsf(g_lf->getAngleRadians()) > STRAIGHT_EXIT_ANGLE) {
                m_follow_ctr   = 0;
                m_straight_ctr = 0;
                m_state        = STATE_FOLLOW;
            }
            break;
    }
}

void roboter_v1_reset(DigitalOut& led1)
{
    led1 = 0;
    *g_en          = 0;
    drive(0.0f, 0.0f);
    m_state        = STATE_BLIND;
    m_turn_ctr     = 0;
    m_follow_ctr   = 0;
    m_straight_ctr = 0;
}

void roboter_v1_print()
{
    const char* s = (m_state == STATE_BLIND)     ? "BLIND  " :
                    (m_state == STATE_TURNING)   ? "TURNING" :
                    (m_state == STATE_REACQUIRE) ? "REACQUI" :
                    (m_state == STATE_STRAIGHT)  ? "STRAGHT" : "FOLLOW ";
    int sens = 0;
    for (int i = 0; i < 8; i++)
        sens += (g_lf->getAvgBit(i) >= SENSOR_THRESHOLD) ? 1 : 0;
    printf("State: %s | a=%+.2f fc=%d sc=%d | M1=%+.1f M2=%+.1f\n",
           s,
           g_lf->getAngleRadians(),
           m_follow_ctr,
           m_straight_ctr,
           g_M1->getVelocity(),
           g_M2->getVelocity());
}

#endif // TEST_ROBOTER_V1
