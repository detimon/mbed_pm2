#include "test_config.h"

#ifdef TEST_ROBOTER_V3

#include "roboter_v3.h"

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
static const float BACKWARD_SPEED = 1.0f;  // pure backwards speed (both motors equal)

static const int   STRAIGHT_LOOPS  = 85;   // 1.7 s at 50 Hz
static const int   FOLLOW_1S_LOOPS = 50;   // 1 s line-follow after crossing detected
static const int   BRAKE_LOOPS     = 12;   // 0.24 s smooth brake from line-follow speed to 0
static const int   PAUSE_LOOPS     = 20;   // 0.4 s standstill before driving backwards
static const int   BACKWARD_LOOPS  = 185;  // 3.7 s backwards drive
static const int   ACCEL_LOOPS     = 12;   // 0.24 s smooth acceleration at start of backwards
static const int   STOP_GUARD      = 75;   // 1.5 s guard after TURN_SPOT before crossing triggers

static const float SENSOR_THRESHOLD = 0.5f;

// ---------------------------------------------------------------------------
// States
// ---------------------------------------------------------------------------
enum State {
    STATE_BLIND      = 1, // drive straight until all 8 sensors see the sideline
    STATE_STRAIGHT   = 2, // drive straight for 1.7 s after sideline
    STATE_TURN_SPOT  = 3, // pivot in-place 90° until line is re-acquired
    STATE_FOLLOW     = 4, // line follower — until all 8 sensors active
    STATE_FOLLOW_1S  = 5, // keep line-following for 1 s after crossing detected
    STATE_BRAKE      = 6, // smooth deceleration from line-follow speed to 0
    STATE_PAUSE      = 7, // 0.4 s standstill
    STATE_BACKWARD   = 8, // smooth accel into full backwards speed, rapid stop after 3.2 s
    STATE_HALT       = 9  // stop forever
};

// ---------------------------------------------------------------------------
// Hardware objects
// ---------------------------------------------------------------------------
static DCMotor*      g_M1 = nullptr;
static DCMotor*      g_M2 = nullptr;
static DigitalOut*   g_en = nullptr;
static LineFollower* g_lf = nullptr;

static State m_state           = STATE_BLIND;
static int   m_straight_ctr    = 0;
static int   m_follow_ctr      = 0;
static int   m_brake_ctr       = 0;
static int   m_pause_ctr       = 0;
static int   m_backward_ctr    = 0;
static int   m_guard_ctr       = 0;
static float m_brake_start_M1  = 0.0f;  // velocity captured when brake begins
static float m_brake_start_M2  = 0.0f;
static float g_cmd_M1          = 0.0f;
static float g_cmd_M2          = 0.0f;

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
void roboter_v3_init(int loops_per_second)
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

    *g_en             = 0;
    m_state           = STATE_BLIND;
    m_straight_ctr    = 0;
    m_follow_ctr      = 0;
    m_brake_ctr       = 0;
    m_pause_ctr       = 0;
    m_backward_ctr    = 0;
    m_guard_ctr       = 0;
    m_brake_start_M1  = 0.0f;
    m_brake_start_M2  = 0.0f;
    g_M1->setVelocity(0.0f);
    g_M2->setVelocity(0.0f);
}

void roboter_v3_task(DigitalOut& led)
{
    led = !led;
    *g_en = 1;

    switch (m_state) {

        // ----------------------------------------------------------------
        // BLIND: drive straight until all 8 sensors see the sideline
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
        //            center sensors pick up the line again
        // ----------------------------------------------------------------
        case STATE_TURN_SPOT:
            g_M1->setVelocity(VEL_SIGN *  TURN_SPEED);
            g_M2->setVelocity(VEL_SIGN * -TURN_SPEED);
            if (center_sensors_active()) {
                m_guard_ctr = STOP_GUARD;
                m_state     = STATE_FOLLOW;
            }
            break;

        // ----------------------------------------------------------------
        // FOLLOW: pure line follower
        // Transitions to FOLLOW_1S when all 8 sensors are active (crossing)
        // ----------------------------------------------------------------
        case STATE_FOLLOW:
            g_cmd_M1 = VEL_SIGN * g_lf->getRightWheelVelocity();
            g_cmd_M2 = VEL_SIGN * g_lf->getLeftWheelVelocity();
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
            g_cmd_M1 = VEL_SIGN * g_lf->getRightWheelVelocity();
            g_cmd_M2 = VEL_SIGN * g_lf->getLeftWheelVelocity();
            g_M1->setVelocity(g_cmd_M1);
            g_M2->setVelocity(g_cmd_M2);

            m_follow_ctr--;
            if (m_follow_ctr <= 0) {
                // capture current velocities for smooth brake
                m_brake_start_M1 = g_cmd_M1;
                m_brake_start_M2 = g_cmd_M2;
                m_brake_ctr      = BRAKE_LOOPS;
                m_state          = STATE_BRAKE;
            }
            break;

        // ----------------------------------------------------------------
        // BRAKE: linearly ramp both motors from their current speed to 0
        //        over BRAKE_LOOPS (0.5 s)
        // ----------------------------------------------------------------
        case STATE_BRAKE: {
            float t   = static_cast<float>(m_brake_ctr) / static_cast<float>(BRAKE_LOOPS);
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
        // BACKWARD: drive both motors backwards for 3.2 s
        // First ACCEL_LOOPS (0.5 s): ramp from 0 to full BACKWARD_SPEED
        // Remaining loops: full speed
        // At counter == 0: stop immediately (rapid stop)
        // ----------------------------------------------------------------
        case STATE_BACKWARD: {
            int elapsed = BACKWARD_LOOPS - m_backward_ctr;
            float ramp  = (elapsed < ACCEL_LOOPS)
                              ? (static_cast<float>(elapsed) / static_cast<float>(ACCEL_LOOPS))
                              : 1.0f;
            float spd = -VEL_SIGN * BACKWARD_SPEED * ramp;
            g_M1->setVelocity(spd);
            g_M2->setVelocity(spd);

            m_backward_ctr--;
            if (m_backward_ctr <= 0) {
                g_M1->setVelocity(0.0f);
                g_M2->setVelocity(0.0f);
                m_state = STATE_HALT;
            }
            break;
        }

        // ----------------------------------------------------------------
        // HALT: stop forever
        // ----------------------------------------------------------------
        case STATE_HALT:
            g_M1->setVelocity(0.0f);
            g_M2->setVelocity(0.0f);
            break;
    }
}

void roboter_v3_reset(DigitalOut& led)
{
    *g_en            = 0;
    g_M1->setVelocity(0.0f);
    g_M2->setVelocity(0.0f);
    g_cmd_M1         = 0.0f;
    g_cmd_M2         = 0.0f;
    m_state          = STATE_BLIND;
    m_straight_ctr   = 0;
    m_follow_ctr     = 0;
    m_brake_ctr      = 0;
    m_pause_ctr      = 0;
    m_backward_ctr   = 0;
    m_guard_ctr      = 0;
    m_brake_start_M1 = 0.0f;
    m_brake_start_M2 = 0.0f;
    led              = 0;
}

void roboter_v3_print()
{
    const char* s = (m_state == STATE_BLIND)     ? "BLIND     " :
                    (m_state == STATE_STRAIGHT)  ? "STRAIGHT  " :
                    (m_state == STATE_TURN_SPOT) ? "TURN_SPOT " :
                    (m_state == STATE_FOLLOW)    ? "FOLLOW    " :
                    (m_state == STATE_FOLLOW_1S) ? "FOLLOW_1S " :
                    (m_state == STATE_BRAKE)     ? "BRAKE     " :
                    (m_state == STATE_PAUSE)     ? "PAUSE     " :
                    (m_state == STATE_BACKWARD)  ? "BACKWARD  " : "HALT      ";
    printf("State: %s | a=%.3f | M1=%+.2f M2=%+.2f | all8=%d\n",
           s,
           g_lf->getAngleRadians(),
           g_M1->getVelocity(),
           g_M2->getVelocity(),
           all_sensors_active() ? 1 : 0);
}

#endif // TEST_ROBOTER_V3
