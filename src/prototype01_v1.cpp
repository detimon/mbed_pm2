#include "test_config.h"

#ifdef PROTOTYPE_01_V1

#include "prototype01_v1.h"

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

static const float BLIND_SPEED     = 1.0f;
static const float REACQUIRE_SPEED = 0.4f;
static const float TURN_SPEED      = 1.0f;
static const float TURN_INNER      = 0.3f;
static const int   TURN_LOOPS      = 80;

static const float SENSOR_THRESHOLD = 0.5f;

// Forward line follower
static const float FOLLOW_LEFT_BIAS  = 0.30f; // extra power on left wheel going forward
static const int   FORWARD_GUARD     = 10;    // ~0.2 s guard to clear trigger zone after backward
// Backward correction sequence
static const float BACKWARD_ANGLE_KP  = 0.6f;  // angle correction gain while reversing
static const float BACKWARD_LEFT_BIAS = 0.10f; // slight reduction on left wheel going backward
static const int   BACKWARD_LOOPS    = 33;    // 0.66 s at 50 Hz
static const int   PAUSE_LOOPS       = 15;   // ~300 ms stop after backward
static const int   REPEAT_COUNT      = 6;    // number of backward cycles before done
// Final stop
static const int   FINAL_STOP_LOOPS  = 250;  // 5 s stop on sideline (b0-b7)

// ---------------------------------------------------------------------------
// States
// ---------------------------------------------------------------------------
enum State {
    STATE_BLIND        = 1,
    STATE_TURNING      = 2,
    STATE_REACQUIRE    = 3,
    STATE_FOLLOW       = 4,
    STATE_BACKWARD     = 5, // backward line follower
    STATE_PAUSE        = 6, // brief stop between cycles
    STATE_FINAL_STOP   = 7  // 5 s stop on sideline after all corrections, then FOLLOW forever
};

// ---------------------------------------------------------------------------
// Hardware objects
// ---------------------------------------------------------------------------
static DCMotor*      g_M1 = nullptr;
static DCMotor*      g_M2 = nullptr;
static DigitalOut*   g_en = nullptr;
static LineFollower* g_lf = nullptr;

static State m_state      = STATE_BLIND;
static int   m_turn_ctr   = 0;
static int   m_guard_ctr  = 0;
static int   m_back_ctr   = 0;
static int   m_pause_ctr  = 0;
static int   m_repeat_ctr = REPEAT_COUNT;
static int   m_final_ctr  = 0;
static float g_cmd_M1     = 0.0f;
static float g_cmd_M2     = 0.0f;

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

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void roboter_v1_init(int loops_per_second)
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

    *g_en        = 0;
    m_state      = STATE_BLIND;
    m_turn_ctr   = 0;
    m_guard_ctr  = 0;
    m_back_ctr   = 0;
    m_pause_ctr  = 0;
    m_repeat_ctr = REPEAT_COUNT;
    m_final_ctr  = 0;
    g_M1->setVelocity(0.0f);
    g_M2->setVelocity(0.0f);
}

void roboter_v1_task(DigitalOut& led)
{
    led = !led;
    *g_en = 1;

    switch (m_state) {

        case STATE_BLIND:
            g_M1->setVelocity(VEL_SIGN * BLIND_SPEED);
            g_M2->setVelocity(VEL_SIGN * BLIND_SPEED);
            if (all_sensors_active()) {
                m_turn_ctr = TURN_LOOPS;
                m_state    = STATE_TURNING;
            }
            break;

        case STATE_TURNING:
            g_M1->setVelocity(VEL_SIGN * TURN_SPEED);
            g_M2->setVelocity(VEL_SIGN * TURN_INNER);
            m_turn_ctr--;
            if (m_turn_ctr <= 0)
                m_state = STATE_REACQUIRE;
            break;

        case STATE_REACQUIRE:
            g_M1->setVelocity(VEL_SIGN * REACQUIRE_SPEED);
            g_M2->setVelocity(VEL_SIGN * REACQUIRE_SPEED);
            if (center_sensors_active())
                m_state = STATE_FOLLOW;
            break;

        case STATE_FOLLOW:
            g_cmd_M1 = VEL_SIGN *  g_lf->getRightWheelVelocity();
            g_cmd_M2 = VEL_SIGN * (g_lf->getLeftWheelVelocity() + FOLLOW_LEFT_BIAS);
            g_M1->setVelocity(g_cmd_M1);
            g_M2->setVelocity(g_cmd_M2);
            // Count down forward guard before allowing trigger again
            if (m_guard_ctr > 0)
                m_guard_ctr--;
            // Correction trigger: b2+b3+b4 active AND guard elapsed AND cycles remain
            if (m_repeat_ctr > 0 && m_guard_ctr == 0 &&
                g_lf->getAvgBit(2) >= SENSOR_THRESHOLD &&
                g_lf->getAvgBit(3) >= SENSOR_THRESHOLD &&
                g_lf->getAvgBit(4) >= SENSOR_THRESHOLD) {
                g_M1->setVelocity(0.0f);
                g_M2->setVelocity(0.0f);
                m_back_ctr = BACKWARD_LOOPS;
                m_state    = STATE_BACKWARD;
            }
            // After all corrections: stop 5 s when sideline (b0-b7) detected
            if (m_repeat_ctr == 0 && all_sensors_active()) {
                g_M1->setVelocity(0.0f);
                g_M2->setVelocity(0.0f);
                m_final_ctr = FINAL_STOP_LOOPS;
                m_state     = STATE_FINAL_STOP;
            }
            break;

        case STATE_BACKWARD: {
            // Backward: negate+swap + flipped angle correction (corrects rightward drift)
            float angle_corr = BACKWARD_ANGLE_KP * g_lf->getAngleRadians();
            g_cmd_M1 = VEL_SIGN * (-g_lf->getLeftWheelVelocity()  + angle_corr);
            g_cmd_M2 = VEL_SIGN * (-g_lf->getRightWheelVelocity() - angle_corr - BACKWARD_LEFT_BIAS);
            g_M1->setVelocity(g_cmd_M1);
            g_M2->setVelocity(g_cmd_M2);
            m_back_ctr--;
            if (m_back_ctr <= 0) {
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
                m_repeat_ctr--;
                m_guard_ctr = FORWARD_GUARD;
                m_state     = STATE_FOLLOW; // always go back to FOLLOW; sideline detected there
            }
            break;

        case STATE_FINAL_STOP:
            // Hold still for 5 s on the sideline
            g_M1->setVelocity(0.0f);
            g_M2->setVelocity(0.0f);
            m_final_ctr--;
            if (m_final_ctr <= 0) {
                m_repeat_ctr = -1; // prevent any trigger from firing again
                m_state      = STATE_FOLLOW;
            }
            break;
    }
}

void roboter_v1_reset(DigitalOut& led)
{
    *g_en        = 0;
    g_M1->setVelocity(0.0f);
    g_M2->setVelocity(0.0f);
    g_cmd_M1     = 0.0f;
    g_cmd_M2     = 0.0f;
    m_state      = STATE_BLIND;
    m_turn_ctr   = 0;
    m_guard_ctr  = 0;
    m_back_ctr   = 0;
    m_pause_ctr  = 0;
    m_repeat_ctr = REPEAT_COUNT;
    m_final_ctr  = 0;
    led          = 0;
}

void roboter_v1_print()
{
    const char* s = (m_state == STATE_BLIND)       ? "BLIND   " :
                    (m_state == STATE_TURNING)     ? "TURNING " :
                    (m_state == STATE_REACQUIRE)   ? "REACQUI " :
                    (m_state == STATE_BACKWARD)    ? "BACKWD  " :
                    (m_state == STATE_PAUSE)       ? "PAUSE   " :
                    (m_state == STATE_FINAL_STOP)  ? "FNLSTP  " : "FOLLOW  ";
    printf("State: %s | a=%.3f | rep=%d | M1=%+.2f M2=%+.2f\n",
           s,
           g_lf->getAngleRadians(),
           m_repeat_ctr,
           g_M1->getVelocity(),
           g_M2->getVelocity());
}

#endif // PROTOTYPE_01_V1
