#include "test_config.h"

#ifdef TEST_SERVO_ALL

#include "test_servo_all.h"

// Servo objects (static, created once in _init)
static Servo* mp_servo_a   = nullptr; // 180° Servo A — PB_D1 (PC_8)
static Servo* mp_servo_b   = nullptr; // 180° Servo B — PB_D2 (PC_6)
static Servo* mp_servo_360 = nullptr; // 360° Servo   — PB_D3 (PB_12)

// Calibrated pulse ranges (measured 2026-03-26)
static const float PULSE_A_MIN   = 0.0303f;
static const float PULSE_A_MAX   = 0.1204f;
static const float PULSE_B_MIN   = 0.0314f;
static const float PULSE_B_MAX   = 0.1232f;
// 360° servo: midpoint = stop (0.0763f), symmetric range
static const float PULSE_360_MIN = 0.0303f; // 0.0763 - 0.0460
static const float PULSE_360_MAX = 0.1223f; // 0.0763 + 0.0460

static int   m_lps      = 50;
static float m_input_a  = 0.0f;
static float m_input_b  = 0.0f;
static bool  m_dir_a    = true;
static bool  m_dir_b    = true;
static int   m_counter  = 0;

// 360° servo cycles: full CW -> full CCW, 3 s each
static const float INPUTS_360[2]    = {0.0f, 1.0f};
static const int   PHASE_DURATION_S = 3;

void servo_all_init(int loops_per_second)
{
    m_lps = loops_per_second;

    static Servo servo_a(PB_D1);
    mp_servo_a = &servo_a;
    mp_servo_a->calibratePulseMinMax(PULSE_A_MIN, PULSE_A_MAX);

    static Servo servo_b(PB_D2);
    mp_servo_b = &servo_b;
    mp_servo_b->calibratePulseMinMax(PULSE_B_MIN, PULSE_B_MAX);

    static Servo servo_360(PB_D3);
    mp_servo_360 = &servo_360;
    mp_servo_360->calibratePulseMinMax(PULSE_360_MIN, PULSE_360_MAX);

    m_input_a = 0.0f;
    m_input_b = 0.0f;
    m_dir_a   = true;
    m_dir_b   = true;
    m_counter = 0;
}

void servo_all_task(DigitalOut& led1)
{
    led1 = 1;

    if (!mp_servo_a->isEnabled())   mp_servo_a->enable();
    if (!mp_servo_b->isEnabled())   mp_servo_b->enable();
    if (!mp_servo_360->isEnabled()) mp_servo_360->enable();

    // 180° servos: snap between 0° and 180° every 2 seconds (full speed)
    if ((m_counter % (m_lps * 2) == 0) && (m_counter != 0)) {
        m_input_a = m_dir_a ? 1.0f : 0.0f;
        m_dir_a   = !m_dir_a;
        m_input_b = m_dir_b ? 1.0f : 0.0f;
        m_dir_b   = !m_dir_b;
    }

    // 360° servo: full CW -> full CCW, 3 s each phase
    int phase        = (m_counter / (m_lps * PHASE_DURATION_S)) % 2;
    float input_360  = INPUTS_360[phase];

    mp_servo_a->setPulseWidth(m_input_a);
    mp_servo_b->setPulseWidth(m_input_b);
    mp_servo_360->setPulseWidth(input_360);

    m_counter++;
}

void servo_all_reset(DigitalOut& led1)
{
    led1 = 0;
    mp_servo_a->disable();
    mp_servo_b->disable();
    mp_servo_360->disable();
    m_input_a = 0.0f;
    m_input_b = 0.0f;
    m_dir_a   = true;
    m_dir_b   = true;
    m_counter = 0;
}

void servo_all_print()
{
    printf("A: %.2f  B: %.2f  360: %s\n",
           m_input_a, m_input_b,
           (m_counter / (m_lps * PHASE_DURATION_S)) % 2 == 0 ? "CW" : "CCW");
}

#endif // TEST_SERVO_ALL
