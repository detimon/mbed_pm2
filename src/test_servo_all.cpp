#include "test_config.h"

#ifdef TEST_SERVO_ALL

#include "test_servo_all.h"

// Servo objects (static, created once in _init)
static Servo* mp_servo_a   = nullptr; // 180° Servo A — PB_D1 (PC_8)
static Servo* mp_servo_b   = nullptr; // 180° Servo B — PB_D2 (PC_6)
static Servo* mp_servo_360 = nullptr; // 360° Servo   — PB_D3 (PB_12)

static int   m_lps        = 50;
static float m_input_a    = 0.0f; // 180° Servo A: 0.0 = 0°, 0.5 = 90°, 1.0 = 180°
static float m_input_b    = 0.0f; // 180° Servo B: same
static float m_input_360  = 0.5f; // 360° Servo: 0.5 = stop, <0.5 = CW, >0.5 = CCW
static int   m_counter    = 0;

// 360° servo cycles through three phases, each lasting 5 seconds:
//   Phase 0: slow CW  (0.35f)
//   Phase 1: stop     (0.50f)
//   Phase 2: slow CCW (0.65f)
static const float INPUTS_360[3]    = {0.35f, 0.50f, 0.65f};
static const int   PHASE_DURATION_S = 5;

void servo_all_init(int loops_per_second)
{
    m_lps = loops_per_second;

    static Servo servo_a(PB_D1);
    mp_servo_a = &servo_a;
    mp_servo_a->calibratePulseMinMax(0.050f, 0.1050f);
    mp_servo_a->setMaxAcceleration(0.3f);

    static Servo servo_b(PB_D2);
    mp_servo_b = &servo_b;
    mp_servo_b->calibratePulseMinMax(0.050f, 0.1050f);
    mp_servo_b->setMaxAcceleration(0.3f);

    static Servo servo_360(PB_D3);
    mp_servo_360 = &servo_360;
    mp_servo_360->calibratePulseMinMax(0.050f, 0.1050f);
}

void servo_all_task(DigitalOut& led1)
{
    led1 = 1;

    if (!mp_servo_a->isEnabled())   mp_servo_a->enable();
    if (!mp_servo_b->isEnabled())   mp_servo_b->enable();
    if (!mp_servo_360->isEnabled()) mp_servo_360->enable();

    // 180° servos: sweep 0 → 1, one step per second
    if ((m_counter % m_lps == 0) && (m_counter != 0)) {
        if (m_input_a < 1.0f) m_input_a += 0.005f;
        if (m_input_b < 1.0f) m_input_b += 0.005f;
    }

    // 360° servo: switch phase every PHASE_DURATION_S seconds
    int phase    = (m_counter / (m_lps * PHASE_DURATION_S)) % 3;
    m_input_360  = INPUTS_360[phase];

    mp_servo_a->setPulseWidth(m_input_a);
    mp_servo_b->setPulseWidth(m_input_b);
    mp_servo_360->setPulseWidth(m_input_360);

    m_counter++;
}

void servo_all_reset(DigitalOut& led1)
{
    led1 = 0;
    mp_servo_a->disable();
    mp_servo_b->disable();
    mp_servo_360->disable();
    m_input_a   = 0.0f;
    m_input_b   = 0.0f;
    m_input_360 = 0.5f;
    m_counter   = 0;
}

void servo_all_print()
{
    printf("Servo A: %.3f  B: %.3f  360: %.3f\n", m_input_a, m_input_b, m_input_360);
}

#endif // TEST_SERVO_ALL
