#include "test_config.h"

#ifdef TEST_ALL

#include "test_all.h"

// ---------------------------------------------------------------------------
// Motor parameters (Gear 100:1, KN = 140/12)
// ---------------------------------------------------------------------------
static const float GEAR_RATIO  = 100.0f;
static const float KN          = 140.0f / 12.0f;
static const float VOLTAGE_MAX = 12.0f;
static const float VEL_SIGN    = -1.0f;

// Line follower — FAST mode gains
static const float D_WHEEL   = 0.0393f;
static const float B_WHEEL   = 0.179f;
static const float BAR_DIST  = 0.1836f;
static const float KP        = 4.0f;
static const float KP_NL     = 15.0f;
static const float MAX_SPEED = 3.5f; // RPS

// Servo calibration (measured 2026-03-26)
static const float PULSE_A_MIN   = 0.0303f;
static const float PULSE_A_MAX   = 0.1204f;
static const float PULSE_B_MIN   = 0.0314f;
static const float PULSE_B_MAX   = 0.1232f;
static const float PULSE_360_MIN = 0.0303f;
static const float PULSE_360_MAX = 0.1223f; // midpoint 0.0763 = stop

// ---------------------------------------------------------------------------
// Hardware objects
// ---------------------------------------------------------------------------
static Servo*        g_servo_a   = nullptr;
static Servo*        g_servo_b   = nullptr;
static Servo*        g_servo_360 = nullptr;
static DCMotor*      g_M1        = nullptr;
static DCMotor*      g_M2        = nullptr;
static DigitalOut*   g_en        = nullptr;
static LineFollower* g_lf        = nullptr;
static ColorSensor*  g_cs        = nullptr;

static int   m_lps     = 50;
static float m_input_a = 0.0f;
static float m_input_b = 0.0f;
static bool  m_dir_a   = true;
static bool  m_dir_b   = true;
static int   m_counter = 0;
static float g_cmd_M1  = 0.0f;
static float g_cmd_M2  = 0.0f;
static int   m_color   = -1;

// ---------------------------------------------------------------------------

void all_init(int loops_per_second)
{
    m_lps = loops_per_second;

    // Servos
    static Servo servo_a(PB_D1);
    g_servo_a = &servo_a;
    g_servo_a->calibratePulseMinMax(PULSE_A_MIN, PULSE_A_MAX);

    static Servo servo_b(PB_D2);
    g_servo_b = &servo_b;
    g_servo_b->calibratePulseMinMax(PULSE_B_MIN, PULSE_B_MAX);

    static Servo servo_360(PB_D3);
    g_servo_360 = &servo_360;
    g_servo_360->calibratePulseMinMax(PULSE_360_MIN, PULSE_360_MAX);

    // DC Motors + Line Follower
    static DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, GEAR_RATIO, KN, VOLTAGE_MAX);
    static DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, GEAR_RATIO, KN, VOLTAGE_MAX);
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
    *g_en = 0;
    g_M1->setVelocity(0.0f);
    g_M2->setVelocity(0.0f);

    // Color Sensor (freq_out, led_en, S0, S1, S2, S3)
    static ColorSensor color_sensor(PB_3, PC_3, PA_4, PB_0, PC_1, PC_0);
    g_cs = &color_sensor;
    g_cs->switchLed(ON);

    m_input_a = 0.0f;
    m_input_b = 0.0f;
    m_dir_a   = true;
    m_dir_b   = true;
    m_counter = 0;
    g_cmd_M1  = 0.0f;
    g_cmd_M2  = 0.0f;
    m_color   = -1;
}

void all_task(DigitalOut& led1)
{
    led1 = !led1;

    *g_en = 1;

    if (!g_servo_a->isEnabled())   g_servo_a->enable();
    if (!g_servo_b->isEnabled())   g_servo_b->enable();
    if (!g_servo_360->isEnabled()) g_servo_360->enable();

    // Servo A + B: snap between 0° and 180° every 2 s
    if ((m_counter % (m_lps * 2) == 0) && (m_counter != 0)) {
        m_input_a = m_dir_a ? 1.0f : 0.0f;
        m_dir_a   = !m_dir_a;
        m_input_b = m_dir_b ? 1.0f : 0.0f;
        m_dir_b   = !m_dir_b;
    }

    // 360° servo: full CW / full CCW, 3 s each
    float input_360 = ((m_counter / (m_lps * 3)) % 2 == 0) ? 0.0f : 1.0f;

    g_servo_a->setPulseWidth(m_input_a);
    g_servo_b->setPulseWidth(m_input_b);
    g_servo_360->setPulseWidth(input_360);

    // DC motors controlled by line follower
    g_cmd_M1 = VEL_SIGN * g_lf->getRightWheelVelocity();
    g_cmd_M2 = VEL_SIGN * g_lf->getLeftWheelVelocity();
    g_M1->setVelocity(g_cmd_M1);
    g_M2->setVelocity(g_cmd_M2);

    // Color sensor
    g_cs->readColor();
    m_color = g_cs->getColor();

    m_counter++;
}

void all_reset(DigitalOut& led1)
{
    led1 = 0;
    *g_en = 0;
    g_M1->setVelocity(0.0f);
    g_M2->setVelocity(0.0f);
    g_servo_a->disable();
    g_servo_b->disable();
    g_servo_360->disable();
    m_input_a = 0.0f;
    m_input_b = 0.0f;
    m_dir_a   = true;
    m_dir_b   = true;
    g_cmd_M1  = 0.0f;
    g_cmd_M2  = 0.0f;
    m_counter = 0;
}

void all_print()
{
    printf("LF: a=%+.2f led=%d | M1=%+.1f M2=%+.1f | A=%.2f B=%.2f | Color=%d\n",
           g_lf->getAngleRadians(),
           g_lf->isLedActive() ? 1 : 0,
           g_M1->getVelocity(),
           g_M2->getVelocity(),
           m_input_a,
           m_input_b,
           m_color);
}

#endif // TEST_ALL
