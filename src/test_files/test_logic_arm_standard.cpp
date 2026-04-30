#include "test_config.h"

#ifdef TEST_LOGIC_ARM_STANDARD

#include "test_logic_arm_standard.h"
#include "Servo.h"
#include "ServoFeedback360.h"

// ---------------------------------------------------------------------------
// Hardware
//   D1 horizontal (180°): PC_8   — 0.0=retracted, 0.95=extended
//   D2 vertical   (180°): PB_2   — 1.0=up, 0.28=down BLAU, 0.07=down GRUEN
//   D3 360° Drehteller  : PB_12  — feedback: PC_2
//
// Button (via main.cpp):
//   Press once  → task starts (warmup, then pickup loop)
//   Press again → TEST_RESET() → arm homes, task stops
// ---------------------------------------------------------------------------

static const float D1_RETRACTED    = 0.0f;
static const float D1_EXTENDED     = 0.95f;
static const float D2_UP           = 1.0f;
static const float D2_PARTIAL_DOWN = 0.85f;
static const float D2_DOWN_BLAU    = 0.28f;
static const float D2_DOWN_GRUEN   = 0.07f;

static const float SF360_KP        = 0.005f;
static const float SF360_TOL_DEG   = 2.1f;
static const float SF360_MIN_SPEED = 0.01f;
static const float SF360_OFFSET    = 70.0f;
static const float TRAY_TARGET_DEG = 0.0f;

static const int WARMUP_LOOPS        = 25;   // 0.5 s
static const int TRAY_TIMEOUT_LOOPS  = 250;  // 5.0 s
static const int PHASE_EXTEND_LOOPS  = 50;   // 1.0 s
static const int PHASE_PARTIAL_LOOPS = 25;   // 0.5 s
static const int PHASE_FULL_LOOPS    = 25;   // 0.5 s
static const int PHASE_RAISE_LOOPS   = 30;   // 0.6 s
static const int PHASE_PAUSE_LOOPS   = 50;   // 1.0 s pause between cycles

enum ArmState {
    ARM_WARMUP,
    ARM_TRAY,
    ARM_EXTEND,
    ARM_PARTIAL_DOWN,
    ARM_FULL_DOWN,
    ARM_RAISE,
    ARM_PAUSE     // brief pause before repeating
};

static Servo*            g_d1   = nullptr;
static Servo*            g_d2   = nullptr;
static ServoFeedback360* g_tray = nullptr;

static ArmState m_state    = ARM_WARMUP;
static int      m_ctr      = 0;
static bool     m_use_blau = false;

// ---------------------------------------------------------------------------

void logic_arm_standard_init(int /*loops_per_second*/)
{
    static Servo servo_d1(PC_8);
    static Servo servo_d2(PB_2);
    static ServoFeedback360 tray(PB_12, PC_2,
                                 SF360_KP, SF360_TOL_DEG,
                                 SF360_MIN_SPEED, SF360_OFFSET);
    g_d1   = &servo_d1;
    g_d2   = &servo_d2;
    g_tray = &tray;

    g_d1->calibratePulseMinMax(0.0500f, 0.1050f);
    g_d2->calibratePulseMinMax(0.0200f, 0.1310f);
    g_d1->setMaxAcceleration(0.3f);

    g_d1->enable(D1_RETRACTED);
    g_d2->enable(D2_UP);

    m_state    = ARM_WARMUP;
    m_ctr      = 0;
    m_use_blau = false;

    printf("Warmup...\n");
}

void logic_arm_standard_task(DigitalOut& led)
{
    led = !led;
    m_ctr++;

    switch (m_state) {

        case ARM_WARMUP:
            g_tray->stop();
            // isFeedbackValid() is already true after a reset — skip warmup then
            if (g_tray->isFeedbackValid() || m_ctr >= WARMUP_LOOPS) {
                g_tray->disable();
                m_ctr   = 0;
                m_state = ARM_TRAY;
                g_tray->enable(0.5f);
                g_tray->moveToAngle(TRAY_TARGET_DEG);
                printf("Tray → %.1f°\n", TRAY_TARGET_DEG);
            }
            break;

        case ARM_TRAY:
            g_tray->update();
            if (g_tray->isAtTarget()) {
                g_tray->stop();
                g_d1->enable(D1_EXTENDED);
                g_d2->enable(D2_UP);
                m_ctr   = 0;
                m_state = ARM_EXTEND;
                printf("Tray at %.1f° — arm extending | depth=%s\n",
                       g_tray->getCurrentAngle(),
                       m_use_blau ? "BLAU(0.28)" : "GRUEN(0.07)");
            } else if (m_ctr >= TRAY_TIMEOUT_LOOPS) {
                g_tray->stop();
                g_d1->enable(D1_EXTENDED);
                g_d2->enable(D2_UP);
                m_ctr   = 0;
                m_state = ARM_EXTEND;
                printf("Tray timeout (%.1f°) — continuing\n", g_tray->getCurrentAngle());
            }
            break;

        case ARM_EXTEND:
            if (m_ctr >= PHASE_EXTEND_LOOPS) {
                g_d2->setPulseWidth(D2_PARTIAL_DOWN);
                m_ctr   = 0;
                m_state = ARM_PARTIAL_DOWN;
                printf("D2 partial down\n");
            }
            break;

        case ARM_PARTIAL_DOWN:
            if (m_ctr >= PHASE_PARTIAL_LOOPS) {
                float depth = m_use_blau ? D2_DOWN_BLAU : D2_DOWN_GRUEN;
                g_d2->setPulseWidth(depth);
                m_ctr   = 0;
                m_state = ARM_FULL_DOWN;
                printf("D2 full down (%.2f)\n", depth);
            }
            break;

        case ARM_FULL_DOWN:
            if (m_ctr >= PHASE_FULL_LOOPS) {
                g_d2->setPulseWidth(D2_UP);
                g_d1->setPulseWidth(D1_RETRACTED);
                m_ctr   = 0;
                m_state = ARM_RAISE;
                printf("Raise D2, retract D1\n");
            }
            break;

        case ARM_RAISE:
            if (m_ctr >= PHASE_RAISE_LOOPS) {
                g_d1->disable();
                g_d2->disable();
                g_tray->disable();
                m_use_blau = !m_use_blau;
                m_ctr      = 0;
                m_state    = ARM_PAUSE;
                printf("DONE — next depth=%s\n", m_use_blau ? "BLAU" : "GRUEN");
            }
            break;

        case ARM_PAUSE:
            if (m_ctr >= PHASE_PAUSE_LOOPS) {
                m_ctr   = 0;
                m_state = ARM_TRAY;
                g_tray->enable(0.5f);
                g_tray->moveToAngle(TRAY_TARGET_DEG);
                printf("Repeat — tray → %.1f°\n", TRAY_TARGET_DEG);
            }
            break;
    }
}

void logic_arm_standard_reset(DigitalOut& led)
{
    if (g_d1)   { g_d1->setPulseWidth(D1_RETRACTED); g_d1->disable(); }
    if (g_d2)   { g_d2->setPulseWidth(D2_UP);        g_d2->disable(); }
    if (g_tray) g_tray->stop();
    m_state = ARM_WARMUP;
    m_ctr   = 0;
    led     = 0;
    printf("RESET — homed\n");
}

void logic_arm_standard_print()
{
    static int ctr = 0;
    if (++ctr < 25) return;
    ctr = 0;

    const char* s =
        (m_state == ARM_WARMUP)       ? "WARMUP      " :
        (m_state == ARM_TRAY)         ? "TRAY        " :
        (m_state == ARM_EXTEND)       ? "EXTEND      " :
        (m_state == ARM_PARTIAL_DOWN) ? "PARTIAL_DOWN" :
        (m_state == ARM_FULL_DOWN)    ? "FULL_DOWN   " :
        (m_state == ARM_RAISE)        ? "RAISE       " :
                                        "PAUSE       ";

    printf("%-12s | ctr=%3d | depth=%s | tray=%.1f°\n",
           s, m_ctr,
           m_use_blau ? "BLAU(0.28)" : "GRUEN(0.07)",
           g_tray ? g_tray->getCurrentAngle() : 0.0f);
}

#endif // TEST_LOGIC_ARM_STANDARD
