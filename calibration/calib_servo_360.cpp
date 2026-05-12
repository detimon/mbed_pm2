// +==============================================================+
// |        INBETRIEBNAHMEPROTOKOLL - LineXpress FS26             |
// +==============================================================+
// | Datei      : calib_servo_360.cpp
// | Komponente : Parallax 360-Grad-Feedback-Servo P-Regler
// | BMK        : M22
// | Pin(s)     : PWM PB_D3=PB_12, Feedback PB_A0=PC_2 (910 Hz)
// +--------------------------------------------------------------+
// | TESTZIEL   : KP, Toleranz, Offset des P-Reglers tunen, bis 90/180/270/0 Grad jeweils erreicht werden.
// | VORAUSSETZ.: Servo verkabelt, Feedback-Leitung verbunden, calibration_values.h aktualisiert.
// +--------------------------------------------------------------+
// | ERWARTETES VERHALTEN:
// |   Servo erreicht jedes Ziel binnen 5 s, Toleranz +- 2.1 Grad, kein Zittern.
// +--------------------------------------------------------------+
// | TESTDATUM  : __.__.____ | TESTER: _______________
// | ERGEBNIS   : [ ] Bestanden   [ ] Nicht bestanden
// | BEMERKUNGEN: ______________________________________________
// +==============================================================+

#include "test_config.h"

#ifdef CALIB_SERVO_360

#include "calib_servo_360.h"
#include "ServoFeedback360.h"
#include "../calibration/calibration_values.h"

// ---------------------------------------------------------------------------
// Hardware
//   PWM control (white wire) : PB_D3 = PB_12
//   Feedback (yellow wire)   : PC_2  = A0
// ---------------------------------------------------------------------------

// Tuning parameters live in calibration_values.h (SERVO_360_*).
// Target sequence is local to this calibration test.
static const float TARGETS[] = {90.0f, 180.0f, 270.0f, 0.0f};
static const int   N_TARGETS = 4;

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
enum P360State { P360_WARMUP, P360_MOVING, P360_AT_TARGET, P360_PAUSE, P360_DONE };

static ServoFeedback360* g_servo = nullptr;
static P360State         m_state      = P360_WARMUP;
static int               m_target_idx = 0;
static int               m_ctr        = 0;
static int               m_print_ctr  = 0;

// ---------------------------------------------------------------------------

void calib_servo_360_init(int /*loops_per_second*/)
{
    static ServoFeedback360 servo(PB_D3, PC_2, SERVO_360_KP, SERVO_360_TOLERANCE_DEG, SERVO_360_MIN_SPEED, SERVO_360_ANGLE_OFFSET);
    g_servo = &servo;

    m_state      = P360_WARMUP;
    m_target_idx = 0;
    m_ctr        = 0;
    m_print_ctr  = 0;
}

void calib_servo_360_task(DigitalOut& led)
{
    led = !led;
    m_ctr++;

    switch (m_state) {

        // -----------------------------------------------------------------
        case P360_WARMUP:
            // Servo outputs 1500 µs (stop) while we wait for the feedback
            // signal to stabilise after power-on.
            g_servo->stop();
            if (m_ctr >= SERVO_360_WARMUP_LOOPS) {
                m_ctr = 0;
                g_servo->moveToAngle(TARGETS[m_target_idx]);
                printf("Ziel gesetzt: %.1f deg\n", TARGETS[m_target_idx]);
                m_state = P360_MOVING;
            }
            break;

        // -----------------------------------------------------------------
        case P360_MOVING:
            g_servo->update();
            if (g_servo->isAtTarget()) {
                g_servo->stop();
                m_ctr   = 0;
                m_state = P360_AT_TARGET;
            } else if (m_ctr >= SERVO_360_TIMEOUT_LOOPS) {
                g_servo->stop();
                printf("TIMEOUT Ziel %.1f deg | aktuelle Pos: %.1f deg\n",
                       TARGETS[m_target_idx], g_servo->getCurrentAngle());
                m_ctr   = 0;
                m_state = P360_AT_TARGET;
            }
            break;

        // -----------------------------------------------------------------
        case P360_AT_TARGET:
            g_servo->stop();
            printf("Position erreicht: %.1f deg\n", g_servo->getCurrentAngle());
            m_ctr   = 0;
            m_state = P360_PAUSE;
            break;

        // -----------------------------------------------------------------
        case P360_PAUSE:
            g_servo->stop();
            if (m_ctr >= SERVO_360_PAUSE_LOOPS) {
                if (m_target_idx >= N_TARGETS - 1) {
                    printf("Sequenz abgeschlossen.\n");
                    m_state = P360_DONE;
                } else {
                    m_target_idx++;
                    g_servo->moveToAngle(TARGETS[m_target_idx]);
                    printf("Ziel gesetzt: %.1f deg\n", TARGETS[m_target_idx]);
                    m_ctr   = 0;
                    m_state = P360_MOVING;
                }
            }
            break;

        // -----------------------------------------------------------------
        case P360_DONE:
            g_servo->stop();
            break;
    }
}

void calib_servo_360_reset(DigitalOut& led)
{
    if (g_servo) g_servo->stop();
    m_state      = P360_WARMUP;
    m_target_idx = 0;
    m_ctr        = 0;
    m_print_ctr  = 0;
    led          = 0;
}

void calib_servo_360_print()
{
    if (!g_servo) return;

    m_print_ctr++;
    if (m_print_ctr < SERVO_360_PRINT_EVERY) return;
    m_print_ctr = 0;

    const char* state_str =
        (m_state == P360_WARMUP)    ? "WARMUP  " :
        (m_state == P360_MOVING)    ? "MOVING  " :
        (m_state == P360_AT_TARGET) ? "REACHED " :
        (m_state == P360_DONE)      ? "DONE    " : "PAUSE   ";

    printf("Target: %5.1f | Current: %5.1f | Error: %6.1f | Speed: %5.2f | %s | fb=%s\n",
           TARGETS[m_target_idx],
           g_servo->getCurrentAngle(),
           g_servo->getError(),
           g_servo->getSpeed(),
           state_str,
           g_servo->isFeedbackValid() ? "OK" : "--");
}

#endif // CALIB_SERVO_360
