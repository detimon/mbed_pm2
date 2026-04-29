#include "test_config.h"

#ifdef TEST_PARALLAX_360

#include "test_parallax_360.h"
#include "ServoFeedback360.h"

// ---------------------------------------------------------------------------
// Hardware
//   PWM control (white wire) : PB_D3 = PB_12
//   Feedback (yellow wire)   : PC_2  = A0
// ---------------------------------------------------------------------------

// ===========================================================================
// TUNING-PARAMETER — hier anpassen
// ===========================================================================

// --- P-Regler Verstärkung --------------------------------------------------
// Wie stark der Servo auf den Positionsfehler reagiert.
// Zu gross → Servo zittert / schwingt um das Ziel herum (Überschwingen)
// Zu klein → Servo kommt zu langsam ans Ziel oder bleibt davor stecken
// Sinnvoller Bereich: 0.001 ... 0.010
static const float KP = 0.005f;

// --- Toleranzband (Grad) ---------------------------------------------------
// Innerhalb dieser Abweichung gilt die Position als "erreicht".
// Zu klein → Servo zittert endlos knapp ums Ziel, meldet es nie als erreicht
// Zu gross → Servo stoppt ungenau
// Sinnvoller Bereich: 2.0 ... 10.0
static const float TOLERANCE_DEG = 2.1f;

// --- Mindest-Speed nahe am Ziel --------------------------------------------
// Verhindert, dass der Servo kurz vor dem Ziel im mechanischen Totbereich
// stecken bleibt. Bei sehr kleinem Fehler wird trotzdem mit diesem Mindestwert
// gefahren, bis TOLERANCE_DEG erreicht ist.
// Zu gross → Servo schiesst immer leicht über das Ziel hinaus (Zittern)
// Zu klein → Servo bleibt wenige Grad vor dem Ziel stecken
// Sinnvoller Bereich: 0.02 ... 0.06
static const float MIN_SPEED = 0.01f;

// --- Winkel-Offset (Grad) --------------------------------------------------
// Korrigiert eine mechanische Verschiebung zwischen Sensor-Nullpunkt und
// gewünschtem Nullpunkt. Falls der Servo systematisch zu weit dreht → positiv.
// Falls er zu wenig dreht → negativ. In 5°-Schritten testen.
static const float ANGLE_OFFSET = 70.0f;

// --- Zielsequenz -----------------------------------------------------------
// Winkel die nacheinander angefahren werden (in Grad, 0–360).
// Nach dem letzten Eintrag stoppt der Servo (kein Wiederholen).
static const float TARGETS[] = {90.0f, 180.0f, 270.0f, 0.0f};
static const int   N_TARGETS = 4;

// --- Wartezeit nach Erreichen des Ziels ------------------------------------
// 1 Loop = 20 ms  →  50 Loops = 1 Sekunde
static const int PAUSE_LOOPS = 50;

// ===========================================================================

// Timing (normalerweise nicht ändern nötig)
static const int WARMUP_LOOPS  = 25;  // 500 ms: Feedback-Signal stabilisieren
static const int TIMEOUT_LOOPS = 250; // 5 s:    Ziel aufgeben falls nicht erreicht
static const int PRINT_EVERY   = 10;  // 200 ms: Serial-Output Intervall

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

void parallax_360_init(int /*loops_per_second*/)
{
    static ServoFeedback360 servo(PB_D3, PC_2, KP, TOLERANCE_DEG, MIN_SPEED, ANGLE_OFFSET);
    g_servo = &servo;

    m_state      = P360_WARMUP;
    m_target_idx = 0;
    m_ctr        = 0;
    m_print_ctr  = 0;
}

void parallax_360_task(DigitalOut& led)
{
    led = !led;
    m_ctr++;

    switch (m_state) {

        // -----------------------------------------------------------------
        case P360_WARMUP:
            // Servo outputs 1500 µs (stop) while we wait for the feedback
            // signal to stabilise after power-on.
            g_servo->stop();
            if (m_ctr >= WARMUP_LOOPS) {
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
            } else if (m_ctr >= TIMEOUT_LOOPS) {
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
            if (m_ctr >= PAUSE_LOOPS) {
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

void parallax_360_reset(DigitalOut& led)
{
    if (g_servo) g_servo->stop();
    m_state      = P360_WARMUP;
    m_target_idx = 0;
    m_ctr        = 0;
    m_print_ctr  = 0;
    led          = 0;
}

void parallax_360_print()
{
    if (!g_servo) return;

    m_print_ctr++;
    if (m_print_ctr < PRINT_EVERY) return;
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

#endif // TEST_PARALLAX_360
