// =============================================================================
// test_arm_sequence.cpp — Isolierter Test der Arm-Sequenz (8 Schritte)
//
// Hardware (Drahtzugliste V10):
//   M21 (180° vertikal):    PB_2  (D0)   — Servo-Klasse
//   M20 (180° horizontal):  PC_8  (D1)   — Servo-Klasse
//   M22 (Parallax 360°):    PB_12 (D3)   PWM, PC_2 (A0) Feedback ~910 Hz
//                                        — ServoFeedback360-Klasse
//   Button: BUTTON1 — via main.cpp DebounceIn (50ms intern)
//
// Werte und Ansteuerungs-Pattern 1:1 aus prototype03_v27.cpp übernommen:
//   - Servo-Kalibrierung, KEIN setMaxAcceleration
//   - SF360_OFFSET=110°, KP=0.005, TOL=2.1°
// =============================================================================

#include "test_config.h"

#ifdef TEST_ARM_SEQUENCE

#include "mbed.h"
#include "Servo.h"
#include "ServoFeedback360.h"

// Forward-Deklarationen für main.cpp (Single-File ohne .h)
void test_arm_sequence_init(int loops_per_second);
void test_arm_sequence_task(DigitalOut& led);
void test_arm_sequence_reset(DigitalOut& led);
void test_arm_sequence_print();

// =============================================================================
// SERVO-WERTE
// =============================================================================
static const float D1_RETRACTED    = 0.0f;
static const float D1_EXTENDED_80  = 0.65f;   // 65% ausgefahren (Schritt 3)
static const float D2_UP           = 1.0f;
static const float D2_PARTIAL_DOWN = 0.50f;   // ~1.7 cm runter (Schritt 2)
static const float D2_FULL_DOWN    = 0.28f;   // Pickup-Tiefe (Schritt 4)

static const float D1_PULSE_MIN = 0.0500f;
static const float D1_PULSE_MAX = 0.1050f;
static const float D2_PULSE_MIN = 0.0200f;
static const float D2_PULSE_MAX = 0.1310f;

// =============================================================================
// 360°-SERVO TUNING — exakt aus prototype03_v27.cpp
// =============================================================================
static const float SF360_KP        = 0.005f;
static const float SF360_TOL_DEG   = 2.1f;
static const float SF360_MIN_SPEED = 0.01f;
static const float SF360_OFFSET    = 110.0f;

// =============================================================================
// PHASENZEITEN (Loops @ 50 Hz, 1 Loop = 20 ms)
// =============================================================================
static const int WARMUP_LOOPS_INIT = 25;    // 500 ms in _init()
static const int WARMUP_MAX_LOOPS  = 200;   // 4 s Sicherheits-Cap
static const int TURN_TIMEOUT      = 250;   // 5 s pro Tray-Move

static const int PAUSE_1S_LOOPS = 50;       // 1 s zwischen allen Schritten

// Schritt 5 — Jiggle ±10° (360°) + minimaler Jiggle (M20), Total 1 s = 50 Loops
static const int JIGGLE_TOTAL = 50;
static const int JIGGLE_PH1   = 17;         // Phasenwechsel bei Loop 17
static const int JIGGLE_PH2   = 34;         // Phasenwechsel bei Loop 34
static const float JIGGLE_AMPL      = 10.0f;
static const float D1_JIGGLE_OFFSET = 0.05f; // ~5-8mm Schwung horizontal

// =============================================================================
// STATE MACHINE
// =============================================================================
enum SeqState {
    SEQ_S1_HOMING,       // Alle 3 Servos auf Nullstellung, Tray → 0°
    SEQ_S1_PAUSE,        // 1 s
    SEQ_S2_PARTIAL_DOWN, // Vertikalen Arm 1 cm runter (D2_PARTIAL_DOWN)
    SEQ_S2_PAUSE,        // 1 s
    SEQ_S3_EXTEND,       // Horizontalen Arm ausfahren (80%)
    SEQ_S3_PAUSE,        // 1 s
    SEQ_S4_FULL_DOWN,    // Vertikalen Arm auf Pickup-Tiefe
    SEQ_S4_PAUSE,        // 1 s
    SEQ_S5_JIGGLE,       // Jiggle: Tray ±10° + minimaler M20-Jiggle, 1 s
    SEQ_S5_PAUSE,        // 1 s
    SEQ_S6_ARM_UP,       // Vertikalen Arm auf Nullstellung
    SEQ_S6_PAUSE,        // 1 s
    SEQ_S7_RETRACT,      // Horizontalen Arm auf Nullstellung
    SEQ_S7_PAUSE,        // 1 s
    SEQ_S8_HOME_TRAY,    // 360° Servo zurück auf 0°
    SEQ_DONE             // Fertig — wartet auf Reset (Knopf 2) dann S1
};

static Servo*            g_d1   = nullptr;  // M20 horizontal
static Servo*            g_d2   = nullptr;  // M21 vertikal
static ServoFeedback360* g_tray = nullptr;  // M22 Drehteller

static SeqState m_state = SEQ_S1_HOMING;
static int      m_ctr   = 0;

static float m_d1_pulse = D1_RETRACTED;
static float m_d2_pulse = D2_UP;

// =============================================================================
// HELPERS
// =============================================================================
static float wrapAngle(float a)
{
    while (a >= 360.0f) a -= 360.0f;
    while (a < 0.0f)    a += 360.0f;
    return a;
}

static void d1Enable(float pulse)
{
    g_d1->enable(pulse);
    m_d1_pulse = pulse;
}

static void d2Enable(float pulse)
{
    g_d2->enable(pulse);
    m_d2_pulse = pulse;
}

static void d1Set(float pulse)
{
    g_d1->setPulseWidth(pulse);
    m_d1_pulse = pulse;
}

static void d2Set(float pulse)
{
    g_d2->setPulseWidth(pulse);
    m_d2_pulse = pulse;
}

static void trayMoveTo(float deg)
{
    g_tray->enable(0.5f);
    g_tray->moveToAngle(wrapAngle(deg));
}

static const char* stateName(SeqState s)
{
    switch (s) {
        case SEQ_S1_HOMING:       return "S1_HOMING ";
        case SEQ_S1_PAUSE:        return "S1_PAUSE  ";
        case SEQ_S2_PARTIAL_DOWN: return "S2_PART_DN";
        case SEQ_S2_PAUSE:        return "S2_PAUSE  ";
        case SEQ_S3_EXTEND:       return "S3_EXTEND ";
        case SEQ_S3_PAUSE:        return "S3_PAUSE  ";
        case SEQ_S4_FULL_DOWN:    return "S4_FULL_DN";
        case SEQ_S4_PAUSE:        return "S4_PAUSE  ";
        case SEQ_S5_JIGGLE:       return "S5_JIGGLE ";
        case SEQ_S5_PAUSE:        return "S5_PAUSE  ";
        case SEQ_S6_ARM_UP:       return "S6_ARM_UP ";
        case SEQ_S6_PAUSE:        return "S6_PAUSE  ";
        case SEQ_S7_RETRACT:      return "S7_RETRACT";
        case SEQ_S7_PAUSE:        return "S7_PAUSE  ";
        case SEQ_S8_HOME_TRAY:    return "S8_HOME   ";
        case SEQ_DONE:            return "DONE      ";
    }
    return "?         ";
}

static void printDebug(const char* tag)
{
    printf("[%s] tray=%5.1f° (soll=%5.1f°, err=%6.1f°) | M20=%.2f | M21=%.2f\n",
           tag,
           g_tray->getCurrentAngle(),
           g_tray->getTargetAngle(),
           g_tray->getError(),
           m_d1_pulse,
           m_d2_pulse);
}

// =============================================================================
// _init() — wird einmal beim Programmstart aufgerufen
// =============================================================================
void test_arm_sequence_init(int /*loops_per_second*/)
{
    static Servo servo_d1(PC_8);     // M20 horizontal
    static Servo servo_d2(PB_2);     // M21 vertikal
    static ServoFeedback360 tray(PB_12, PC_2,
                                 SF360_KP, SF360_TOL_DEG,
                                 SF360_MIN_SPEED, SF360_OFFSET);
    g_d1   = &servo_d1;
    g_d2   = &servo_d2;
    g_tray = &tray;

    g_d1->calibratePulseMinMax(D1_PULSE_MIN, D1_PULSE_MAX);
    g_d2->calibratePulseMinMax(D2_PULSE_MIN, D2_PULSE_MAX);

    d1Enable(D1_RETRACTED);
    d2Enable(D2_UP);

    g_tray->stop();
    int warmup_ctr = 0;
    while (warmup_ctr < WARMUP_LOOPS_INIT || !g_tray->isFeedbackValid()) {
        thread_sleep_for(20);
        g_tray->stop();
        warmup_ctr++;
        if (warmup_ctr > WARMUP_MAX_LOOPS) break;
    }
    g_tray->disable();

    m_state = SEQ_S1_HOMING;
    m_ctr   = 0;

    printf("\n=== TEST_ARM_SEQUENCE (8 Schritte) ===\n");
    printf("Warmup OK | tray=%.1f° | feedback=%d\n",
           g_tray->getCurrentAngle(), g_tray->isFeedbackValid() ? 1 : 0);
    printf("Bereit. Buttondruck startet Sequenz.\n");
}

// =============================================================================
// _task() — wird ab 1. Buttondruck alle 20 ms aufgerufen
// =============================================================================
void test_arm_sequence_task(DigitalOut& led)
{
    led = !led;

    switch (m_state) {

        // =====================================================================
        // SCHRITT 1: NULLSTELLUNG — Tray → 0°, M21 → UP, M20 → RETRACTED
        // =====================================================================
        case SEQ_S1_HOMING: {
            if (m_ctr == 0) {
                d2Enable(D2_UP);
                d1Enable(D1_RETRACTED);
                trayMoveTo(0.0f);
                printf("\nSchritt 1: Nullstellung — tray->0°, M21->%.2f, M20->%.2f\n",
                       D2_UP, D1_RETRACTED);
            }
            g_tray->update();
            m_ctr++;
            if (g_tray->isAtTarget() || m_ctr >= TURN_TIMEOUT) {
                if (!g_tray->isAtTarget())
                    printf("  TIMEOUT bei %.1f°\n", g_tray->getCurrentAngle());
                g_tray->stop();
                printf("Schritt 1: OK — Ist: %.1f°\n", g_tray->getCurrentAngle());
                m_ctr   = 0;
                m_state = SEQ_S1_PAUSE;
            }
            break;
        }

        case SEQ_S1_PAUSE:
            if (++m_ctr >= PAUSE_1S_LOOPS) { m_ctr = 0; m_state = SEQ_S2_PARTIAL_DOWN; }
            break;

        // =====================================================================
        // SCHRITT 2: VERTIKALEN ARM 1 CM RUNTER
        // =====================================================================
        case SEQ_S2_PARTIAL_DOWN:
            if (m_ctr == 0) {
                d2Set(D2_PARTIAL_DOWN);
                printf("\nSchritt 2: Vertikaler Arm 1 cm runter (M21=%.2f)\n",
                       D2_PARTIAL_DOWN);
                printDebug("S2");
            }
            m_ctr++;
            if (m_ctr >= 2) { m_ctr = 0; m_state = SEQ_S2_PAUSE; }
            break;

        case SEQ_S2_PAUSE:
            if (++m_ctr >= PAUSE_1S_LOOPS) { m_ctr = 0; m_state = SEQ_S3_EXTEND; }
            break;

        // =====================================================================
        // SCHRITT 3: HORIZONTALEN ARM AUSFAHREN (80%)
        // =====================================================================
        case SEQ_S3_EXTEND:
            if (m_ctr == 0) {
                d1Enable(D1_EXTENDED_80);
                printf("\nSchritt 3: Horizontaler Arm ausgefahren (M20=%.2f)\n",
                       D1_EXTENDED_80);
                printDebug("S3");
            }
            m_ctr++;
            if (m_ctr >= 2) { m_ctr = 0; m_state = SEQ_S3_PAUSE; }
            break;

        case SEQ_S3_PAUSE:
            if (++m_ctr >= PAUSE_1S_LOOPS) { m_ctr = 0; m_state = SEQ_S4_FULL_DOWN; }
            break;

        // =====================================================================
        // SCHRITT 4: VERTIKALEN ARM AUF PICKUP-TIEFE
        // =====================================================================
        case SEQ_S4_FULL_DOWN:
            if (m_ctr == 0) {
                d2Set(D2_FULL_DOWN);
                printf("\nSchritt 4: Vertikaler Arm auf Pickup-Tiefe (M21=%.2f)\n",
                       D2_FULL_DOWN);
                printDebug("S4");
            }
            m_ctr++;
            if (m_ctr >= 2) { m_ctr = 0; m_state = SEQ_S4_PAUSE; }
            break;

        case SEQ_S4_PAUSE:
            if (++m_ctr >= PAUSE_1S_LOOPS) { m_ctr = 0; m_state = SEQ_S5_JIGGLE; }
            break;

        // =====================================================================
        // SCHRITT 5: JIGGLE — Tray ±10° + minimaler M20-Jiggle (1 s)
        //   Phase A (ctr=0):  tray->350°(-10°), M20->0.85 (vor)
        //   Phase B (ctr=17): tray->10°(+10°),  M20->0.75 (zurück)
        //   Phase C (ctr=34): tray->0°,          M20->0.80 (mitte)
        // =====================================================================
        case SEQ_S5_JIGGLE:
            if (m_ctr == 0) {
                trayMoveTo(-JIGGLE_AMPL);                           // 350°
                d1Set(D1_EXTENDED_80 + D1_JIGGLE_OFFSET);
                printf("\nSchritt 5: Jiggle ±10° + M20-Jiggle (1 s)\n");
            } else if (m_ctr == JIGGLE_PH1) {
                trayMoveTo(JIGGLE_AMPL);                            // 10°
                d1Set(D1_EXTENDED_80 - D1_JIGGLE_OFFSET);
            } else if (m_ctr == JIGGLE_PH2) {
                trayMoveTo(0.0f);                                   // 0°
                d1Set(D1_EXTENDED_80);
            }
            g_tray->update();
            m_ctr++;
            if (m_ctr >= JIGGLE_TOTAL) {
                g_tray->stop();
                printf("Schritt 5: Jiggle fertig, tray=%.1f°\n",
                       g_tray->getCurrentAngle());
                m_ctr   = 0;
                m_state = SEQ_S5_PAUSE;
            }
            break;

        case SEQ_S5_PAUSE:
            if (++m_ctr >= PAUSE_1S_LOOPS) { m_ctr = 0; m_state = SEQ_S6_ARM_UP; }
            break;

        // =====================================================================
        // SCHRITT 6: VERTIKALEN ARM AUF NULLSTELLUNG
        // =====================================================================
        case SEQ_S6_ARM_UP:
            if (m_ctr == 0) {
                d2Set(D2_UP);
                printf("\nSchritt 6: Vertikaler Arm hoch (M21=%.2f)\n", D2_UP);
                printDebug("S6");
            }
            m_ctr++;
            if (m_ctr >= 2) { m_ctr = 0; m_state = SEQ_S6_PAUSE; }
            break;

        case SEQ_S6_PAUSE:
            if (++m_ctr >= PAUSE_1S_LOOPS) { m_ctr = 0; m_state = SEQ_S7_RETRACT; }
            break;

        // =====================================================================
        // SCHRITT 7: HORIZONTALEN ARM AUF NULLSTELLUNG
        // =====================================================================
        case SEQ_S7_RETRACT:
            if (m_ctr == 0) {
                d1Set(D1_RETRACTED);
                printf("\nSchritt 7: Horizontaler Arm eingefahren (M20=%.2f)\n",
                       D1_RETRACTED);
                printDebug("S7");
            }
            m_ctr++;
            if (m_ctr >= 2) { m_ctr = 0; m_state = SEQ_S7_PAUSE; }
            break;

        case SEQ_S7_PAUSE:
            if (++m_ctr >= PAUSE_1S_LOOPS) { m_ctr = 0; m_state = SEQ_S8_HOME_TRAY; }
            break;

        // =====================================================================
        // SCHRITT 8: 360° SERVO ZURÜCK AUF 0°
        // =====================================================================
        case SEQ_S8_HOME_TRAY: {
            if (m_ctr == 0) {
                trayMoveTo(0.0f);
                printf("\nSchritt 8: Drehteller -> 0°\n");
            }
            g_tray->update();
            m_ctr++;
            if (g_tray->isAtTarget() || m_ctr >= TURN_TIMEOUT) {
                if (!g_tray->isAtTarget())
                    printf("  TIMEOUT bei %.1f°\n", g_tray->getCurrentAngle());
                g_tray->stop();
                printf("Schritt 8: OK — Ist: %.1f°\n", g_tray->getCurrentAngle());
                printDebug("S8");

                thread_sleep_for(500);
                g_d1->disable();
                g_d2->disable();
                g_tray->disable();
                led = 0;

                printf("\nSequenz abgeschlossen. Knopf 2x druecken um neu zu starten.\n");
                m_state = SEQ_DONE;
                m_ctr   = 0;
            }
            break;
        }

        case SEQ_DONE:
            break;
    }
}

// =============================================================================
// _reset() — Sequenz stoppen und auf S1 zurücksetzen
// =============================================================================
void test_arm_sequence_reset(DigitalOut& led)
{
    if (g_d1)   g_d1->disable();
    if (g_d2)   g_d2->disable();
    if (g_tray) g_tray->stop();

    m_state    = SEQ_S1_HOMING;
    m_ctr      = 0;
    m_d1_pulse = D1_RETRACTED;
    m_d2_pulse = D2_UP;
    led        = 0;

    printf("STOP — Reset auf S1. Naechster Knopfdruck startet erneut.\n");
}

// =============================================================================
// _print() — auf 5 Hz throttlen
// =============================================================================
void test_arm_sequence_print()
{
    static int print_ctr = 0;
    if (++print_ctr < 10) return;
    print_ctr = 0;

    if (!g_tray) return;

    printf("[%s] tray=%5.1f° (soll=%5.1f°, err=%6.1f°) | M20=%.2f | M21=%.2f\n",
           stateName(m_state),
           g_tray->getCurrentAngle(),
           g_tray->getTargetAngle(),
           g_tray->getError(),
           m_d1_pulse,
           m_d2_pulse);
}

#endif // TEST_ARM_SEQUENCE
