// =============================================================================
// test_arm_sequence.cpp — Isolierter Test der Aufpick-Sequenz (10 Schritte)
//
// Hardware (Drahtzugliste V10):
//   M21 (180° vertikal):    PB_2  (D0)   — Servo-Klasse
//   M20 (180° horizontal):  PC_8  (D1)   — Servo-Klasse
//   M22 (Parallax 360°):    PB_12 (D3)   PWM, PC_2 (A0) Feedback ~910 Hz
//                                        — ServoFeedback360-Klasse
//   Button: BUTTON1 — via main.cpp DebounceIn (50ms intern)
//
// Werte und Ansteuerungs-Pattern 1:1 aus prototype03_v27.cpp übernommen:
//   - Servo-Kalibrierung, Positionswerte, KEIN setMaxAcceleration
//   - SF360_OFFSET=70°, KP=0.005, TOL=2.1° (V27 unverändert)
//   - Drehrichtung über moveToAngle() — wie V27 (kürzester Weg, +330° als 2 Substeps)
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
// SERVO-WERTE — exakt aus prototype03_v27.cpp (Zeile 91-98, 487-488)
// =============================================================================
static const float D1_RETRACTED      = 0.0f;
static const float D1_EXTENDED       = 0.95f;
static const float D2_UP             = 1.0f;
static const float D2_PARTIAL_DOWN   = 0.85f;   // ~10 mm runter (Zwischenposition)
static const float D2_FULL_DOWN      = 0.28f;   // SERVO_D2_DOWN_BLAU (volle Pickup-Tiefe)

static const float D1_PULSE_MIN = 0.0500f;
static const float D1_PULSE_MAX = 0.1050f;
static const float D2_PULSE_MIN = 0.0200f;
static const float D2_PULSE_MAX = 0.1310f;

// =============================================================================
// 360°-SERVO TUNING — exakt aus prototype03_v27.cpp (Zeile 115-118)
// =============================================================================
static const float SF360_KP        = 0.005f;
static const float SF360_TOL_DEG   = 2.1f;     // V27 unverändert
static const float SF360_MIN_SPEED = 0.01f;
static const float SF360_OFFSET    = 70.0f;    // V27 unverändert

// =============================================================================
// PHASENZEITEN (Loops @ 50 Hz, 1 Loop = 20 ms)
// =============================================================================
static const int WARMUP_LOOPS_INIT = 25;    // 500 ms in _init()
static const int WARMUP_MAX_LOOPS  = 200;   // 4 s Sicherheits-Cap
static const int TURN_TIMEOUT      = 250;   // 5 s pro Tray-Move

static const int PAUSE_500_LOOPS = 25;
static const int PAUSE_300_LOOPS = 15;
static const int PAUSE_200_LOOPS = 10;

// Step 7 — grober Jiggle ±10°, Total 1 s = 50 Loops, 3 Phasen à ~17 Loops
static const int JIGGLE_GROB_TOTAL = 50;
static const int JIGGLE_GROB_PH1   = 17;
static const int JIGGLE_GROB_PH2   = 34;

// Step 9 — feiner Jiggle ±5°, Total 1.5 s = 75 Loops, 3 Phasen à 25 Loops
static const int JIGGLE_FEIN_TOTAL = 75;
static const int JIGGLE_FEIN_PH1   = 25;
static const int JIGGLE_FEIN_PH2   = 50;

// =============================================================================
// SEQUENZ-PARAMETER
// =============================================================================
static const float TARGET_HOME    =  0.0f;   // Schritt 1
static const float TARGET_STEP2   = 30.0f;   // Schritt 2: +30° CW
static const float STEP5_HALF1    = 195.0f;  // 30 + 165 (CW-Zwischenpunkt)
static const float TARGET_STEP5   =  0.0f;   // Schritt 5: zurück auf 0° (CW +330°)
static const float TARGET_STEP8   = 45.0f;   // Schritt 8: auf 45° (CW +45°)

static const float JIGGLE_GROB_AMPL = 10.0f;
static const float JIGGLE_FEIN_AMPL =  5.0f;
static const float D1_JIGGLE_OFFSET = 0.05f; // V27 D1_JIGGLE_OFFSET, ~5-8mm Schwung

// =============================================================================
// STATE MACHINE
// =============================================================================
enum SeqState {
    SEQ_S1_HOMING,        // Tray → 0°, M21 → UP, M20 → RETRACTED
    SEQ_S1_PAUSE,         // 500 ms
    SEQ_S2_TURN_30,       // Tray → 30°
    SEQ_S2_PAUSE,         // 300 ms
    SEQ_S3_EXTEND,        // M20 → 0.95
    SEQ_S3_PAUSE,         // 500 ms
    SEQ_S4_PARTIAL,       // M21 → 0.85
    SEQ_S4_PAUSE,         // 300 ms
    SEQ_S5A_TURN_HALF,    // Tray → 195° (CW +165° von 30°)
    SEQ_S5B_TURN_END,     // Tray → 0°   (CW +165° = total +330°)
    SEQ_S5_PAUSE,         // 300 ms
    SEQ_S6_FULL,          // M21 → 0.28
    SEQ_S6_PAUSE,         // 300 ms
    SEQ_S7_JIGGLE_GROB,   // Tray ±10° um 0°, 1 s
    SEQ_S7_PAUSE,         // 200 ms
    SEQ_S8_TURN_45,       // Tray → 45°
    SEQ_S8_PAUSE,         // 300 ms
    SEQ_S9_JIGGLE_FEIN,   // Tray ±5° um 45° + M20 Jiggle, 1.5 s
    SEQ_S9_PAUSE,         // 200 ms
    SEQ_S10_END,          // M21 → UP, M20 → RETRACTED, alle disable
    SEQ_DONE              // Fertig — wartet auf Reset (Knopf 2) dann S1
};

static Servo*            g_d1   = nullptr;  // M20 horizontal
static Servo*            g_d2   = nullptr;  // M21 vertikal
static ServoFeedback360* g_tray = nullptr;  // M22 Drehteller

static SeqState m_state = SEQ_S1_HOMING;
static int      m_ctr   = 0;

// Pulsbreiten-Tracking für Debug (Servo-API hat keinen Getter)
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

// V27 Pattern: enable() um den Servo zu wecken / sofort auf Wert zu springen,
// setPulseWidth() um Wert zu ändern wenn schon enabled.
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
    g_tray->enable(0.5f);              // Bit-bang PWM aufwecken (stop pulse)
    g_tray->moveToAngle(wrapAngle(deg));
}

static const char* stateName(SeqState s)
{
    switch (s) {
        case SEQ_S1_HOMING:      return "S1_HOMING ";
        case SEQ_S1_PAUSE:       return "S1_PAUSE  ";
        case SEQ_S2_TURN_30:     return "S2_TURN30 ";
        case SEQ_S2_PAUSE:       return "S2_PAUSE  ";
        case SEQ_S3_EXTEND:      return "S3_EXTEND ";
        case SEQ_S3_PAUSE:       return "S3_PAUSE  ";
        case SEQ_S4_PARTIAL:     return "S4_PARTIAL";
        case SEQ_S4_PAUSE:       return "S4_PAUSE  ";
        case SEQ_S5A_TURN_HALF:  return "S5A_HALF  ";
        case SEQ_S5B_TURN_END:   return "S5B_END   ";
        case SEQ_S5_PAUSE:       return "S5_PAUSE  ";
        case SEQ_S6_FULL:        return "S6_FULL   ";
        case SEQ_S6_PAUSE:       return "S6_PAUSE  ";
        case SEQ_S7_JIGGLE_GROB: return "S7_JIG_GR ";
        case SEQ_S7_PAUSE:       return "S7_PAUSE  ";
        case SEQ_S8_TURN_45:     return "S8_TURN45 ";
        case SEQ_S8_PAUSE:       return "S8_PAUSE  ";
        case SEQ_S9_JIGGLE_FEIN: return "S9_JIG_FN ";
        case SEQ_S9_PAUSE:       return "S9_PAUSE  ";
        case SEQ_S10_END:        return "S10_END   ";
        case SEQ_DONE:           return "DONE      ";
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
    // 1) Hardware-Objekte (static — bleiben dauerhaft im Speicher)
    static Servo servo_d1(PC_8);     // M20 horizontal
    static Servo servo_d2(PB_2);     // M21 vertikal
    static ServoFeedback360 tray(PB_12, PC_2,
                                 SF360_KP, SF360_TOL_DEG,
                                 SF360_MIN_SPEED, SF360_OFFSET);
    g_d1   = &servo_d1;
    g_d2   = &servo_d2;
    g_tray = &tray;

    // 2) Kalibrierung (V27 exakt). KEIN setMaxAcceleration!
    g_d1->calibratePulseMinMax(D1_PULSE_MIN, D1_PULSE_MAX);
    g_d2->calibratePulseMinMax(D2_PULSE_MIN, D2_PULSE_MAX);

    // 3) Transport-Position: Arm hoch + Arm eingefahren (V27 line 743-744 Pattern)
    d1Enable(D1_RETRACTED);
    d2Enable(D2_UP);

    // 4) Warmup Parallax-Feedback (synchron, ~500 ms)
    g_tray->stop();
    int warmup_ctr = 0;
    while (warmup_ctr < WARMUP_LOOPS_INIT || !g_tray->isFeedbackValid()) {
        thread_sleep_for(20);
        g_tray->stop();
        warmup_ctr++;
        if (warmup_ctr > WARMUP_MAX_LOOPS) break;
    }

    // 5) Bit-bang PWM aus (reduziert ISR-Konkurrenz / Jitter)
    g_tray->disable();

    m_state = SEQ_S1_HOMING;
    m_ctr   = 0;

    printf("\n=== TEST_ARM_SEQUENCE ===\n");
    printf("Warmup OK | aktuelle tray-Position: %.1f° | feedback=%d\n",
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
        // SCHRITT 1: AUSRICHTEN — Tray → 0°, M21 → UP, M20 → RETRACTED
        // =====================================================================
        case SEQ_S1_HOMING: {
            if (m_ctr == 0) {
                d2Enable(D2_UP);
                d1Enable(D1_RETRACTED);
                trayMoveTo(TARGET_HOME);
                printf("\nSchritt 1: AUSRICHTEN — tray->0°, M21->%.2f, M20->%.2f\n",
                       D2_UP, D1_RETRACTED);
            }
            g_tray->update();
            m_ctr++;
            if (g_tray->isAtTarget() || m_ctr >= TURN_TIMEOUT) {
                if (!g_tray->isAtTarget())
                    printf("  TIMEOUT bei %.1f°\n", g_tray->getCurrentAngle());
                printf("Schritt 1: Ausgerichtet auf 0-Stellung (Ist: %.1f°)\n",
                       g_tray->getCurrentAngle());
                g_tray->stop();
                m_ctr   = 0;
                m_state = SEQ_S1_PAUSE;
            }
            break;
        }

        case SEQ_S1_PAUSE:
            m_ctr++;
            if (m_ctr >= PAUSE_500_LOOPS) {
                m_ctr   = 0;
                m_state = SEQ_S2_TURN_30;
            }
            break;

        // =====================================================================
        // SCHRITT 2: DREHTELLER +30° NACH VORNE
        // =====================================================================
        case SEQ_S2_TURN_30: {
            if (m_ctr == 0) {
                trayMoveTo(TARGET_STEP2);
                printf("\nSchritt 2: Drehteller -> %.1f° (CW +30°)\n", TARGET_STEP2);
            }
            g_tray->update();
            m_ctr++;
            if (g_tray->isAtTarget() || m_ctr >= TURN_TIMEOUT) {
                if (!g_tray->isAtTarget())
                    printf("  TIMEOUT bei %.1f°\n", g_tray->getCurrentAngle());
                printf("Schritt 2: Drehteller auf 30°, Ist: %.1f°\n",
                       g_tray->getCurrentAngle());
                g_tray->stop();
                m_ctr   = 0;
                m_state = SEQ_S2_PAUSE;
            }
            break;
        }

        case SEQ_S2_PAUSE:
            m_ctr++;
            if (m_ctr >= PAUSE_300_LOOPS) {
                m_ctr   = 0;
                m_state = SEQ_S3_EXTEND;
            }
            break;

        // =====================================================================
        // SCHRITT 3: HORIZONTALEN ARM AUSFAHREN — V27 Pattern: enable() jumpt
        // =====================================================================
        case SEQ_S3_EXTEND:
            if (m_ctr == 0) {
                d1Enable(D1_EXTENDED);
                printf("\nSchritt 3: Horizontaler Arm ausgefahren (M20=%.2f)\n",
                       D1_EXTENDED);
                printDebug("S3");
            }
            m_ctr++;
            if (m_ctr >= 2) {  // 1 Loop reicht — Servo springt instantan (kein setMaxAccel)
                m_ctr   = 0;
                m_state = SEQ_S3_PAUSE;
            }
            break;

        case SEQ_S3_PAUSE:
            m_ctr++;
            if (m_ctr >= PAUSE_500_LOOPS) {
                m_ctr   = 0;
                m_state = SEQ_S4_PARTIAL;
            }
            break;

        // =====================================================================
        // SCHRITT 4: VERTIKALEN ARM 10mm RUNTER
        // =====================================================================
        case SEQ_S4_PARTIAL:
            if (m_ctr == 0) {
                d2Set(D2_PARTIAL_DOWN);
                printf("\nSchritt 4: Vertikaler Arm 10mm abgesenkt (M21=%.2f)\n",
                       D2_PARTIAL_DOWN);
                printDebug("S4");
            }
            m_ctr++;
            if (m_ctr >= 2) {
                m_ctr   = 0;
                m_state = SEQ_S4_PAUSE;
            }
            break;

        case SEQ_S4_PAUSE:
            m_ctr++;
            if (m_ctr >= PAUSE_300_LOOPS) {
                m_ctr   = 0;
                m_state = SEQ_S5A_TURN_HALF;
            }
            break;

        // =====================================================================
        // SCHRITT 5: DREHTELLER +330° CW ZURÜCK AUF 0°
        //   — als 2 Substeps à +165° um 180°-Ambiguität zu vermeiden
        // =====================================================================
        case SEQ_S5A_TURN_HALF: {
            if (m_ctr == 0) {
                trayMoveTo(STEP5_HALF1);
                printf("\nSchritt 5a: Drehteller -> %.1f° (CW +165°)\n", STEP5_HALF1);
            }
            g_tray->update();
            m_ctr++;
            if (g_tray->isAtTarget() || m_ctr >= TURN_TIMEOUT) {
                printf("  5a Ist: %.1f°\n", g_tray->getCurrentAngle());
                m_ctr   = 0;
                m_state = SEQ_S5B_TURN_END;
            }
            break;
        }

        case SEQ_S5B_TURN_END: {
            if (m_ctr == 0) {
                trayMoveTo(TARGET_STEP5);
                printf("Schritt 5b: Drehteller -> %.1f° (CW +165° = total +330°)\n",
                       TARGET_STEP5);
            }
            g_tray->update();
            m_ctr++;
            if (g_tray->isAtTarget() || m_ctr >= TURN_TIMEOUT) {
                if (!g_tray->isAtTarget())
                    printf("  TIMEOUT bei %.1f°\n", g_tray->getCurrentAngle());
                printf("Schritt 5: Drehteller zurück auf 0°, Ist: %.1f°\n",
                       g_tray->getCurrentAngle());
                g_tray->stop();
                m_ctr   = 0;
                m_state = SEQ_S5_PAUSE;
            }
            break;
        }

        case SEQ_S5_PAUSE:
            m_ctr++;
            if (m_ctr >= PAUSE_300_LOOPS) {
                m_ctr   = 0;
                m_state = SEQ_S6_FULL;
            }
            break;

        // =====================================================================
        // SCHRITT 6: VERTIKALEN ARM KOMPLETT RUNTER (BLAU-Tiefe)
        // =====================================================================
        case SEQ_S6_FULL:
            if (m_ctr == 0) {
                d2Set(D2_FULL_DOWN);
                printf("\nSchritt 6: Vertikaler Arm voll abgesenkt (M21=%.2f)\n",
                       D2_FULL_DOWN);
                printDebug("S6");
            }
            m_ctr++;
            if (m_ctr >= 2) {
                m_ctr   = 0;
                m_state = SEQ_S6_PAUSE;
            }
            break;

        case SEQ_S6_PAUSE:
            m_ctr++;
            if (m_ctr >= PAUSE_300_LOOPS) {
                m_ctr   = 0;
                m_state = SEQ_S7_JIGGLE_GROB;
            }
            break;

        // =====================================================================
        // SCHRITT 7: GROBER JIGGLE — Drehteller 0° → -10° → +10° → 0° (1 s)
        // =====================================================================
        case SEQ_S7_JIGGLE_GROB:
            if (m_ctr == 0) {
                trayMoveTo(wrapAngle(-JIGGLE_GROB_AMPL));   // -10° = 350°
                printf("\nSchritt 7: Grober Jiggle ±10° (1 s)\n");
            } else if (m_ctr == JIGGLE_GROB_PH1) {
                trayMoveTo(JIGGLE_GROB_AMPL);                // +10°
            } else if (m_ctr == JIGGLE_GROB_PH2) {
                trayMoveTo(0.0f);                            // zurück auf 0°
            }
            g_tray->update();
            m_ctr++;
            if (m_ctr >= JIGGLE_GROB_TOTAL) {
                printf("Schritt 7: Jiggle fertig, Ist: %.1f°\n",
                       g_tray->getCurrentAngle());
                g_tray->stop();
                m_ctr   = 0;
                m_state = SEQ_S7_PAUSE;
            }
            break;

        case SEQ_S7_PAUSE:
            m_ctr++;
            if (m_ctr >= PAUSE_200_LOOPS) {
                m_ctr   = 0;
                m_state = SEQ_S8_TURN_45;
            }
            break;

        // =====================================================================
        // SCHRITT 8: DREHTELLER AUF 45°
        // =====================================================================
        case SEQ_S8_TURN_45: {
            if (m_ctr == 0) {
                trayMoveTo(TARGET_STEP8);
                printf("\nSchritt 8: Drehteller -> 45° (CW)\n");
            }
            g_tray->update();
            m_ctr++;
            if (g_tray->isAtTarget() || m_ctr >= TURN_TIMEOUT) {
                if (!g_tray->isAtTarget())
                    printf("  TIMEOUT bei %.1f°\n", g_tray->getCurrentAngle());
                printf("Schritt 8: Drehteller auf 45°, Ist: %.1f°\n",
                       g_tray->getCurrentAngle());
                g_tray->stop();
                m_ctr   = 0;
                m_state = SEQ_S8_PAUSE;
            }
            break;
        }

        case SEQ_S8_PAUSE:
            m_ctr++;
            if (m_ctr >= PAUSE_300_LOOPS) {
                m_ctr   = 0;
                m_state = SEQ_S9_JIGGLE_FEIN;
            }
            break;

        // =====================================================================
        // SCHRITT 9: FEINER JIGGLE — Tray ±5° um 45° + M20 Jiggle (1.5 s)
        //   Phasen: 40° + vor → 50° + zurück → 45° + mitte
        // =====================================================================
        case SEQ_S9_JIGGLE_FEIN:
            if (m_ctr == 0) {
                trayMoveTo(TARGET_STEP8 - JIGGLE_FEIN_AMPL);   // 40°
                d1Set(D1_EXTENDED + D1_JIGGLE_OFFSET);          // vor
                printf("\nSchritt 9: Feiner 2D-Jiggle (1.5 s)\n");
                printf("  A: tray->40°, M20->%.2f (vor)\n",
                       D1_EXTENDED + D1_JIGGLE_OFFSET);
            } else if (m_ctr == JIGGLE_FEIN_PH1) {
                trayMoveTo(TARGET_STEP8 + JIGGLE_FEIN_AMPL);    // 50°
                d1Set(D1_EXTENDED - D1_JIGGLE_OFFSET);          // zurück
                printf("  B: tray->50°, M20->%.2f (zurueck)\n",
                       D1_EXTENDED - D1_JIGGLE_OFFSET);
            } else if (m_ctr == JIGGLE_FEIN_PH2) {
                trayMoveTo(TARGET_STEP8);                       // 45°
                d1Set(D1_EXTENDED);                             // mitte
                printf("  C: tray->45°, M20->%.2f (mitte)\n", D1_EXTENDED);
            }
            g_tray->update();
            m_ctr++;
            if (m_ctr >= JIGGLE_FEIN_TOTAL) {
                printf("Schritt 9: Feiner Jiggle fertig, Ist: %.1f°\n",
                       g_tray->getCurrentAngle());
                g_tray->stop();
                m_ctr   = 0;
                m_state = SEQ_S9_PAUSE;
            }
            break;

        case SEQ_S9_PAUSE:
            m_ctr++;
            if (m_ctr >= PAUSE_200_LOOPS) {
                m_ctr   = 0;
                m_state = SEQ_S10_END;
            }
            break;

        // =====================================================================
        // SCHRITT 10: ARM HOCH — Sequenz fertig
        // =====================================================================
        case SEQ_S10_END:
            d2Set(D2_UP);
            d1Set(D1_RETRACTED);
            printf("\nSchritt 10: Arm hoch. Sequenz abgeschlossen.\n");
            printDebug("S10");

            thread_sleep_for(500);  // Servos die Endposition erreichen lassen

            g_d1->disable();
            g_d2->disable();
            g_tray->disable();
            led = 0;

            printf("Fertig. Knopf 2x druecken um Sequenz neu zu starten (Stop + Start).\n");
            m_state = SEQ_DONE;
            break;

        case SEQ_DONE:
            // Idle — wartet auf Knopf 2 (Stop via _reset) + Knopf 3 (Start von S1)
            break;
    }
}

// =============================================================================
// _reset() — Sequenz stoppen und auf S1 zurücksetzen
//   Wird von main.cpp aufgerufen wenn Knopf gedrückt (gerade paar = Stop).
//   Nächster Knopfdruck (ungerade = Start) ruft _task() wieder auf → S1.
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
