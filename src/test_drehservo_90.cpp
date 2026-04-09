#include "test_config.h"

#ifdef TEST_DREHSERVO_90

#include "test_drehservo_90.h"
#include "Servo.h"

// ---------------------------------------------------------------------------
// Kalibrierung:
//   1. Flashen, Button drücken → Servo macht 4 × 90° mit Pause dazwischen
//   2. Beobachten ob er nach 4 Schritten exakt wieder an Startposition ist
//   3. Falls zu weit:  TIME_FOR_90_MS verkleinern
//      Falls zu wenig: TIME_FOR_90_MS vergrössern
//   Präzision: ±20 ms (= 1 Loop bei 50 Hz, Servo-PWM-Periode)
// ---------------------------------------------------------------------------
static const float PULSE_360_MIN  = 0.0303f; // kalibriert 2026-03-26
static const float PULSE_360_MAX  = 0.1223f;
static const float SERVO_CW       = 0.25f;   // CW Drehung (wie roboter_v6)
static const float SERVO_STOP     = 0.50f;   // Stopp (Mittelpunkt)

static const int   TIME_FOR_90_MS = 140;     // ← HIER ANPASSEN (in Millisekunden)
static const int   PAUSE_MS       = 1000;    // 1 s Pause zwischen Schritten
static const int   REPS           = 4;       // 4 × 90° = 360° → muss wieder an Start

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
enum DS90State { DS90_IDLE, DS90_RUNNING, DS90_PAUSE, DS90_DONE };

static Servo*    g_servo  = nullptr;
static DS90State m_state  = DS90_IDLE;
static int       m_rep    = 0;
static Timer     m_timer;

void drehservo_90_init(int loops_per_second)
{
    static Servo servo_360(PB_D3);
    g_servo = &servo_360;
    g_servo->calibratePulseMinMax(PULSE_360_MIN, PULSE_360_MAX);

    m_state = DS90_IDLE;
    m_rep   = 0;
    m_timer.stop();
    m_timer.reset();
}

void drehservo_90_task(DigitalOut& led)
{
    led = !led;

    switch (m_state) {

        case DS90_IDLE:
            if (!g_servo->isEnabled()) g_servo->enable();
            g_servo->setPulseWidth(SERVO_STOP);
            m_rep = 0;
            m_rep++;
            m_timer.reset();
            m_timer.start();
            m_state = DS90_RUNNING;
            break;

        case DS90_RUNNING:
            g_servo->setPulseWidth(SERVO_CW);
            if (std::chrono::duration_cast<std::chrono::milliseconds>(m_timer.elapsed_time()).count() >= TIME_FOR_90_MS) {
                g_servo->setPulseWidth(SERVO_STOP);
                m_timer.stop();
                m_timer.reset();
                if (m_rep >= REPS) {
                    m_state = DS90_DONE;
                } else {
                    m_timer.start();
                    m_state = DS90_PAUSE;
                }
            }
            break;

        case DS90_PAUSE:
            g_servo->setPulseWidth(SERVO_STOP);
            if (std::chrono::duration_cast<std::chrono::milliseconds>(m_timer.elapsed_time()).count() >= PAUSE_MS) {
                m_rep++;
                m_timer.reset();
                m_timer.start();
                m_state = DS90_RUNNING;
            }
            break;

        case DS90_DONE:
            g_servo->setPulseWidth(SERVO_STOP);
            break;
    }
}

void drehservo_90_reset(DigitalOut& led)
{
    if (g_servo) {
        g_servo->setPulseWidth(SERVO_STOP);
        g_servo->disable();
    }
    m_timer.stop();
    m_timer.reset();
    m_state = DS90_IDLE;
    m_rep   = 0;
    led     = 0;
}

void drehservo_90_print()
{
    int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(m_timer.elapsed_time()).count();
    const char* s = (m_state == DS90_IDLE)    ? "IDLE   " :
                    (m_state == DS90_RUNNING)  ? "RUN    " :
                    (m_state == DS90_PAUSE)    ? "PAUSE  " : "DONE   ";
    printf("State: %s | rep=%d/%d | elapsed=%4d ms | TIME_FOR_90_MS=%d ms\n",
           s, m_rep, REPS, elapsed, TIME_FOR_90_MS);
}

#endif // TEST_DREHSERVO_90
