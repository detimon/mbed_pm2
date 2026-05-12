// +==============================================================+
// |        INBETRIEBNAHMEPROTOKOLL - LineXpress FS26             |
// +==============================================================+
// | Datei      : calib_servo_180.cpp
// | Komponente : 180-Grad-Servo Pulsweiten-Kalibrierung
// | BMK        : M20 / M21
// | Pin(s)     : PB_D3 = PB_12 (im Test als Stellvertreter)
// +--------------------------------------------------------------+
// | TESTZIEL   : Untere und obere Pulse-Grenze (PULSE_MIN/MAX) durch Sweep ermitteln.
// | VORAUSSETZ.: Servo angeschlossen, Mechanik nicht montiert (freier Lauf).
// +--------------------------------------------------------------+
// | ERWARTETES VERHALTEN:
// |   Sweep nach oben/unten zeigt mechanischen Anschlag — Pulse-Wert in calibration_values.h eintragen.
// +--------------------------------------------------------------+
// | TESTDATUM  : __.__.____ | TESTER: _______________
// | ERGEBNIS   : [ ] Bestanden   [ ] Nicht bestanden
// | BEMERKUNGEN: ______________________________________________
// +==============================================================+

#include "test_config.h"

#ifdef CALIB_SERVO_180

#include "calib_servo_180.h"
#include "../calibration/calibration_values.h"

static Servo* g_servo_b = nullptr;

// ---------------------------------------------------------------------------
// Sweep state machine
// ---------------------------------------------------------------------------
enum CalibState {
    CS_GOTO_LOW,   // servo schnell auf 0.01f fahren
    CS_WAIT_LOW,   // 1s warten am unteren Anschlag
    CS_SWEEP_UP,   // langsam nach oben -> MAX-Stotter beobachten
    CS_GOTO_HIGH,  // servo schnell auf 0.15f fahren
    CS_WAIT_HIGH,  // 1s warten am oberen Anschlag
    CS_SWEEP_DOWN, // langsam nach unten -> MIN-Stotter beobachten
    CS_DONE
};

static CalibState g_state     = CS_GOTO_LOW;
static float      g_pulse     = SERVO_180_PULSE_LOW;
static int        g_wait_ctr  = 0;
static int        g_sweep_ctr = 0;

// ---------------------------------------------------------------------------
// Print throttle
// ---------------------------------------------------------------------------
static int  g_print_ctr   = 0;
static bool g_first_print = true;
// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static const char* state_label()
{
    switch (g_state) {
        case CS_GOTO_LOW:   return "Fahre zu unterem Anschlag ...";
        case CS_WAIT_LOW:   return "Warte 1s am unteren Anschlag ...";
        case CS_SWEEP_UP:   return ">>> Sweep nach OBEN  (MAX-Anschlag beobachten) <<<";
        case CS_GOTO_HIGH:  return "Fahre zu oberem Anschlag ...";
        case CS_WAIT_HIGH:  return "Warte 1s am oberen Anschlag ...";
        case CS_SWEEP_DOWN: return ">>> Sweep nach UNTEN (MIN-Anschlag beobachten) <<<";
        default:            return "Fertig — Werte oben ablesen und weitersagen!";
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void calib_servo_180_init(int loops_per_second)
{
    (void)loops_per_second;

    static Servo servo_b(PB_D3);
    g_servo_b = &servo_b;

    auto* fh = mbed::mbed_file_handle(STDIN_FILENO);
    if (fh) fh->set_blocking(false);

    g_state       = CS_GOTO_LOW;
    g_pulse       = SERVO_180_PULSE_LOW;
    g_wait_ctr    = 0;
    g_sweep_ctr   = 0;
    g_print_ctr   = 0;
    g_first_print = true;

    g_servo_b->enable();
    g_servo_b->setPulseWidth(g_pulse);
}

void calib_servo_180_task(DigitalOut& led1)
{
    led1 = !led1;

    switch (g_state) {
        case CS_GOTO_LOW:
            g_pulse = SERVO_180_PULSE_LOW;
            g_servo_b->setPulseWidth(g_pulse);
            g_wait_ctr++;
            if (g_wait_ctr >= SERVO_180_GOTO_LOOPS) { // ~200ms, damit Servo ankommt
                g_wait_ctr = 0;
                g_state    = CS_WAIT_LOW;
            }
            break;

        case CS_WAIT_LOW:
            g_wait_ctr++;
            if (g_wait_ctr >= SERVO_180_WAIT_LOOPS) {
                g_wait_ctr  = 0;
                g_sweep_ctr = 0;
                g_state     = CS_SWEEP_UP;
            }
            break;

        case CS_SWEEP_UP:
            g_sweep_ctr++;
            if (g_sweep_ctr >= SERVO_180_SWEEP_DIV) {
                g_sweep_ctr = 0;
                g_pulse += SERVO_180_SWEEP_STEP;
                if (g_pulse > SERVO_180_PULSE_HIGH) g_pulse = SERVO_180_PULSE_HIGH;
                g_servo_b->setPulseWidth(g_pulse);
            }
            if (g_pulse >= SERVO_180_PULSE_HIGH) {
                g_wait_ctr = 0;
                g_state    = CS_GOTO_HIGH;
            }
            break;

        case CS_GOTO_HIGH:
            g_pulse = SERVO_180_PULSE_HIGH;
            g_servo_b->setPulseWidth(g_pulse);
            g_wait_ctr++;
            if (g_wait_ctr >= SERVO_180_GOTO_LOOPS) {
                g_wait_ctr = 0;
                g_state    = CS_WAIT_HIGH;
            }
            break;

        case CS_WAIT_HIGH:
            g_wait_ctr++;
            if (g_wait_ctr >= SERVO_180_WAIT_LOOPS) {
                g_wait_ctr  = 0;
                g_sweep_ctr = 0;
                g_state     = CS_SWEEP_DOWN;
            }
            break;

        case CS_SWEEP_DOWN:
            g_sweep_ctr++;
            if (g_sweep_ctr >= SERVO_180_SWEEP_DIV) {
                g_sweep_ctr = 0;
                g_pulse -= SERVO_180_SWEEP_STEP;
                if (g_pulse < SERVO_180_PULSE_LOW) g_pulse = SERVO_180_PULSE_LOW;
                g_servo_b->setPulseWidth(g_pulse);
            }
            if (g_pulse <= SERVO_180_PULSE_LOW) {
                g_state = CS_DONE;
            }
            break;

        case CS_DONE:
            break;
    }
}

void calib_servo_180_reset(DigitalOut& led1)
{
    led1 = 0;
    if (g_servo_b) g_servo_b->disable();

    g_state       = CS_GOTO_LOW;
    g_pulse       = SERVO_180_PULSE_LOW;
    g_wait_ctr    = 0;
    g_sweep_ctr   = 0;
    g_first_print = true;
    g_print_ctr   = 0;
}

void calib_servo_180_print()
{
    g_print_ctr++;
    if (g_print_ctr < SERVO_180_PRINT_EVERY_N) return;
    g_print_ctr = 0;

    if (!g_first_print)
        printf("\033[%dA", SERVO_180_DISPLAY_LINES);
    g_first_print = false;

    printf("==== Servo B (D2 / PC_6) Kalibrierung ================\n");
    printf(" Status: %-44s\n", state_label());
    printf("------------------------------------------------------\n");
    printf("  >> Aktueller Pulse: %.4f <<                     \n", g_pulse);
    printf("------------------------------------------------------\n");
    printf(" Sweep UP   -> wenn Servo stockt: das ist MAX        \n");
    printf(" Sweep DOWN -> wenn Servo stockt: das ist MIN        \n");
    printf("------------------------------------------------------\n");
    printf(" Wert notieren und Claude weitersagen!                \n");
    printf("======================================================\n");
}

#endif // CALIB_SERVO_180
