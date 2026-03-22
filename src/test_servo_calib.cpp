#include "test_config.h"

#ifdef TEST_SERVO_CALIB

#include "test_servo_calib.h"

// ---------------------------------------------------------------------------
// Servo objects
// ---------------------------------------------------------------------------
static Servo* g_servo_a   = nullptr; // 180° Servo A — PB_D1 (PC_8)
static Servo* g_servo_b   = nullptr; // 180° Servo B — PB_D2 (PC_6)
static Servo* g_servo_360 = nullptr; // 360° Servo   — PB_D3 (PB_12)

// ---------------------------------------------------------------------------
// Wizard state
// ---------------------------------------------------------------------------
static int   g_phase          = 0;       // 0..6  (6 = done)
static float g_cur_pulse      = 0.050f;  // currently commanded pulse
static bool  g_fast_mode      = false;   // step size: false=0.001f  true=0.005f
static bool  g_phase5_cw_done = false;   // sub-step inside phase 5

// Calibration results (intentionally survive _reset so partial work is kept)
static float g_min_a    = 0.050f;
static float g_max_a    = 0.105f;
static float g_min_b    = 0.050f;
static float g_max_b    = 0.105f;
static float g_stop_360 = 0.075f;
static float g_min_360  = 0.050f;
static float g_max_360  = 0.105f;

// Validity flags — set to true once a value has been confirmed by the user
static bool g_valid_min_a    = false;
static bool g_valid_max_a    = false;
static bool g_valid_min_b    = false;
static bool g_valid_max_b    = false;
static bool g_valid_stop_360 = false;
static bool g_valid_min_360  = false;
static bool g_valid_max_360  = false;

// ---------------------------------------------------------------------------
// Print throttle
// ---------------------------------------------------------------------------
static int  g_print_ctr   = 0;
static bool g_first_print = true;
static const int PRINT_EVERY_N  = 25; // ~500 ms at 50 Hz
static const int DISPLAY_LINES  = 14; // must match exact line count in _print()

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static void clamp_pulse(float& p)
{
    if (p < 0.01f) p = 0.01f;
    if (p > 0.99f) p = 0.99f;
}

// Enable only the servo that belongs to the current phase, disable the others.
static void activate_servo(int phase)
{
    if (phase >= 6) {
        if (g_servo_a->isEnabled())   g_servo_a->disable();
        if (g_servo_b->isEnabled())   g_servo_b->disable();
        if (g_servo_360->isEnabled()) g_servo_360->disable();
        return;
    }
    if (phase <= 1) {
        if (!g_servo_a->isEnabled())   g_servo_a->enable();
        if (g_servo_b->isEnabled())    g_servo_b->disable();
        if (g_servo_360->isEnabled())  g_servo_360->disable();
    } else if (phase <= 3) {
        if (g_servo_a->isEnabled())    g_servo_a->disable();
        if (!g_servo_b->isEnabled())   g_servo_b->enable();
        if (g_servo_360->isEnabled())  g_servo_360->disable();
    } else {
        if (g_servo_a->isEnabled())    g_servo_a->disable();
        if (g_servo_b->isEnabled())    g_servo_b->disable();
        if (!g_servo_360->isEnabled()) g_servo_360->enable();
    }
}

// Returns the pulse value to start with when entering a new phase.
static float start_pulse_for_phase(int phase, bool phase5_cw_done)
{
    switch (phase) {
        case 0: return 0.050f;
        case 1: return g_min_a;       // continue from where MIN was found
        case 2: return 0.050f;
        case 3: return g_min_b;
        case 4: return 0.075f;        // midpoint — servo probably turns slowly
        case 5: return phase5_cw_done ? g_stop_360 : 0.050f;
        default: return 0.050f;
    }
}

// Advance to the next phase and store the confirmed value.
static void confirm_phase()
{
    switch (g_phase) {
        case 0:
            g_min_a       = g_cur_pulse;
            g_valid_min_a = true;
            g_phase       = 1;
            g_cur_pulse   = start_pulse_for_phase(1, false);
            break;

        case 1:
            g_max_a       = g_cur_pulse;
            g_valid_max_a = true;
            g_servo_a->calibratePulseMinMax(g_min_a, g_max_a);
            g_servo_a->setMaxAcceleration(0.3f);
            g_phase     = 2;
            g_cur_pulse = start_pulse_for_phase(2, false);
            break;

        case 2:
            g_min_b       = g_cur_pulse;
            g_valid_min_b = true;
            g_phase       = 3;
            g_cur_pulse   = start_pulse_for_phase(3, false);
            break;

        case 3:
            g_max_b       = g_cur_pulse;
            g_valid_max_b = true;
            g_servo_b->calibratePulseMinMax(g_min_b, g_max_b);
            g_servo_b->setMaxAcceleration(0.3f);
            g_phase     = 4;
            g_cur_pulse = start_pulse_for_phase(4, false);
            break;

        case 4:
            g_stop_360       = g_cur_pulse;
            g_valid_stop_360 = true;
            g_phase          = 5;
            g_phase5_cw_done = false;
            g_cur_pulse      = start_pulse_for_phase(5, false);
            break;

        case 5:
            if (!g_phase5_cw_done) {
                g_min_360        = g_cur_pulse;
                g_valid_min_360  = true;
                g_phase5_cw_done = true;
                g_cur_pulse      = start_pulse_for_phase(5, true);
            } else {
                g_max_360       = g_cur_pulse;
                g_valid_max_360 = true;
                g_servo_360->calibratePulseMinMax(g_min_360, g_max_360);
                g_servo_360->disable();
                g_phase = 6;
            }
            break;

        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// Phase label for display
// ---------------------------------------------------------------------------
static const char* phase_label()
{
    switch (g_phase) {
        case 0: return "Servo A  ->  MIN-Anschlag (0 Grad)";
        case 1: return "Servo A  ->  MAX-Anschlag (180 Grad)";
        case 2: return "Servo B  ->  MIN-Anschlag (0 Grad)";
        case 3: return "Servo B  ->  MAX-Anschlag (180 Grad)";
        case 4: return "360 Servo  ->  Stopp-Position finden";
        case 5: return g_phase5_cw_done ? "360 Servo  ->  CCW-Limit finden"
                                        : "360 Servo  ->  CW-Limit finden";
        default: return "Kalibrierung abgeschlossen";
    }
}

static const char* active_servo_name()
{
    if (g_phase <= 1) return "Servo A (PB_D1 / PC_8)";
    if (g_phase <= 3) return "Servo B (PB_D2 / PC_6)";
    if (g_phase <= 5) return "360 Servo (PB_D3 / PB_12)";
    return "---";
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void servo_calib_init(int loops_per_second)
{
    (void)loops_per_second;

    // Create servo objects exactly once (static — no heap)
    static Servo servo_a(PB_D1);
    static Servo servo_b(PB_D2);
    static Servo servo_360(PB_D3);
    g_servo_a   = &servo_a;
    g_servo_b   = &servo_b;
    g_servo_360 = &servo_360;

    // Enable non-blocking stdin so getchar() returns EOF when no key pressed
    auto* fh = mbed::mbed_file_handle(STDIN_FILENO);
    if (fh) fh->set_blocking(false);

    // Reset wizard state (validity flags cleared — fresh calibration)
    g_phase          = 0;
    g_cur_pulse      = 0.050f;
    g_fast_mode      = false;
    g_phase5_cw_done = false;
    g_valid_min_a    = false;
    g_valid_max_a    = false;
    g_valid_min_b    = false;
    g_valid_max_b    = false;
    g_valid_stop_360 = false;
    g_valid_min_360  = false;
    g_valid_max_360  = false;
    g_print_ctr      = 0;
    g_first_print    = true;

    // Only Servo A is active at the start (phase 0)
    activate_servo(0);
    g_servo_a->setPulseWidth(g_cur_pulse);
}

void servo_calib_task(DigitalOut& led1)
{
    led1 = 1;

    if (g_phase >= 6) return; // nothing left to do

    // Non-blocking serial read — at most one character per 20 ms tick
    int ch = getchar();
    if (ch != EOF) {
        float step = g_fast_mode ? 0.005f : 0.001f;
        switch (ch) {
            case '+':
                g_cur_pulse += step;
                clamp_pulse(g_cur_pulse);
                break;
            case '-':
                g_cur_pulse -= step;
                clamp_pulse(g_cur_pulse);
                break;
            case 'f':
                g_fast_mode = !g_fast_mode;
                break;
            case 'c':
                confirm_phase();
                activate_servo(g_phase);
                break;
            default:
                break;
        }
    }

    // Drive the currently active servo
    switch (g_phase) {
        case 0: case 1: g_servo_a->setPulseWidth(g_cur_pulse);   break;
        case 2: case 3: g_servo_b->setPulseWidth(g_cur_pulse);   break;
        case 4: case 5: g_servo_360->setPulseWidth(g_cur_pulse); break;
        default: break;
    }
}

void servo_calib_reset(DigitalOut& led1)
{
    led1 = 0;
    if (g_servo_a)   g_servo_a->disable();
    if (g_servo_b)   g_servo_b->disable();
    if (g_servo_360) g_servo_360->disable();

    g_phase          = 0;
    g_cur_pulse      = 0.050f;
    g_fast_mode      = false;
    g_phase5_cw_done = false;
    g_first_print    = true;
    g_print_ctr      = 0;
    // Calibration results are intentionally kept so partial work survives stop/start
}

void servo_calib_print()
{
    g_print_ctr++;
    if (g_print_ctr < PRINT_EVERY_N) return;
    g_print_ctr = 0;

    // Rewind cursor to overwrite previous output block
    if (!g_first_print)
        printf("\033[%dA", DISPLAY_LINES);
    g_first_print = false;

    if (g_phase < 6) {
        // --- Active calibration display (14 lines) ---
        printf("==== Servo Kalibrierung  Phase %d/6 ===================\n", g_phase + 1);
        printf(" [+] feiner   [-] kleiner   [f] Schritt   [c] bestaetigen\n");
        printf("------------------------------------------------------\n");
        printf(" Phase: %-38s\n", phase_label());
        printf(" Aktiver Servo: %-36s\n", active_servo_name());
        printf(" Schrittgroesse: %s              \n",
               g_fast_mode ? "0.005 (SCHNELL)" : "0.001 (fein)   ");
        printf(" Aktueller Pulse: %.4f                          \n", g_cur_pulse);
        printf("------------------------------------------------------\n");
        printf(" Gespeicherte Werte:\n");
        // Servo A
        printf("   Servo A:  min=");
        if (g_valid_min_a) printf("%.4f", g_min_a); else printf("----  ");
        printf("  max=");
        if (g_valid_max_a) printf("%.4f\n", g_max_a); else printf("----\n");
        // Servo B
        printf("   Servo B:  min=");
        if (g_valid_min_b) printf("%.4f", g_min_b); else printf("----  ");
        printf("  max=");
        if (g_valid_max_b) printf("%.4f\n", g_max_b); else printf("----\n");
        // 360 servo
        printf("   360 Grad: stp=");
        if (g_valid_stop_360) printf("%.4f", g_stop_360); else printf("----  ");
        printf("  CW=");
        if (g_valid_min_360) printf("%.4f", g_min_360); else printf("----  ");
        printf("  CCW=");
        if (g_valid_max_360) printf("%.4f\n", g_max_360); else printf("----\n");
        printf("------------------------------------------------------\n");
        printf(" [AKTIV] Servo fahren und [c] druecken zum Bestaetigen \n");
    } else {
        // --- Summary display (14 lines) ---
        printf("==== Kalibrierung fertig! Werte in test_servo_all.cpp =\n");
        printf(" Folgende Zeilen kopieren:                             \n");
        printf("------------------------------------------------------\n");
        printf("   servo_a.calibratePulseMinMax(%.4ff, %.4ff);\n", g_min_a, g_max_a);
        printf("   servo_a.setMaxAcceleration(0.3f);                  \n");
        printf("   servo_b.calibratePulseMinMax(%.4ff, %.4ff);\n", g_min_b, g_max_b);
        printf("   servo_b.setMaxAcceleration(0.3f);                  \n");
        printf("   servo_360.calibratePulseMinMax(%.4ff, %.4ff);\n", g_min_360, g_max_360);
        printf("   // 360 Stopp-Position (setPulseWidth(0.5f)): %.4f\n", g_stop_360);
        printf("------------------------------------------------------\n");
        printf(" Servo A:  MIN=%.4f  MAX=%.4f              \n", g_min_a, g_max_a);
        printf(" Servo B:  MIN=%.4f  MAX=%.4f              \n", g_min_b, g_max_b);
        printf(" 360 Grad: CW=%.4f  STOP=%.4f  CCW=%.4f\n",
               g_min_360, g_stop_360, g_max_360);
        printf("======================================================\n");
    }
}

#endif // TEST_SERVO_CALIB
