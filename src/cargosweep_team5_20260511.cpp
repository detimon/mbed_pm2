// CargoSweep — Team 5 | 2026-05-11
// Refactored from cargosweep_final_version_05.cpp
// States: 20 → 15 (merged alignment + crossing stop states, renamed for clarity)
// Logic and timing: UNCHANGED from Version 05.
#include "test_config.h"

#ifdef CARGOSWEEP_TEAM5_20260511

#include "cargosweep_team5_20260511.h"
#include <cmath>
#include "ColorSensor.h"
#include "DCMotor.h"
#include "LineFollower.h"
#include "NeoPixel.h"
#include "Servo.h"
#include "ServoFeedback360.h"
#include "PESBoardPinMap.h"

// ===========================================================================
// Motor & Geometrie
// ===========================================================================
static const float GEAR_RATIO  = 100.0f;               // 100:1 Untersetzungsgetriebe
static const float KN          = 140.0f / 12.0f;       // Motorkenngrösse rpm/V
static const float VOLTAGE_MAX = 12.0f;                 // Maximale Versorgungsspannung [V]
static const float VEL_SIGN    = 1.0f;                  // Vorzeichenkorrektur nach physischem Motortausch

static const float D_WHEEL  = 0.0291f;                  // Raddurchmesser [m]
static const float B_WHEEL  = 0.1493f;                  // Radabstand (Achsbreite) [m]
static const float BAR_DIST = 0.182f;                   // Abstand zwischen Querbalken [m]

// ===========================================================================
// Linienfolger PID
// ===========================================================================
static const float KP           = 2.8f;                 // P-Gain Hauptprogramm (schnell, präzise)
static const float KP_NL        = 4.55f;                // Nichtlinearer Gain Hauptprogramm
static const float KP_FOLLOW    = 1.1f;                 // P-Gain Ausrichtungsphase (sanft)
static const float KP_NL_FOLLOW = 0.7f;                 // Nichtlinearer Gain Ausrichtungsphase

// Tunnel: 3-Phasen-Sequenz (A: 2 s Linefollower, B: 1.5 s Gerade, C: Linefollower)
static const int TUNNEL_FOLLOW_LOOPS   = 100;   // Phase A: 2 s line-following before straight section (100 × 20 ms)
static const int TUNNEL_STRAIGHT_LOOPS = 75;    // Phase B: 1.5 s straight through tunnel where line disappears

static const float MAX_SPEED    = 1.6f;                 // Maximale Radgeschwindigkeit [RPS]
static const float SPEED_SCALE  = 1.0f / MAX_SPEED;     // Skalierungsfaktor für zeitbasierte Guards

static const float BLIND_SPEED    = 1.0f;               // Geschwindigkeit im Blindanfahrts-State [RPS]
static const float STRAIGHT_SPEED = 1.0f;               // Geradeaus-Geschwindigkeit nach Ausrichtung [RPS]
static const float TURN_SPEED     = 0.5f;               // Drehgeschwindigkeit auf der Stelle [RPS]
static const float BACKWARD_SPEED = 1.0f;               // Rückwärtsfahrt-Geschwindigkeit [RPS]
static const float APPROACH_SPEED = 0.5f;               // Langsame Annäherungs-Geschwindigkeit [RPS]

static const int STRAIGHT_LOOPS       = 70;             // 1.4 s Geradeausfahrt nach Ausrichtung
static const int FOLLOW_1S_LOOPS      = static_cast<int>(50 * SPEED_SCALE);  // ~0.6 s Linienfolgen bis Bremsen
static const int BRAKE_LOOPS          = 12;             // 0.24 s Bremsrampe von Vollgas auf Null
static const int PAUSE_LOOPS          = 20;             // 0.4 s Stillstand nach Bremsen
static const int BACKWARD_LOOPS       = 222;            // 4.44 s Rückwärtsfahrt zur Startposition
static const int ACCEL_LOOPS          = 12;             // 0.24 s Anlauframpe in STATE_APPROACH
static const int RESTART_ACCEL_LOOPS  = 50;             // 1 s Anlauframpe nach jedem Stopp
static const int REAL_APPROACH_GUARD  = 20;             // Ignore bars until robot has physically cleared the start bar
static const int STOP_GUARD           = static_cast<int>(75  * SPEED_SCALE);  // 1.5 s Sperre nach Querbalken
static const int START_GUARD          = static_cast<int>(300 * SPEED_SCALE);  // 6 s Sperre beim ersten Einlauf
static const int SLOWDOWN_LOOPS       = 13;             // 0.26 s Bremsrampe bei Farberkennung
static const float SLOW_FACTOR        = 0.2f;           // Reduktion auf 20% Geschwindigkeit beim Abbremsen
static const int COLOR_READ_DELAY     = 50;             // 1 s Sperrzeit nach Stopp gegen Phantomfarben
static const int COLOR_STABLE_CNT     = 3;              // 3 gleiche Farbmessungen nötig (ROT/GELB, fahrend)
static const int COLOR_READ_PHASE     = 25;             // 0.5 s Lesefenster für Farbbestätigung
static const int COLOR_STOP_STABLE_CNT = 1;             // 1 stabile Messung genügt im Stillstand

// ===========================================================================
// Programmkonstanten
// ===========================================================================
static const int   TOTAL_CROSSINGS       = 4;           // Anzahl breiter Querbalken im Parcours
static const int   CROSSING_STOP_LOOPS   = 250;         // 5 s reading window — allows tray to rotate up to 270°
static const int   SMALL_FOLLOW_START_GUARD   = static_cast<int>(781 * SPEED_SCALE);  // 9.8 s Sperre nach 4. Balken
static const int   SMALL_REENTRY_GUARD        = static_cast<int>(100 * SPEED_SCALE);  // 2 s Sperre gegen Farbkarten-Stopp
static const int   TOTAL_SMALL_CROSSINGS      = 4;      // Anzahl schmaler Querlinien im Parcours
static const int   SMALL_CROSSING_STOP_LOOPS  = 100;    // 2 s Lesefenster bei schmalen Linien

// 180° Servo-Positionen (normierter Puls 0.0–1.0)
static const float D1_RETRACTED       = 0.1f;           // D1 eingefahren (sicherer Startzustand)
static const float D1_EXTENDED        = 0.82f;          // D1 Standard-Ausfahrposition (Fallback)
static const float D1_EXTENDED_ROT    = 0.87f;          // ~7 mm extra extension for heavier packages at ROT slot
static const float D1_EXTENDED_GELB   = 0.87f;          // ~7 mm extra extension for heavier packages at GELB slot
static const float D1_EXTENDED_GRUEN  = 0.87f;          // ~7 mm extra extension for heavier packages at GRUEN slot
static const float D1_EXTENDED_BLAU   = 0.87f;          // ~7 mm extra extension for heavier packages at BLAU slot
static const float D2_UP              = 1.0f;           // D2 vollständig eingefahren (Transportposition)
static const float D2_PARTIAL_DOWN    = 0.50f;          // D2 halb abgesenkt (nicht verwendet)
static const float D2_DOWN_BLAU       = 0.25f;          // D2 Ablegetiefe für BLAU-Behälter
static const float D2_DOWN_GRUEN      = 0.10f;          // D2 Ablegetiefe für GRÜN-Behälter (flacher als BLAU)
static const float D2_SMALL_DESCENT_ACCEL = 4.0f;          // Sanfte D2-Absenkung bei Schmallinien (~0.6 s Hub)

// ===========================================================================
// Ablage-Sequenz Timing
// ===========================================================================
static const int   PICKUP_PAUSE_LOOPS  = 50;            // 1 s Pause nach D1-Ausfahren — Tray stabilisiert sich
static const int   D2_DOWN_DELAY_LOOPS = 23;            // 0.45 s pause before lowering arm — lets tray settle at carry angle
static const int   D2_DOWN_ONLY_LOOPS  = 12;            // 0.24 s D2-only drop before D1 extends — prevents collision while swinging out
static const int   JIGGLE_LOOPS        = 65;            // 1.3 s jiggle duration (alle breiten Balken)
static const int   JIGGLE_LOOPS_SMALL   = 25;            // 0.5 s jiggle fuer Schmallinien
static const int   JIGGLE_RAMP_LOOPS    = 12;            // 0.24 s Auf-/Abrampe der Jiggle-Amplitude
static const int   RAISE_LOOPS         = 50;            // 1 s zum Hochfahren von D1+D2 nach Ablage
static const int   WIDE_BAR_SETTLE_LOOPS = 5;             // 0.1 s Pause nach Jiggle vor Hochfahren (nur breite Balken)
static const int   RETRACT_LOOPS       = 120;           // 2.4 s zum vollständigen Einfahren von D1
static const float JIGGLE_AMPL_DEG            = 11.0f;   // Referenzwinkel (unused direct; Amp via D1_JIGGLE_OFFSET / TRAY_JIGGLE_SPEED)
static const float D1_JIGGLE_OFFSET_BAR1      = 0.098f;   // D1 Amplitude Balken 1 (12 Grad)
static const float D1_JIGGLE_OFFSET           = 0.09f;    // D1 Amplitude Balken 2 (11 Grad)
static const float D1_JIGGLE_OFFSET_SMALL     = 0.057f;   // D1 Amplitude Schmallinien (~7 Grad)
static const float D1_JIGGLE_OFFSET_BAR3      = 0.082f;   // D1 Amplitude Balken 3 (10 Grad)
static const float D1_JIGGLE_OFFSET_LAST_WIDE = 0.070f;   // D1 Amplitude Balken 4 ( 9 Grad)
static const int   D1_JIGGLE_PERIOD   = 30;             // D1 oscillation period: one full cycle every 0.6 s (30 loops)
static const int   TRAY_JIGGLE_PERIOD = 30;             // Tray oscillation period: one full cycle every 0.6 s
static const float TRAY_JIGGLE_SPEED_BAR1      = 0.38f;   // Tray Amplitude Balken 1 (12 Grad)
static const float TRAY_JIGGLE_SPEED_BAR1_CCW  = 0.38f;   // Tray Balken 1 in Karussel  (12 Grad, symmetrisch)
static const float TRAY_JIGGLE_SPEED          = 0.35f;   // Tray Amplitude Balken 2 (11 Grad)
static const float TRAY_JIGGLE_SPEED_CCW        = 0.31f;   // Tray Balken 2 in Karussel  ( 9.8 Grad)
static const float TRAY_JIGGLE_SPEED_SMALL_CW  = 0.25f;   // Tray Schmallinien gegen Karussel (~8 Grad)
static const float TRAY_JIGGLE_SPEED_SMALL_CCW = 0.19f;   // Tray Schmallinien in Karussel  (~6 Grad)
static const float TRAY_JIGGLE_SPEED_BAR3      = 0.32f;   // Tray Amplitude Balken 3 (10 Grad)
static const float TRAY_JIGGLE_SPEED_BAR3_CCW  = 0.28f;   // Tray Balken 3 in Karussel  ( 8.8 Grad)
static const float TRAY_JIGGLE_SPEED_LAST_WIDE = 0.25f;   // Tray Amplitude Balken 4 ( 9 Grad)
static const float TRAY_JIGGLE_SPEED_LAST_WIDE_CCW = 0.21f; // Tray Balken 4 in Karussel  ( 7.8 Grad)
static const int   SF360_TIMEOUT_LOOPS = 150;           // Yellow needs 270° rotation — 60 loops was too short, 150 gives margin

static const int   ROT_GELB_DRIVE_LOOPS  = 51;          // 1.02 s Vorwärtsfahrt für ROT/GELB zum richtigen Ablage-Slot
static const int   ROT_GELB_ACCEL_LOOPS  = 10;          // 0.2 s Anlauframpe für ROT/GELB Fahrt
static const int   ROT_GELB_BRAKE_LOOPS  = 12;          // 0.24 s Bremsrampe für ROT/GELB Fahrt

// Drive a few mm past bar 1 before stopping — centres sensor bar on the marking
static const int   POST_BAR_DRIVE_LOOPS  = 9;           // ~9 mm extra travel after bar 1 detection for better alignment

static const float SENSOR_THRESHOLD = 0.40f;            // Schwellwert IR-Sensor: ab diesem Wert gilt Sensor als aktiv

// ===========================================================================
// ServoFeedback360 Parameter (Parallax 360° Tablett-Servo)
// ===========================================================================
static const float SF360_KP          = 0.005f;          // P-Gain für Winkelregler (empirisch bestimmt)
static const float SF360_TOL_DEG     = 2.1f;            // Toleranzband ±2.1° gilt als "am Ziel"
static const float SF360_MIN_SPEED   = 0.18f;           // Above dead-band (~0.14) so servo never stalls when approaching target
static const float SF360_OFFSET      = 110.0f;          // Mechanischer Offset: Tablett-0° liegt bei 110° Sensorwert
static const int   SF360_WARMUP_LOOPS = 25;             // 0.5 s Einschaltzeit bis PwmIn valides Signal liefert
static const float JIGGLE_DEG        = 5.0f;            // ±5° Tray-Vorabrotation (nicht aktiv genutzt)

// Fester Zielwinkel des Tabletts pro Farbe (Grad, 0°=Endschalter-Position)
static const float ANGLE_ROT   = 0.0f;                  // ROT-Slot: Endschalter-Position (0°)
static const float ANGLE_GRUEN = 90.0f;                 // GRÜN-Slot: 90° CCW
static const float ANGLE_BLAU  = 180.0f;                // BLAU-Slot: 180° CCW (gegenüberliegend)
static const float ANGLE_GELB  = 270.0f;                // GELB-Slot: 270° CCW

// ===========================================================================
// States (15 States: 6 Ausrichtung + 5 Navigation + 4 Ablage + 1 Ende)
// ===========================================================================
enum State {
    // --- Ausrichtungsphase (6) ---
    STATE_ALIGN_START    = 1,   // Blindfahrt bis Startlinie erkannt
    STATE_ALIGN_STRAIGHT = 2,   // Geradeaus von Startlinie wegfahren
    STATE_ALIGN_TURN     = 3,   // Auf der Stelle drehen bis Linie gefunden
    STATE_ALIGN_FOLLOW   = 4,   // Linie folgen + Einlauf-Guard + Bremsvorbereitung (verschmelzt FOLLOW + FOLLOW_1S)
    STATE_ALIGN_BRAKE    = 5,   // Bremsrampe + Standstill-Pause (verschmelzt BRAKE + PAUSE)
    STATE_ALIGN_REVERSE  = 6,   // Rückwärtsfahrt zur Startposition

    // --- Navigationsphase (5) ---
    STATE_START_PAUSE    = 14,  // Kurze Pause vor erstem Querbalken
    STATE_APPROACH       = 15,  // Langsame Annäherung an ersten Querbalken
    STATE_REAL_FOLLOW    = 9,   // Linienfolger für breite Balken (Vollgas + Farberkennung)
    STATE_SMALL_FOLLOW   = 12,  // Linienfolger für schmale Querlinien
    STATE_TUNNEL_FOLLOW  = 24,  // 3-Phasen Tunnelfahrt (A: Linie, B: Gerade, C: Linie)

    // --- Dispatch-State (1, für breit und schmal) ---
    STATE_STOP_AND_READ  = 10,  // Stillstand + Farblesen + Dispatch zu Farb-State (verschmelzt CROSSING + SMALL)

    // --- Ablage-States (4) ---
    STATE_RED    = 20,          // Ablage-Sequenz für ROT  (Tray 0°,   Vorwärtsfahrt nötig)
    STATE_GREEN  = 21,          // Ablage-Sequenz für GRÜN (Tray 90°,  kein Fahren)
    STATE_BLUE   = 22,          // Ablage-Sequenz für BLAU (Tray 180°, kein Fahren)
    STATE_YELLOW = 23,          // Ablage-Sequenz für GELB (Tray 270°, Vorwärtsfahrt nötig)

    // --- Ende ---
    STATE_FINISH = 11           // Endposition: alles gestoppt
};

// ===========================================================================
// Hardware-Zeiger (statisch, in _init() befüllt)
// ===========================================================================
static DCMotor*          g_M1        = nullptr;
static DCMotor*          g_M2        = nullptr;
static DigitalOut*       g_en        = nullptr;
static LineFollower*     g_lf        = nullptr;
static ColorSensor*      g_cs        = nullptr;
static Servo*            g_servo_D1  = nullptr;
static Servo*            g_servo_D2  = nullptr;
static ServoFeedback360* g_servoTray = nullptr;
static NeoPixel*         g_np        = nullptr;

// ===========================================================================
// Laufzeit-Zustand
// ===========================================================================
static State m_state = STATE_ALIGN_START;

static int   m_sf360_warmup_ctr = 0;
static bool  m_sf360_ready      = false;
static float m_target_angle     = 0.0f;

static int   m_straight_ctr        = 0;
static int   m_follow_ctr          = 0;     // Zählt FOLLOW_1S-Phase ab (0 = Guard-Phase aktiv)
static int   m_brake_ctr           = 0;
static int   m_pause_ctr           = 0;
static int   m_backward_ctr        = 0;
static int   m_guard_ctr           = 0;
static int   m_crossing_ctr        = 0;     // Lesefenster-Countdown (breit: 250, schmal: 100)
static int   m_crossings_left      = 0;
static int   m_tray_wait_ctr       = 0;     // (ehem. m_tray_timeout_ctr) Wartezeit auf Tray-Positionierung
static int   m_small_crossings_left = 0;
static int   m_approach_ctr        = 0;
static int   m_small_accel_ctr     = 0;
static int   m_real_accel_ctr      = 0;
static int   m_arm_retract_ctr     = 0;
static int   m_deliver_phase       = 0;     // (ehem. m_phase_idx) Aktuelle Phase in runDeliverPhase (0–3)
static int   m_deliver_ctr         = 0;     // (ehem. m_phase_ctr) Loop-Zähler innerhalb der aktuellen Phase
static int   m_forward_ctr         = 0;     // (ehem. m_drive_ctr) ROT/GELB Vorwärtsfahrt-Zähler
static bool  m_reversing           = false; // (ehem. m_driving_back) ROT/GELB Rückfahrt nach letztem Balken aktiv
static int   m_reverse_ctr         = 0;     // (ehem. m_back_ctr) ROT/GELB Rückfahrt-Loop-Zähler
static int   m_post_bar_ctr        = 0;     // Extra-Loops nach erster Balkenerkennung in STATE_APPROACH
static int   m_arm_prep_phase      = 0;     // Tray-Vorbereitungsphase (Reserviert)
static float m_brake_start_M1      = 0.0f;
static float m_brake_start_M2      = 0.0f;
static float g_cmd_M1              = 0.0f;
static float g_cmd_M2              = 0.0f;

static int   m_current_color       = 0;
static int   m_prev_color          = 0;
static int   m_led_color           = 0;
static int   m_led_ctr             = 0;
static int   m_color_log[8]        = {};
static int   m_color_log_ctr       = 0;
static int   m_action_color        = 0;
static bool  m_slowing             = false;
static int   m_slow_ctr            = 0;
static int   m_color_delay_ctr     = 0;
static int   m_color_stable_ctr    = 0;
static int   m_color_pending       = 0;
static int   m_color_fallback      = 0;
static int   m_approach_fallback   = 0;

static bool  m_tray_moving         = false;
static bool  m_small_line_mode     = false; // true wenn aktueller Stopp von Schmallinie stammt
static int   m_kick_ctr            = 0;
static float m_last_target_angle   = -1.0f;
static int   m_line_count          = 0;     // Gesamtzähler aller erkannten Linien (4 breit + 4 schmal)
static bool  m_counting            = false; // Wird nach Ausrichtung auf true gesetzt

// NeoPixel-Zustandsmaschine
enum NeoMode { NEO_CYCLE = 0, NEO_COLOR = 1, NEO_DRIVE_HOLD = 2, NEO_OFF_WAIT = 3 };
static NeoMode m_neo_mode       = NEO_CYCLE;
static int     m_neo_timer      = 0;
static int     m_neo_held_color = 0;
static const int NEO_DRIVE_HOLD_LOOPS = 150; // 3 s Farb-Hold nach letztem Balken (bar 4 oder 8)
static const int NEO_OFF_LOOPS        = 50;  // 1 s Schwarzphase nach Hold
static const int NEO_CYCLE_LOOPS      = 40;  // 0.8 s pro Farbe im Farbzyklus

static int   m_home_timer          = 0;
static int   m_startup_delay       = 150;   // 3 s Wartezeit nach Button-Druck vor Losfahren

// ===========================================================================
// Sensor-Hilfsfunktionen
// ===========================================================================

// Gibt true zurück wenn ≥7 von 8 Sensoren auf der Linie stehen (toleriert Schrägeinlauf).
static bool all_sensors_active()
{
    int active = 0;
    for (int i = 0; i < 8; i++) {
        if (g_lf->getAvgBit(i) >= SENSOR_THRESHOLD) active++;
    }
    return active >= 7;
}

// Gibt true zurück wenn mindestens einer der beiden mittleren Sensoren die Linie sieht.
static bool center_sensors_active()
{
    return g_lf->getAvgBit(3) >= SENSOR_THRESHOLD ||
           g_lf->getAvgBit(4) >= SENSOR_THRESHOLD;
}

// Gibt true zurück wenn ≥4 Sensoren aktiv sind — erkennt breite Querbalken.
static bool wide_bar_active()
{
    int count = 0;
    for (int i = 0; i < 8; i++) {
        if (g_lf->getAvgBit(i) >= SENSOR_THRESHOLD) count++;
    }
    return count >= 4;
}

// Gibt true zurück wenn ein Cluster von 3 zentralen Sensoren (3-4-5 oder 2-3-4) aktiv ist.
// Erkennt schmale Querlinien zuverlässig ohne Verwechslung mit breiten Balken.
static bool small_line_active()
{
    return (g_lf->getAvgBit(3) >= SENSOR_THRESHOLD &&
            g_lf->getAvgBit(4) >= SENSOR_THRESHOLD &&
            g_lf->getAvgBit(5) >= SENSOR_THRESHOLD) ||
           (g_lf->getAvgBit(2) >= SENSOR_THRESHOLD &&
            g_lf->getAvgBit(3) >= SENSOR_THRESHOLD &&
            g_lf->getAvgBit(4) >= SENSOR_THRESHOLD);
}

// ===========================================================================
// Farb- und Servo-Hilfsfunktionen
// ===========================================================================

// Gibt die Ablagetiefe von D2 zurück (BLAU und GELB identisch, GRÜN flacher).
static float depthForColor(int c)
{
    if (c == 7 || c == 4) return D2_DOWN_BLAU;
    return D2_DOWN_GRUEN;
}

// Gibt true wenn der Tray innerhalb von tol Grad am Ziel ist (Kurzweg-Fehler).
static bool trayNearTarget(float tol = 20.0f)
{
    float err = g_servoTray->getCurrentAngle() - m_target_angle;
    if (err >  180.0f) err -= 360.0f;
    if (err < -180.0f) err += 360.0f;
    return fabsf(err) < tol;
}

// Gibt den Ziel-Tray-Winkel für die gegebene Farbe zurück.
static float angleForColor(int c)
{
    switch (c) {
        case 3: return ANGLE_ROT;
        case 4: return ANGLE_GELB;
        case 5: return ANGLE_GRUEN;
        case 7: return ANGLE_BLAU;
        default: return m_target_angle;
    }
}

// Pre-rotate 30° short of target so tray inertia doesn't overshoot on arrival.
// Tray fährt direkt hierher während Annäherung, dann Arm raus, dann beim D2-Absenken auf Zielwinkel.
static float carryAngleForColor(int c)
{
    return fmodf(angleForColor(c) - 30.0f + 360.0f, 360.0f);
}

// Gibt den farbspezifischen D1-Ausfahrweg zurück (~7 mm länger als Standard für schwerere Pakete).
static float extensionForColor(int c)
{
    switch (c) {
        case 3: return D1_EXTENDED_ROT;
        case 4: return D1_EXTENDED_GELB;
        case 5: return D1_EXTENDED_GRUEN;
        case 7: return D1_EXTENDED_BLAU;
        default: return D1_EXTENDED;
    }
}

// ===========================================================================
// NeoPixel-Hilfsfunktionen
// ===========================================================================

// Setzt die NeoPixel-Farbe direkt auf den Farbcode (3=ROT, 4=GELB, 5=GRÜN, 7=BLAU).
static void setNeoColor(int color)
{
    if (!g_np) return;
    switch (color) {
        case 3: g_np->setRGB(255,   0,   0); break;
        case 4: g_np->setRGB(220, 220,   0); break;
        case 5: g_np->setRGB(  0, 255,   0); break;
        case 7: g_np->setRGB(  0,   0, 255); break;
        default: g_np->setRGB(5, 5, 5);      break;
    }
    g_np->show();
}

// Steuert die NeoPixel-Zustandsmaschine (Farbzyklus → Farb-Hold → Schwarzpause → Zyklus).
static void updateNeoPixel()
{
    if (!g_np) return;
    switch (m_neo_mode) {

        case NEO_CYCLE: {
            // Cycle R→G→B→Y at 0.8 s each — stays until arm state triggers NEO_COLOR
            m_neo_timer++;
            int phase = (m_neo_timer / NEO_CYCLE_LOOPS) % 4;
            switch (phase) {
                case 0: g_np->setRGB(200,   0,   0); break;
                case 1: g_np->setRGB(  0, 200,   0); break;
                case 2: g_np->setRGB(  0,   0, 200); break;
                case 3: g_np->setRGB(200, 200,   0); break;
            }
            g_np->show();
            break;
        }

        case NEO_COLOR:
            setNeoColor(m_neo_held_color);
            break;

        case NEO_DRIVE_HOLD:
            // Hold last delivery colour for 3 s, then go dark
            setNeoColor(m_neo_held_color);
            m_neo_timer++;
            if (m_neo_timer >= NEO_DRIVE_HOLD_LOOPS) {
                m_neo_mode  = NEO_OFF_WAIT;
                m_neo_timer = 0;
            }
            break;

        case NEO_OFF_WAIT:
            // Black for 1 s, then colour cycle
            g_np->setRGB(0, 0, 0);
            g_np->show();
            m_neo_timer++;
            if (m_neo_timer >= NEO_OFF_LOOPS) {
                m_neo_mode  = NEO_CYCLE;
                m_neo_timer = 0;
            }
            break;
    }
}

// ===========================================================================
// Tray-Steuerung
// ===========================================================================

// Setzt den Zielwinkel des Tabletts. serviceTray() übernimmt die Regelung jede Loop.
static void trayMoveTo(float deg)
{
    m_target_angle = deg;
}

// No-op: Servo bleibt aktiv. Nur STATE_FINISH deaktiviert den Servo direkt.
static void trayStop()
{
    // No-op: servo stays enabled, P-controller continues via serviceTray().
    // FINISH stops/disables directly — all other callers just leave servo running.
}

// Erzeugt ein Dreiecks-Jiggle-Signal für den Tray: –AMPL → +AMPL → ZIEL (3 Phasen à JIGGLE_LOOPS/3).
static void applyJiggleTick(int tick)
{
    const int ph1 = JIGGLE_LOOPS / 3;
    const int ph2 = 2 * JIGGLE_LOOPS / 3;
    int jp = tick % JIGGLE_LOOPS;
    if      (jp == 0)    g_servoTray->moveToAngle(m_target_angle - JIGGLE_AMPL_DEG);
    else if (jp == ph1)  g_servoTray->moveToAngle(m_target_angle + JIGGLE_AMPL_DEG);
    else if (jp == ph2)  g_servoTray->moveToAngle(m_target_angle);
}

// ===========================================================================
// Ablage-Sequenz runDeliverPhase()
//
// Führt die 4-phasige Ablagesequenz aus. Wird einmal pro Loop aufgerufen.
// Gibt true zurück wenn die Sequenz vollständig abgeschlossen ist.
//
//   Phase 0: D2 absenken (D2_DOWN_ONLY_LOOPS = 0.24 s) — Arm noch eingefahren
//   Phase 1: D1 ausfahren + Tray auf Zielwinkel (PICKUP_PAUSE_LOOPS = 1 s)
//   Phase 2: D1 Jiggle (JIGGLE_LOOPS = 1.3 s) — Paket durch Vibration ablösen
//   Phase 3: D2 + D1 hochfahren (RAISE_LOOPS = 1 s), dann D1 deaktivieren → true
// ===========================================================================
static bool runDeliverPhase()
{
    // Phase 0: Warten bis Tray den Zielwinkel erreicht hat (verhindert Ablage bei falschem Winkel).
    if (m_deliver_phase == 0) {
        m_deliver_ctr++;
        float ccw = fmodf(g_servoTray->getCurrentAngle() - m_target_angle + 360.0f, 360.0f);
        bool tray_ok = (ccw <= 35.0f) || (ccw >= 185.0f) || (m_deliver_ctr >= SF360_TIMEOUT_LOOPS);
        if (tray_ok) { m_deliver_phase = 1; m_deliver_ctr = 0; }
        return false;
    }
    // Phase 1: D2 nur runter — D1 und Tray bleiben beim Carry-Winkel.
    if (m_deliver_phase == 1) {
        if (m_deliver_ctr == 0) {
            g_servo_D2->setMaxAcceleration(D2_SMALL_DESCENT_ACCEL);
            g_servo_D2->enable(depthForColor(m_action_color));
        }
        m_deliver_ctr++;
        if (m_deliver_ctr >= D2_DOWN_ONLY_LOOPS) { m_deliver_phase = 2; m_deliver_ctr = 0; }
        return false;
    }
    // Phase 2: D1 raus + Tray auf Zielwinkel korrigieren.
    if (m_deliver_phase == 2) {
        if (m_deliver_ctr == 0) {
            g_servo_D1->enable(extensionForColor(m_action_color));
            m_target_angle = angleForColor(m_action_color);
        }
        m_deliver_ctr++;
        if (m_deliver_ctr >= PICKUP_PAUSE_LOOPS) { m_deliver_phase = 3; m_deliver_ctr = 0; }
        return false;
    }
    // Phase 3: D1 Jiggle — Puls-Breitenmodulation ±D1_JIGGLE_OFFSET mit D1_JIGGLE_PERIOD.
    if (m_deliver_phase == 3) {
        const float base    = extensionForColor(m_action_color);
        float jig_off;
        if (m_small_line_mode) {
            jig_off = D1_JIGGLE_OFFSET_SMALL;
        } else {
            switch (m_line_count) {
                case 1:  jig_off = D1_JIGGLE_OFFSET_BAR1;      break;
                case 3:  jig_off = D1_JIGGLE_OFFSET_BAR3;      break;
                case 4:  jig_off = D1_JIGGLE_OFFSET_LAST_WIDE; break;
                default: jig_off = D1_JIGGLE_OFFSET;           break;
            }
        }
        const int jig_end = m_small_line_mode ? JIGGLE_LOOPS_SMALL : JIGGLE_LOOPS;
        // Amplituden-Envelope: 0 -> max -> 0 ueber die Jiggle-Phase
        const int   ramp = (JIGGLE_RAMP_LOOPS < jig_end / 3) ? JIGGLE_RAMP_LOOPS : jig_end / 3;
        float       t;
        if      (m_deliver_ctr < ramp)            t = (float)(m_deliver_ctr + 1) / ramp;
        else if (m_deliver_ctr >= jig_end - ramp) t = (float)(jig_end - m_deliver_ctr) / ramp;
        else                                      t = 1.0f;
        if (t < 0.0f) t = 0.0f;
        const float eff_off = jig_off * t;
        const int   half = D1_JIGGLE_PERIOD / 2;
        int p = m_deliver_ctr % D1_JIGGLE_PERIOD;
        if      (p == 0)    g_servo_D1->setPulseWidth(base + eff_off);
        else if (p == half) g_servo_D1->setPulseWidth(base - eff_off);
        m_deliver_ctr++;
        if (m_deliver_ctr >= jig_end) {
            g_servo_D1->setPulseWidth(base);
            m_deliver_phase = 4;
            m_deliver_ctr   = 0;
        }
        return false;
    }
    // Phase 4: 0.1 s Settle-Pause (breite Balken) + Arm hochfahren.
    if (m_deliver_phase == 4) {
        const int settle = m_small_line_mode ? 0 : WIDE_BAR_SETTLE_LOOPS;
        if (m_deliver_ctr == settle) {
            g_servo_D2->setMaxAcceleration(1.0e6f);
            g_servo_D2->setPulseWidth(D2_UP);
            g_servo_D1->setMaxAcceleration(1.0e6f);
            g_servo_D1->setPulseWidth(D1_RETRACTED);
        }
        m_deliver_ctr++;
        if (m_deliver_ctr >= RAISE_LOOPS + settle) {
            g_servo_D1->disable();
            return true;
        }
        return false;
    }
    return true;
}

// ===========================================================================
// serviceTray()
//
// Wird jede Loop aufgerufen. Steuert den 360°-Servo je nach Zustand:
//   - In Ablage-States (RED/GREEN/BLUE/YELLOW): P-Regler auf m_target_angle.
//     In Phase 2 (Jiggle): addSpeed() mit Square-Wave überschreibt P-Regler.
//     Ausserhalb des Ziel-Fensters: Kick-Impuls für Anlaufdrehmoment.
//   - In Navigations-States: langsame CCW-Drehung (-0.27) mit Kick (-0.35) bei
//     Winkeländerung. Ermöglicht Pre-Adjust während Fahrt.
// ===========================================================================
static void serviceTray()
{
    if (!g_servoTray) return;
    if (!m_sf360_ready) return;  // Tray ist noch nicht aktiviert (vor Ausrichtung)
    if (m_state == STATE_FINISH) return;

    // Kick-Impuls bei jedem neuen Zielwinkel (25 Loops = 0.5 s)
    if (m_target_angle != m_last_target_angle) {
        m_kick_ctr          = 25;
        m_last_target_angle = m_target_angle;
    }

    bool in_arm_state = (m_state == STATE_RED    || m_state == STATE_GREEN ||
                         m_state == STATE_BLUE   || m_state == STATE_YELLOW);

    if (in_arm_state) {
        g_servoTray->moveToAngle(m_target_angle);
        g_servoTray->update();

        // Ziel-Fenster: CCW-Restdrehung ≤35° oder ≥185° (Tray fast am Ziel)
        float ccw_dist = fmodf(g_servoTray->getCurrentAngle() - m_target_angle + 360.0f,
                               360.0f);
        bool in_window = (ccw_dist <= 35.0f) || (ccw_dist >= 185.0f);

        if (in_window) {
            if (m_deliver_phase == 3) {
                // Tray-Jiggle in Phase 2: Square-Wave ±TRAY_JIGGLE_SPEED überschreibt P-Regler
                static int s_jiggle_ctr = 0;
                const int half = TRAY_JIGGLE_PERIOD / 2;
                int p = s_jiggle_ctr % TRAY_JIGGLE_PERIOD;
                float spd_pos, spd_neg;
                if (m_small_line_mode) {
                    spd_pos = TRAY_JIGGLE_SPEED_SMALL_CW;   // gegen Karussel
                    spd_neg = TRAY_JIGGLE_SPEED_SMALL_CCW;  // in Karussel
                } else {
                    switch (m_line_count) {
                        case 1:  spd_pos = TRAY_JIGGLE_SPEED_BAR1;          spd_neg = TRAY_JIGGLE_SPEED_BAR1_CCW;      break;
                        case 3:  spd_pos = TRAY_JIGGLE_SPEED_BAR3;          spd_neg = TRAY_JIGGLE_SPEED_BAR3_CCW;      break;
                        case 4:  spd_pos = TRAY_JIGGLE_SPEED_LAST_WIDE;     spd_neg = TRAY_JIGGLE_SPEED_LAST_WIDE_CCW; break;
                        default: spd_pos = TRAY_JIGGLE_SPEED;               spd_neg = TRAY_JIGGLE_SPEED_CCW;           break;
                    }
                }
                // Gleiche Amplituden-Envelope wie D1 (basiert auf m_deliver_ctr)
                const int jig_end_t = m_small_line_mode ? JIGGLE_LOOPS_SMALL : JIGGLE_LOOPS;
                const int ramp_t = (JIGGLE_RAMP_LOOPS < jig_end_t / 3) ? JIGGLE_RAMP_LOOPS : jig_end_t / 3;
                float t2;
                if      (m_deliver_ctr < ramp_t)                t2 = (float)(m_deliver_ctr + 1) / ramp_t;
                else if (m_deliver_ctr >= jig_end_t - ramp_t)  t2 = (float)(jig_end_t - m_deliver_ctr) / ramp_t;
                else                                            t2 = 1.0f;
                if (t2 < 0.0f) t2 = 0.0f;
                float jiggle = (p < half) ? +spd_pos * t2 : -spd_neg * t2;
                g_servoTray->addSpeed(jiggle);
                s_jiggle_ctr++;
            }
        } else {
            // Ausserhalb Fenster: Servo stoppen + Kick-Geschwindigkeit addieren
            g_servoTray->stop();
            float speed = (m_kick_ctr > 0) ? -0.40f : -0.30f;
            if (m_kick_ctr > 0) m_kick_ctr--;
            g_servoTray->addSpeed(speed);
        }
    } else {
        // Navigation: langsame CCW-Drehung zum Vorausrichten, Kick bei neuem Ziel
        g_servoTray->update();
        g_servoTray->stop();
        float nav_speed = (m_kick_ctr > 0) ? -0.40f : -0.30f;
        if (m_kick_ctr > 0) m_kick_ctr--;
        g_servoTray->addSpeed(nav_speed);
    }
}

// Dreht das Tablett um +90° weiter (für zukünftige Verwendung reserviert).
static void advanceTray()
{
    m_target_angle = fmodf(m_target_angle + 90.0f, 360.0f);
}

// ===========================================================================
// tryLockColor()
//
// Bestätigt eine Farbe durch wiederholtes Lesen im Stillstand.
// Gibt true zurück wenn col_now >= threshold Mal in Folge identisch war.
// Kein Reset bei neutraler Farbe (0/1/2) — Zähler bleibt erhalten.
// Wird nur in STATE_STOP_AND_READ verwendet.
// ===========================================================================
static bool tryLockColor(int col_now, int& pending, int& stable_ctr, int threshold)
{
    bool is_action = (col_now == 3 || col_now == 4 || col_now == 5 || col_now == 7);
    if (is_action && col_now == pending) {
        stable_ctr++;
    } else if (is_action) {
        pending    = col_now;
        stable_ctr = 1;
    }
    return (stable_ctr >= threshold);
}

// ===========================================================================
// Init
// ===========================================================================
void cargosweep_team5_20260511_init(int /*loops_per_second*/)
{
    static DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1,
                             GEAR_RATIO, KN, VOLTAGE_MAX);
    static DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2,
                             GEAR_RATIO, KN, VOLTAGE_MAX);
    static DigitalOut enable_motors(PB_ENABLE_DCMOTORS);
    static LineFollower lineFollower(PB_9, PB_8, BAR_DIST, D_WHEEL, B_WHEEL,
                                     motor_M1.getMaxPhysicalVelocity());
    static ColorSensor colorSensor(PB_3, PC_3, PA_4, PB_0, PC_1, PC_0);
    static Servo servo_D1(PC_8);
    static Servo servo_D2(PB_2);
    static ServoFeedback360 servoTray(PB_12, PC_2,
                                      SF360_KP, SF360_TOL_DEG,
                                      SF360_MIN_SPEED, SF360_OFFSET);
    static NeoPixel neopixel(PC_5);

    g_M1        = &motor_M1;
    g_M2        = &motor_M2;
    g_en        = &enable_motors;
    g_lf        = &lineFollower;
    g_cs        = &colorSensor;
    g_servo_D1  = &servo_D1;
    g_servo_D2  = &servo_D2;
    g_servoTray = &servoTray;
    g_np        = &neopixel;

    g_np->clear();
    g_cs->setCalibration();
    g_cs->switchLed(ON);
    g_servo_D1->calibratePulseMinMax(0.0500f, 0.1050f);
    g_servo_D1->setMaxAcceleration(0.3f);
    g_servo_D2->calibratePulseMinMax(0.0200f, 0.1310f);
    // D1, D2, Tray werden erst in STATE_ALIGN_START aktiviert (wenn all_sensors_active()).

    g_lf->setRotationalVelocityControllerGains(KP, KP_NL);
    g_lf->setMaxWheelVelocity(MAX_SPEED);

    TIM1->BDTR |= TIM_BDTR_MOE;  // STM32 Hardware: Motor-Output muss manuell freigegeben werden

    *g_en = 0;

    m_state               = STATE_ALIGN_START;
    m_sf360_warmup_ctr    = 0;
    m_sf360_ready         = false;
    m_target_angle        = 0.0f;
    m_straight_ctr        = 0;
    m_follow_ctr          = 0;
    m_brake_ctr           = 0;
    m_pause_ctr           = 0;
    m_backward_ctr        = 0;
    m_guard_ctr           = 0;
    m_crossing_ctr        = 0;
    m_crossings_left      = 0;
    m_small_crossings_left = 0;
    m_approach_ctr        = 0;
    m_small_accel_ctr     = 0;
    m_real_accel_ctr      = 0;
    m_arm_retract_ctr     = 0;
    m_deliver_phase       = 0;
    m_deliver_ctr         = 0;
    m_forward_ctr         = 0;
    m_post_bar_ctr        = 0;
    m_arm_prep_phase      = 0;
    m_tray_wait_ctr       = 0;
    m_brake_start_M1      = 0.0f;
    m_brake_start_M2      = 0.0f;
    m_current_color       = 0;
    m_prev_color          = 0;
    m_led_color           = 0;
    m_led_ctr             = 0;
    m_color_log_ctr       = 0;
    m_action_color        = 0;
    m_slowing             = false;
    m_slow_ctr            = 0;
    m_color_delay_ctr     = 0;
    m_color_stable_ctr    = 0;
    m_color_pending       = 0;
    m_color_fallback      = 0;
    m_approach_fallback   = 0;
    m_startup_delay       = 150;
    for (int i = 0; i < 8; i++) m_color_log[i] = 0;
    g_M1->setVelocity(0.0f);
    g_M2->setVelocity(0.0f);
}

// ===========================================================================
// State-Übergangs-Helfer
// ===========================================================================

// resetArmExitVars()
// Setzt alle Arm-Sequenz-Variablen zurück am Ende jeder Ablage.
// Muss VOR jedem exitWideBar() / exitSmallLine() aufgerufen werden.
static void resetArmExitVars()
{
    m_slowing           = false;
    m_slow_ctr          = 0;
    m_action_color      = 0;
    m_color_delay_ctr   = COLOR_READ_DELAY;
    m_color_stable_ctr  = 0;
    m_color_pending     = 0;
    m_color_fallback    = 0;
    m_approach_fallback = 0;
    m_arm_retract_ctr   = 0;
    m_deliver_phase     = 0;
    m_deliver_ctr       = 0;
    m_forward_ctr       = 0;
    m_reversing         = false;
    m_reverse_ctr       = 0;
    m_post_bar_ctr      = 0;
    m_arm_prep_phase    = 0;
    m_tray_wait_ctr     = 0;
}

// exitWideBar()
// Übergang nach Abschluss eines breiten Balkens.
// Nach dem 4. Balken → STATE_TUNNEL_FOLLOW (Tunnel liegt nach Balken 4).
// Sonst → STATE_REAL_FOLLOW mit STOP_GUARD gegen sofortiges Wiedererkennen.
static void exitWideBar()
{
    if (m_line_count >= 4) {
        m_guard_ctr       = SMALL_FOLLOW_START_GUARD;
        m_small_accel_ctr = 0;
        m_state           = STATE_TUNNEL_FOLLOW;
    } else {
        m_guard_ctr      = STOP_GUARD;
        m_real_accel_ctr = 0;
        m_state          = STATE_REAL_FOLLOW;
    }
}

// exitSmallLine()
// Übergang nach Abschluss einer schmalen Querlinie.
// Nach der 8. Linie → STATE_FINISH (Roboter hält an).
// Sonst → STATE_SMALL_FOLLOW mit SMALL_REENTRY_GUARD gegen Doppel-Trigger.
static void exitSmallLine()
{
    if (m_line_count >= 8) {
        m_state = STATE_FINISH;
    } else {
        m_guard_ctr       = SMALL_REENTRY_GUARD;
        m_small_accel_ctr = 0;
        m_state           = STATE_SMALL_FOLLOW;
    }
}

// ===========================================================================
// Task (50 Hz — wird von main.cpp jede 20 ms aufgerufen)
// ===========================================================================
void cargosweep_team5_20260511_task(DigitalOut& led)
{
    m_current_color = g_cs->getColor();

    // Pre-Adjust: Tray während Annäherung vorausrichten.
    // Nur in Navigations-States aktiv — nicht während Rückfahrt oder Ablage.
    bool can_preadjust = (m_state == STATE_APPROACH      ||
                          m_state == STATE_REAL_FOLLOW   ||
                          m_state == STATE_SMALL_FOLLOW  ||
                          m_state == STATE_TUNNEL_FOLLOW);
    if (m_sf360_ready && can_preadjust) {
        int col = m_current_color;
        if (col == 3 || col == 4 || col == 5 || col == 7) {
            m_target_angle   = carryAngleForColor(col);
            m_color_fallback = col;
        }
    }

    updateNeoPixel();

    // Farb-Log: erste Detektion jeder neuen Signalfarbe aufzeichnen
    bool is_sig    = (m_current_color == 3 || m_current_color == 4 ||
                      m_current_color == 5 || m_current_color == 7);
    bool was_neutral = (m_prev_color == 0 || m_prev_color == 1 || m_prev_color == 2);
    if (is_sig && was_neutral) {
        if (m_color_log_ctr < 8) m_color_log[m_color_log_ctr++] = m_current_color;
        m_led_color = m_current_color;
    }
    m_prev_color = m_current_color;

    // LED-Muster: ohne Farbe = Blinken, mit Farbe = farbspezifisches Pulsmuster
    m_led_ctr = (m_led_ctr + 1) % 100;
    if (m_led_color == 0) {
        led = !led;
    } else {
        int n  = (m_led_color == 3) ? 1 :
                 (m_led_color == 4) ? 2 :
                 (m_led_color == 5) ? 3 :
                 (m_led_color == 7) ? 4 : 0;
        int pos = m_led_ctr;
        led     = (n > 0 && pos < n * 16 && (pos % 16) < 8) ? 1 : 0;
    }

    *g_en = 1;

    serviceTray();

    if (m_home_timer > 0) {
        if (--m_home_timer == 0) trayStop();
    }

    switch (m_state) {

        case STATE_ALIGN_START:
            // Roboter fährt blind vorwärts bis alle 8 Liniensensoren die Startlinie erkennen.
            // Erst beim Triggern werden Servos (D1, D2, Tray) aktiviert und der Linienzähler gestartet.
            // Achtung: Tray bleibt bis hier deaktiviert — kein serviceTray()-Effekt vor diesem Trigger.
            // Übergang → STATE_ALIGN_STRAIGHT sobald all_sensors_active() ≥7 Sensoren aktiv ist.
            if (m_startup_delay > 0) {
                m_startup_delay--;
                g_M1->setVelocity(0.0f);
                g_M2->setVelocity(0.0f);
                break;
            }
            g_M1->setVelocity(VEL_SIGN * BLIND_SPEED);
            g_M2->setVelocity(VEL_SIGN * BLIND_SPEED);
            if (all_sensors_active()) {
                g_servo_D1->enable(D1_RETRACTED);
                g_servo_D2->enable(D2_UP);
                g_servoTray->enable(0.5f);
                m_sf360_ready  = true;
                m_home_timer   = 150;   // 3 s Wartezeit bis Tray sich auf Startposition dreht
                m_straight_ctr = STRAIGHT_LOOPS;
                m_counting     = true;
                m_line_count   = 0;
                m_state        = STATE_ALIGN_STRAIGHT;
            }
            break;

        case STATE_ALIGN_STRAIGHT:
            // Roboter fährt STRAIGHT_LOOPS = 70 Loops (1.4 s) geradeaus von der Startlinie weg.
            // Zweck: Abstand zur Linie gewinnen, damit der Drehradius beim Wenden passt.
            // Übergang → STATE_ALIGN_TURN nach Ablauf des Counters.
            g_M1->setVelocity(VEL_SIGN * STRAIGHT_SPEED);
            g_M2->setVelocity(VEL_SIGN * STRAIGHT_SPEED);
            m_straight_ctr--;
            if (m_straight_ctr <= 0) {
                g_M1->setVelocity(0.0f);
                g_M2->setVelocity(0.0f);
                m_state = STATE_ALIGN_TURN;
            }
            break;

        case STATE_ALIGN_TURN:
            // Roboter dreht auf der Stelle (M1 rückwärts, M2 vorwärts) bis ein Mittelsensor die Linie sieht.
            // Setzt weiche Linienfolger-Gains (KP=1.1, KP_NL=0.7) für ruhigen Einlauf auf die Linie.
            // START_GUARD = 6 s verhindert sofortigen Trigger auf den Einlauf-Balken.
            // Übergang → STATE_ALIGN_FOLLOW sobald center_sensors_active() = true.
            g_M1->setVelocity(VEL_SIGN * -TURN_SPEED);
            g_M2->setVelocity(VEL_SIGN *  TURN_SPEED);
            if (center_sensors_active()) {
                g_lf->setRotationalVelocityControllerGains(KP_FOLLOW, KP_NL_FOLLOW);
                m_guard_ctr  = START_GUARD;
                m_follow_ctr = 0;
                m_state      = STATE_ALIGN_FOLLOW;
            }
            break;

        case STATE_ALIGN_FOLLOW: {
            // Phase 1 — Guard (m_follow_ctr == 0, m_guard_ctr > 0): Linie folgen, Querbalken gesperrt.
            // Phase 2 — Warten (m_follow_ctr == 0, m_guard_ctr == 0): Warten auf all_sensors_active().
            // Phase 3 — Countdown (m_follow_ctr > 0): FOLLOW_1S_LOOPS weitere Loops folgen, dann Bremsen.
            // Letzte Motorwerte werden gespeichert für symmetrische Bremsrampe in STATE_ALIGN_BRAKE.
            // Übergang → STATE_ALIGN_BRAKE wenn m_follow_ctr auf 0 abgelaufen ist.
            g_cmd_M1 = VEL_SIGN * g_lf->getLeftWheelVelocity();
            g_cmd_M2 = VEL_SIGN * g_lf->getRightWheelVelocity();
            g_M1->setVelocity(g_cmd_M1);
            g_M2->setVelocity(g_cmd_M2);
            if (m_follow_ctr > 0) {
                m_follow_ctr--;
                if (m_follow_ctr <= 0) {
                    m_brake_start_M1 = g_cmd_M1;
                    m_brake_start_M2 = g_cmd_M2;
                    m_brake_ctr      = BRAKE_LOOPS;
                    m_state          = STATE_ALIGN_BRAKE;
                }
            } else if (m_guard_ctr > 0) {
                m_guard_ctr--;
            } else if (all_sensors_active()) {
                m_follow_ctr = FOLLOW_1S_LOOPS;
            }
            break;
        }

        case STATE_ALIGN_BRAKE: {
            // Bremsphase (m_brake_ctr > 0): Geschwindigkeit linear von Startgeschwindigkeit auf 0.
            // Pausephase (m_brake_ctr == 0): Robot steht für PAUSE_LOOPS = 20 Loops (0.4 s) still.
            // Achtung: m_pause_ctr wird erst beim Übergang von Brems- zu Pausephase gesetzt.
            // Übergang → STATE_ALIGN_REVERSE nach Ende der Pause.
            if (m_brake_ctr > 0) {
                float t = static_cast<float>(m_brake_ctr) / static_cast<float>(BRAKE_LOOPS);
                g_M1->setVelocity(m_brake_start_M1 * t);
                g_M2->setVelocity(m_brake_start_M2 * t);
                m_brake_ctr--;
                if (m_brake_ctr <= 0) {
                    g_M1->setVelocity(0.0f);
                    g_M2->setVelocity(0.0f);
                    m_pause_ctr = PAUSE_LOOPS;
                }
            } else {
                g_M1->setVelocity(0.0f);
                g_M2->setVelocity(0.0f);
                m_pause_ctr--;
                if (m_pause_ctr <= 0) {
                    m_backward_ctr = BACKWARD_LOOPS;
                    m_state        = STATE_ALIGN_REVERSE;
                }
            }
            break;
        }

        case STATE_ALIGN_REVERSE: {
            // Roboter fährt BACKWARD_LOOPS = 222 Loops (4.44 s) rückwärts mit Trapez-Geschwindigkeitsprofil.
            // Trapez: Beschleunigung ACCEL_LOOPS + konstant + Bremsung BRAKE_LOOPS.
            // Alle Farb-Fallback-Werte werden verworfen (Farbsensor überfährt Aufkleber rückwärts).
            // Übergang → STATE_START_PAUSE nach vollständiger Rückwärtsfahrt.
            int   elapsed   = BACKWARD_LOOPS - m_backward_ctr;
            float ramp_up   = (elapsed < ACCEL_LOOPS)
                                  ? static_cast<float>(elapsed) / static_cast<float>(ACCEL_LOOPS)
                                  : 1.0f;
            float ramp_down = (m_backward_ctr < BRAKE_LOOPS)
                                  ? static_cast<float>(m_backward_ctr) / static_cast<float>(BRAKE_LOOPS)
                                  : 1.0f;
            float ramp = (ramp_up < ramp_down) ? ramp_up : ramp_down;
            float spd  = -VEL_SIGN * BACKWARD_SPEED * ramp;
            g_M1->setVelocity(spd);
            g_M2->setVelocity(spd);
            m_backward_ctr--;
            if (m_backward_ctr <= 0) {
                g_M1->setVelocity(0.0f);
                g_M2->setVelocity(0.0f);
                m_color_fallback    = 0;  // discard any colours read during backward travel
                m_approach_fallback = 0;
                m_pause_ctr = PAUSE_LOOPS;
                m_state     = STATE_START_PAUSE;
            }
            break;
        }

        // ====================================================================
        // HAUPTPROGRAMM
        // ====================================================================

        case STATE_START_PAUSE:
            // Roboter steht PAUSE_LOOPS = 20 Loops (0.4 s) still vor dem ersten Balken.
            // Farbsensor liest: falls Sensor über Aufkleber steht → m_approach_fallback setzen.
            // Initialisiert m_crossings_left = TOTAL_CROSSINGS = 4 für den Haupt-Loop.
            // Übergang → STATE_APPROACH nach Ablauf der Pause.
            g_M1->setVelocity(0.0f);
            g_M2->setVelocity(0.0f);
            {   // Fallback-Farbe merken falls Farbsensor direkt über Balken 1 steht
                int col_p = m_current_color;
                if (col_p == 3 || col_p == 4 || col_p == 5 || col_p == 7)
                    m_approach_fallback = col_p;
            }
            m_pause_ctr--;
            if (m_pause_ctr <= 0) {
                m_crossings_left = TOTAL_CROSSINGS;
                m_approach_ctr   = 0;
                m_state          = STATE_APPROACH;
            }
            break;

        case STATE_APPROACH: {
            // Roboter nähert sich dem ersten breiten Balken mit APPROACH_SPEED = 0.5 RPS (Trapez-Rampe).
            // Farbe wird kontinuierlich gelesen und als m_color_fallback gespeichert.
            // REAL_APPROACH_GUARD = 20 Loops: Ignore bars until robot has physically cleared the start bar.
            // POST_BAR_DRIVE_LOOPS = 9 Extra-Loops nach Balkenerkennung für bessere Sensorausrichtung.
            // Achtung: POST_BAR_DRIVE_LOOPS laufen ab BEVOR der Stopp-State gesetzt wird.
            // Übergang → STATE_STOP_AND_READ (m_small_line_mode=false) wenn Balken erkannt und abgefahren.
            float ramp = (m_approach_ctr < ACCEL_LOOPS)
                             ? static_cast<float>(m_approach_ctr) / static_cast<float>(ACCEL_LOOPS)
                             : 1.0f;
            g_M1->setVelocity(VEL_SIGN * APPROACH_SPEED * ramp);
            g_M2->setVelocity(VEL_SIGN * APPROACH_SPEED * ramp);
            m_approach_ctr++;

            {
                int col_a = m_current_color;
                if (col_a == 3 || col_a == 4 || col_a == 5 || col_a == 7) {
                    m_color_fallback = col_a;
                    m_target_angle   = carryAngleForColor(col_a);
                }
            }

            if (m_post_bar_ctr > 0) {
                m_post_bar_ctr--;
                if (m_post_bar_ctr == 0) {
                    g_M1->setVelocity(0.0f);
                    g_M2->setVelocity(0.0f);
                    if (m_counting) m_line_count++;
                    m_crossing_ctr     = CROSSING_STOP_LOOPS;
                    m_action_color     = 0;
                    m_color_stable_ctr = 0;
                    m_color_pending    = 0;
                    m_arm_retract_ctr  = 0;
                    m_small_line_mode  = false;
                    if (m_color_fallback == 0) m_color_fallback = m_approach_fallback;
                    if (m_color_fallback != 0) m_action_color = m_color_fallback;
                    m_state            = STATE_STOP_AND_READ;
                }
            } else if (m_approach_ctr > REAL_APPROACH_GUARD && wide_bar_active()) {
                m_post_bar_ctr = POST_BAR_DRIVE_LOOPS;
            }
            break;
        }

        case STATE_REAL_FOLLOW: {
            // Linienfolger für breite Balken. Geschwindigkeit: MAX_SPEED = 1.6 RPS.
            // Anlauframpe: RESTART_ACCEL_LOOPS = 50 Loops von 0 auf MAX_SPEED.
            // Farb-Erkennung während Fahrt: GRÜN/BLAU = 1 Messung, ROT/GELB = 3 Messungen.
            // Bei stabiler Farbmessung → Bremsrampe SLOWDOWN_LOOPS auf SLOW_FACTOR = 20%.
            // STOP_GUARD = 1.5 s Sperre nach Balken gegen sofortiges Wiedererkennen.
            // Übergang → STATE_STOP_AND_READ (m_small_line_mode=false) bei wide_bar_active().
            g_lf->setMaxWheelVelocity(MAX_SPEED);
            g_lf->setRotationalVelocityControllerGains(KP, KP_NL);

            if (m_real_accel_ctr < RESTART_ACCEL_LOOPS) {
                float ramp = static_cast<float>(m_real_accel_ctr + 1) /
                             static_cast<float>(RESTART_ACCEL_LOOPS);
                g_cmd_M1 = g_cmd_M2 = VEL_SIGN * MAX_SPEED * ramp;
                m_real_accel_ctr++;
            } else {
                if (m_color_delay_ctr > 0) {
                    m_color_delay_ctr--;
                    m_color_stable_ctr = 0;
                    m_color_pending    = 0;
                } else if (!m_slowing) {
                    int col_now = m_current_color;
                    bool is_action = (col_now == 3 || col_now == 4 ||
                                      col_now == 5 || col_now == 7);
                    if (is_action && col_now == m_color_pending) {
                        m_color_stable_ctr++;
                    } else if (is_action) {
                        m_color_pending    = col_now;
                        m_color_stable_ctr = 1;
                        m_color_fallback   = col_now;
                        m_target_angle     = carryAngleForColor(col_now);
                    } else {
                        m_color_pending    = 0;
                        m_color_stable_ctr = 0;
                    }
                    if (m_color_stable_ctr >= (m_color_pending == 5 || m_color_pending == 7 ? 1 : COLOR_STABLE_CNT)) {
                        m_slowing  = true;
                        m_slow_ctr = 0;
                    }
                }
                float slow_ramp = 1.0f;
                if (m_slowing) {
                    float t = (m_slow_ctr < SLOWDOWN_LOOPS)
                                  ? static_cast<float>(m_slow_ctr) / static_cast<float>(SLOWDOWN_LOOPS)
                                  : 1.0f;
                    slow_ramp = 1.0f - t * (1.0f - SLOW_FACTOR);
                    if (m_slow_ctr < SLOWDOWN_LOOPS) m_slow_ctr++;
                }
                g_cmd_M1 = VEL_SIGN * g_lf->getLeftWheelVelocity()  * slow_ramp;
                g_cmd_M2 = VEL_SIGN * g_lf->getRightWheelVelocity() * slow_ramp;
            }
            g_M1->setVelocity(g_cmd_M1);
            g_M2->setVelocity(g_cmd_M2);

            if (m_guard_ctr > 0) {
                m_guard_ctr--;
            } else if (wide_bar_active()) {
                g_M1->setVelocity(0.0f);
                g_M2->setVelocity(0.0f);
                if (m_counting) m_line_count++;
                m_crossing_ctr     = CROSSING_STOP_LOOPS;
                m_action_color     = 0;
                m_color_stable_ctr = 0;
                m_color_pending    = 0;
                m_arm_retract_ctr  = 0;
                m_small_line_mode  = false;
                if (m_color_fallback != 0) m_action_color = m_color_fallback;
                m_state            = STATE_STOP_AND_READ;
            }
            break;
        }

        case STATE_SMALL_FOLLOW: {
            // Linienfolger für schmale Querlinien. Gleiche Gains wie STATE_REAL_FOLLOW.
            // SMALL_REENTRY_GUARD = 2 s Sperre gegen Farbkarten-Stopp nach vorherigem Balken.
            // Nach 4. breitem Balken gesperrt für SMALL_FOLLOW_START_GUARD = ~9.8 s (Tunnel).
            // Farb-Erkennung identisch zu STATE_REAL_FOLLOW (GRÜN/BLAU=1, ROT/GELB=3).
            // Übergang → STATE_STOP_AND_READ (m_small_line_mode=true) bei small_line_active().
            g_lf->setMaxWheelVelocity(MAX_SPEED);
            g_lf->setRotationalVelocityControllerGains(KP, KP_NL);

            if (m_small_accel_ctr < RESTART_ACCEL_LOOPS) {
                float ramp = static_cast<float>(m_small_accel_ctr + 1) /
                             static_cast<float>(RESTART_ACCEL_LOOPS);
                g_cmd_M1 = g_cmd_M2 = VEL_SIGN * MAX_SPEED * ramp;
                m_small_accel_ctr++;
            } else {
                m_small_accel_ctr++;
                if (m_guard_ctr > 0 || m_color_delay_ctr > 0) {
                    if (m_color_delay_ctr > 0) m_color_delay_ctr--;
                    m_color_stable_ctr = 0;
                    m_color_pending    = 0;
                } else if (!m_slowing) {
                    int col_now = m_current_color;
                    bool is_action = (col_now == 3 || col_now == 4 ||
                                      col_now == 5 || col_now == 7);
                    if (is_action && col_now == m_color_pending) {
                        m_color_stable_ctr++;
                    } else if (is_action) {
                        m_color_pending    = col_now;
                        m_color_stable_ctr = 1;
                        m_color_fallback   = col_now;
                        m_target_angle     = carryAngleForColor(col_now);
                    } else {
                        m_color_pending    = 0;
                        m_color_stable_ctr = 0;
                    }
                    if (m_color_stable_ctr >= (m_color_pending == 5 || m_color_pending == 7 ? 1 : COLOR_STABLE_CNT)) {
                        m_slowing  = true;
                        m_slow_ctr = 0;
                    }
                }
                float slow_ramp = 1.0f;
                if (m_slowing) {
                    float t = (m_slow_ctr < SLOWDOWN_LOOPS)
                                  ? static_cast<float>(m_slow_ctr) / static_cast<float>(SLOWDOWN_LOOPS)
                                  : 1.0f;
                    slow_ramp = 1.0f - t * (1.0f - SLOW_FACTOR);
                    if (m_slow_ctr < SLOWDOWN_LOOPS) m_slow_ctr++;
                }
                g_cmd_M1 = VEL_SIGN * g_lf->getLeftWheelVelocity()  * slow_ramp;
                g_cmd_M2 = VEL_SIGN * g_lf->getRightWheelVelocity() * slow_ramp;
            }
            g_M1->setVelocity(g_cmd_M1);
            g_M2->setVelocity(g_cmd_M2);

            if (m_guard_ctr > 0) {
                m_guard_ctr--;
            } else if (small_line_active()) {
                g_M1->setVelocity(0.0f);
                g_M2->setVelocity(0.0f);
                if (m_counting) m_line_count++;
                m_crossing_ctr       = SMALL_CROSSING_STOP_LOOPS;
                m_action_color       = 0;
                m_color_stable_ctr   = 0;
                m_color_pending      = 0;
                m_arm_retract_ctr    = 0;
                m_small_line_mode    = true;
                if (m_color_fallback != 0) m_action_color = m_color_fallback;
                m_state              = STATE_STOP_AND_READ;
            }
            break;
        }

        case STATE_TUNNEL_FOLLOW: {
            // Drei-Phasen-Sequenz durch den Tunnel:
            //   Phase A (0–100 loops = 2 s): Normaler Linienfolger mit Anlauframpe (50 loops)
            //   Phase B (100–175 loops = 1.5 s): Geradeaus ohne Lenken (kein Liniensignal im Tunnel)
            //   Phase C (>175 loops): Normaler Linienfolger bis zur nächsten Schmallinie
            // Achtung: m_small_accel_ctr zählt hier kontinuierlich hoch (kein Reset zwischen Phasen).
            // Übergang → STATE_STOP_AND_READ (m_small_line_mode=true) bei small_line_active().
            g_lf->setMaxWheelVelocity(MAX_SPEED);
            g_lf->setRotationalVelocityControllerGains(KP, KP_NL);

            m_small_accel_ctr++;

            if (m_small_accel_ctr <= TUNNEL_FOLLOW_LOOPS) {
                // Phase A: Linienfolger mit Anlauframpe
                float ramp = (m_small_accel_ctr <= RESTART_ACCEL_LOOPS)
                                 ? static_cast<float>(m_small_accel_ctr) / static_cast<float>(RESTART_ACCEL_LOOPS)
                                 : 1.0f;
                g_cmd_M1 = VEL_SIGN * g_lf->getLeftWheelVelocity()  * ramp;
                g_cmd_M2 = VEL_SIGN * g_lf->getRightWheelVelocity() * ramp;
            } else if (m_small_accel_ctr <= TUNNEL_FOLLOW_LOOPS + TUNNEL_STRAIGHT_LOOPS) {
                // Phase B: geradeaus, kein Linienfolger
                g_cmd_M1 = VEL_SIGN * MAX_SPEED;
                g_cmd_M2 = VEL_SIGN * MAX_SPEED;
            } else {
                // Phase C: normaler Linienfolger
                g_cmd_M1 = VEL_SIGN * g_lf->getLeftWheelVelocity();
                g_cmd_M2 = VEL_SIGN * g_lf->getRightWheelVelocity();
            }
            g_M1->setVelocity(g_cmd_M1);
            g_M2->setVelocity(g_cmd_M2);

            if (m_guard_ctr > 0) {
                m_guard_ctr--;
            } else if (small_line_active()) {
                g_M1->setVelocity(0.0f);
                g_M2->setVelocity(0.0f);
                if (m_counting) m_line_count++;
                m_crossing_ctr       = SMALL_CROSSING_STOP_LOOPS;
                m_action_color       = 0;
                m_color_stable_ctr   = 0;
                m_color_pending      = 0;
                m_arm_retract_ctr    = 0;
                m_small_line_mode    = true;
                if (m_color_fallback != 0) m_action_color = m_color_fallback;
                m_state              = STATE_STOP_AND_READ;
            }
            break;
        }

        case STATE_STOP_AND_READ: {
            // Dispatch-State für alle Querbalken — breit (m_small_line_mode=false) und schmal (=true).
            // Roboter steht still und liest Farbe via tryLockColor() (1 stabile Messung genügt).
            // m_crossing_ctr Countdown: breit = CROSSING_STOP_LOOPS (250), schmal = SMALL_CROSSING_STOP_LOOPS (100).
            // Fallback nach Timeout: m_color_fallback (aus Fahr-Farberkennung) wird als action_color übernommen.
            // Übergang → STATE_RED / STATE_GREEN / STATE_BLUE / STATE_YELLOW je nach Farbe.
            //            Bei unbekannter Farbe → exitWideBar() oder exitSmallLine() (je nach m_small_line_mode).
            g_M1->setVelocity(0.0f);
            g_M2->setVelocity(0.0f);

            if (m_action_color == 0) {
                if (tryLockColor(m_current_color, m_color_pending, m_color_stable_ctr,
                                 COLOR_STOP_STABLE_CNT)) {
                    m_action_color = m_color_pending;
                    setNeoColor(m_action_color);
                }
            }

            if (m_crossing_ctr > 0) m_crossing_ctr--;

            // Fallback bei Timeout (nach Dekrement, damit der letzte Loop greift)
            if (m_crossing_ctr <= 0 && m_action_color == 0) {
                m_action_color = m_color_fallback;
                if (m_action_color != 0) setNeoColor(m_action_color);
            }

            if (m_action_color != 0 || m_crossing_ctr <= 0) {
                m_deliver_phase    = 0;
                m_deliver_ctr      = 0;
                m_forward_ctr      = 0;
                m_color_stable_ctr = 0;
                m_color_pending    = 0;
                m_color_fallback   = 0;
                m_tray_wait_ctr    = 0;
                if (m_action_color != 0) {
                    m_neo_held_color = m_action_color;
                    m_neo_mode       = NEO_COLOR;
                }
                switch (m_action_color) {
                    case 3: m_state = STATE_RED;    break;
                    case 4: m_state = STATE_YELLOW; break;
                    case 5: m_state = STATE_GREEN;  break;
                    case 7: m_state = STATE_BLUE;   break;
                    default:
                        m_slowing = false; m_slow_ctr = 0; m_action_color = 0;
                        m_color_delay_ctr = COLOR_READ_DELAY;
                        if (m_small_line_mode) exitSmallLine(); else exitWideBar();
                        break;
                }
            }
            break;
        }

        case STATE_RED:
        case STATE_GREEN:
        case STATE_BLUE:
        case STATE_YELLOW: {
            // Ablage-Sequenz für alle 4 Farben (m_action_color bestimmt Tray-Winkel und D2-Tiefe).
            // ROT (0°) + GELB (270°): Vorwärtsfahrt ROT_GELB_DRIVE_LOOPS = 51 Loops auf Linie, dann Arm.
            //   Nach letztem Balken (line_count 4 oder 8): Rückfahrt m_reverse_ctr über selbe Distanz.
            // GRÜN (90°) + BLAU (180°): Kein Fahren — direkt auf Tray warten, dann Arm-Sequenz.
            // Tray-Warten: bereit nach ≥50 Loops ODER ≥25 Loops + trayNearTarget(20°) — verhindert
            //   Blockade bei Servo-Fehler. Tray ist via Pre-Adjust schon am Carry-Winkel (base–30°).
            // Übergang → exitWideBar() / exitSmallLine() nach Abschluss von runDeliverPhase().
            bool needs_drive = (m_action_color == 3 || m_action_color == 4);
            bool is_last_bar = (m_line_count == 4 || m_line_count == 8);

            if (m_reversing) {
                // Rückfahrt nach letztem ROT/GELB-Balken mit Trapez-Rampe
                int   bd = m_reverse_ctr;
                int   bl = ROT_GELB_DRIVE_LOOPS - bd;
                float ramp;
                if      (bd < ROT_GELB_ACCEL_LOOPS)  ramp = static_cast<float>(bd + 1) / ROT_GELB_ACCEL_LOOPS;
                else if (bl <= ROT_GELB_BRAKE_LOOPS)  ramp = static_cast<float>(bl)     / ROT_GELB_BRAKE_LOOPS;
                else                                   ramp = 1.0f;
                float v = -VEL_SIGN * MAX_SPEED * ramp;
                g_M1->setVelocity(v);
                g_M2->setVelocity(v);
                m_reverse_ctr++;
                if (m_reverse_ctr >= ROT_GELB_DRIVE_LOOPS) {
                    g_M1->setVelocity(0.0f);
                    g_M2->setVelocity(0.0f);
                    resetArmExitVars();
                    if (m_small_line_mode) exitSmallLine(); else exitWideBar();
                }
            } else if (needs_drive && m_forward_ctr < ROT_GELB_DRIVE_LOOPS) {
                // Vorwärtsfahrt zum richtigen Ablage-Slot (ROT/GELB liegen seitlich)
                g_lf->setMaxWheelVelocity(MAX_SPEED);
                g_lf->setRotationalVelocityControllerGains(KP, KP_NL);
                int   ld = m_forward_ctr;
                int   ll = ROT_GELB_DRIVE_LOOPS - ld;
                float ramp;
                if      (ld < ROT_GELB_ACCEL_LOOPS) ramp = static_cast<float>(ld + 1) / ROT_GELB_ACCEL_LOOPS;
                else if (ll <= ROT_GELB_BRAKE_LOOPS) ramp = static_cast<float>(ll)     / ROT_GELB_BRAKE_LOOPS;
                else                                  ramp = 1.0f;
                if (m_small_line_mode) {
                    g_cmd_M1 = g_cmd_M2 = VEL_SIGN * MAX_SPEED * ramp;
                } else {
                    g_cmd_M1 = VEL_SIGN * g_lf->getLeftWheelVelocity()  * ramp;
                    g_cmd_M2 = VEL_SIGN * g_lf->getRightWheelVelocity() * ramp;
                }
                g_M1->setVelocity(g_cmd_M1);
                g_M2->setVelocity(g_cmd_M2);
                m_forward_ctr++;
            } else {
                // Arm-Sequenz: warten bis Tray bereit, dann runDeliverPhase()
                g_M1->setVelocity(0.0f);
                g_M2->setVelocity(0.0f);
                // Tray ist via pre-adjust schon zum Korrekturwinkel (base-30°) unterwegs.
                m_tray_wait_ctr++;
                bool tray_ready = (m_tray_wait_ctr >= 50) ||
                                  (m_tray_wait_ctr >= 25 && trayNearTarget(20.0f));
                if (tray_ready) {
                    if (runDeliverPhase()) {
                        m_neo_held_color = m_action_color;
                        if (m_line_count == 8) {
                            m_neo_mode  = NEO_DRIVE_HOLD;
                            m_neo_timer = 0;
                        } else if (m_line_count == 4) {
                            m_neo_mode  = NEO_DRIVE_HOLD;
                            m_neo_timer = 0;
                        } else {
                            m_neo_mode = NEO_COLOR;
                        }
                        if (needs_drive && is_last_bar) {
                            m_reversing   = true;
                            m_reverse_ctr = 0;
                        } else {
                            resetArmExitVars();
                            if (m_small_line_mode) exitSmallLine(); else exitWideBar();
                        }
                    }
                }
            }
            break;
        }

        case STATE_FINISH:
            // Roboter hält permanent an. Tray-Servo wird gestoppt und deaktiviert.
            // NeoPixel läuft weiter (NEO_DRIVE_HOLD → NEO_OFF_WAIT → NEO_CYCLE via updateNeoPixel).
            // Dieser State wird nie verlassen — nur ein Reset startet den Ablauf neu.
            g_M1->setVelocity(0.0f);
            g_M2->setVelocity(0.0f);
            g_servoTray->stop();
            g_servoTray->disable();
            break;
    }
}

// ===========================================================================
// Reset
// ===========================================================================
void cargosweep_team5_20260511_reset(DigitalOut& led)
{
    *g_en = 0;
    g_M1->setVelocity(0.0f);
    g_M2->setVelocity(0.0f);
    g_cmd_M1 = g_cmd_M2 = 0.0f;
    if (g_servo_D1)  g_servo_D1->disable();
    if (g_servo_D2)  g_servo_D2->disable();
    if (g_servoTray) { g_servoTray->stop(); g_servoTray->disable(); }
    if (g_np)        { g_np->setRGB(0,0,0); g_np->show(); }
    m_neo_mode       = NEO_CYCLE;
    m_neo_timer      = 0;
    m_neo_held_color = 0;

    m_state               = STATE_ALIGN_START;
    m_sf360_warmup_ctr    = 0;
    m_sf360_ready         = false;
    m_tray_moving         = false;
    m_target_angle        = 0.0f;
    m_straight_ctr        = 0;
    m_follow_ctr          = 0;
    m_brake_ctr           = 0;
    m_pause_ctr           = 0;
    m_backward_ctr        = 0;
    m_guard_ctr           = 0;
    m_crossing_ctr        = 0;
    m_crossings_left      = 0;
    m_small_crossings_left = 0;
    m_approach_ctr        = 0;
    m_small_accel_ctr     = 0;
    m_real_accel_ctr      = 0;
    m_arm_retract_ctr     = 0;
    m_deliver_phase       = 0;
    m_deliver_ctr         = 0;
    m_forward_ctr         = 0;
    m_post_bar_ctr        = 0;
    m_arm_prep_phase      = 0;
    m_tray_wait_ctr       = 0;
    m_brake_start_M1      = 0.0f;
    m_brake_start_M2      = 0.0f;
    m_current_color       = 0;
    m_prev_color          = 0;
    m_led_color           = 0;
    m_led_ctr             = 0;
    m_color_log_ctr       = 0;
    m_action_color        = 0;
    m_slowing             = false;
    m_slow_ctr            = 0;
    m_color_delay_ctr     = 0;
    m_color_stable_ctr    = 0;
    m_color_pending       = 0;
    m_color_fallback      = 0;
    m_approach_fallback   = 0;
    m_home_timer          = 0;
    m_startup_delay       = 150;
    for (int i = 0; i < 8; i++) m_color_log[i] = 0;
    led = 0;
}

// ===========================================================================
// Print (~5 Hz — zeigt aktuellen Zustand im Serial Monitor)
// ===========================================================================
void cargosweep_team5_20260511_print()
{
    const char* s =
        (m_state == STATE_ALIGN_START)   ? "ALIGN_START  " :
        (m_state == STATE_ALIGN_STRAIGHT)? "ALIGN_STR    " :
        (m_state == STATE_ALIGN_TURN)    ? "ALIGN_TURN   " :
        (m_state == STATE_ALIGN_FOLLOW)  ? "ALIGN_FOLLOW " :
        (m_state == STATE_ALIGN_BRAKE)   ? "ALIGN_BRAKE  " :
        (m_state == STATE_ALIGN_REVERSE) ? "ALIGN_REV    " :
        (m_state == STATE_START_PAUSE)   ? "START_PAUSE  " :
        (m_state == STATE_APPROACH)      ? "APPROACH     " :
        (m_state == STATE_REAL_FOLLOW)   ? "REAL_FOLLOW  " :
        (m_state == STATE_STOP_AND_READ) ? "STOP_AND_READ" :
        (m_state == STATE_SMALL_FOLLOW)  ? "SMALL_FOLLOW " :
        (m_state == STATE_TUNNEL_FOLLOW) ? "TUNNEL_FOLLOW" :
        (m_state == STATE_RED)           ? "RED          " :
        (m_state == STATE_GREEN)         ? "GREEN        " :
        (m_state == STATE_BLUE)          ? "BLUE         " :
        (m_state == STATE_YELLOW)        ? "YELLOW       " : "FINISH       ";

    float tray_angle = g_servoTray ? g_servoTray->getCurrentAngle() : 0.0f;
    printf("State: %s ph=%d dr=%d | wide=%d small=%d | tray=%.1f/%.1f | col=%-5s act=%-5s\n",
           s,
           m_deliver_phase,
           m_forward_ctr,
           m_crossings_left,
           m_small_crossings_left,
           tray_angle,
           m_target_angle,
           ColorSensor::getColorString(m_current_color),
           ColorSensor::getColorString(m_action_color));
}

#endif // CARGOSWEEP_TEAM5_20260511
