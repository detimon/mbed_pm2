#pragma once
// ════════════════════════════════════════════════════════════
// FILE   : calibration_values.h
// PROJEKT: LineXpress — ZHAW SoE PM2 FS26
// ZWECK  : Zentrale Kalibrierungskonstanten. Hier und NUR hier
//          werden Werte angepasst. Alle anderen Dateien
//          inkludieren diesen Header.
// UPDATE : Datum + Initialen bei jeder Kalibrierung eintragen!
// ════════════════════════════════════════════════════════════

// ── FARBSENSOR TCS3200 (B4) — Hue-Schwellwerte ──────────────
// OUT=PB3 | LED=PC3 | S0=PA1 | S1=PA4 | S2=PC1 | S3=PC0
// Einheit: Grad Hue (0–360°), nach Weissabgleich gemessen
// Werte aus lib/ColorSensor/ColorSensor.cpp (Stand 2026-05-12).
// Diese Datei dokumentiert die aktiven Grenzen. Anwendung in
// calib_color_hue.cpp; lib/ColorSensor bleibt eigenständig.
// Kalibriert: 2026-05-06 [Team5]

constexpr float HUE_RED_MIN     = 0.0f;     // wrap: 0°
constexpr float HUE_RED_MAX     = 15.0f;
constexpr float HUE_YELLOW_MIN  = 25.0f;
constexpr float HUE_YELLOW_MAX  = 50.0f;
constexpr float HUE_GREEN_MIN   = 65.0f;
constexpr float HUE_GREEN_MAX   = 125.0f;
constexpr float HUE_BLUE_MIN    = 210.0f;
constexpr float HUE_BLUE_MAX    = 255.0f;
constexpr float HUE_TOLERANCE   = 5.0f;     // empfohlener Puffer zwischen Zonen

// Druckanzeige in calib_color_hue.cpp
constexpr int   HUE_DISPLAY_LINES = 10;     // Zeilen im ANSI-Refresh-Block
constexpr int   HUE_PRINT_EVERY_N = 25;     // ~500 ms zwischen Refreshes

// ── PARALLAX 360° SERVO (M22) ────────────────────────────────
// PWM=PB12 (D3) | Feedback=PC2 (A0, 910 Hz)
// 0.0=voll rückwärts | 0.5=Stopp | 1.0=voll vorwärts
// Kalibriert: 2026-04-27 [andre]

constexpr float SERVO_360_STOP      = 0.5f;
constexpr float SERVO_360_FWD_FAST  = 0.40f;   // Test-Wert; bei Bedarf nachmessen
constexpr float SERVO_360_FWD_SLOW  = 0.52f;   // CARGOSWEEP align speed
constexpr float SERVO_360_BWD_FAST  = 0.60f;
constexpr float SERVO_360_BWD_SLOW  = 0.48f;
constexpr float SERVO_360_DEADBAND  = 0.02f;   // Totband um STOP

// P-Regler-Tuning (calib_servo_360.cpp)
constexpr float SERVO_360_KP            = 0.005f;  // P-Verstärkung
constexpr float SERVO_360_TOLERANCE_DEG = 2.1f;    // ±° Zielband
constexpr float SERVO_360_MIN_SPEED     = 0.01f;   // Mindest-Speed nahe Ziel
constexpr float SERVO_360_ANGLE_OFFSET  = 10.0f;   // mechanischer Nullpunkt-Offset

// Sequenz-Steuerung (calib_servo_360.cpp)
constexpr int   SERVO_360_PAUSE_LOOPS   = 50;      // 1 s zwischen Targets
constexpr int   SERVO_360_WARMUP_LOOPS  = 25;      // 500 ms Feedback-Stabilisierung
constexpr int   SERVO_360_TIMEOUT_LOOPS = 250;     // 5 s Ziel aufgeben
constexpr int   SERVO_360_PRINT_EVERY   = 10;      // 200 ms Serial-Refresh

// ── 180° SERVOS ──────────────────────────────────────────────
// M21 vertikal   : PB2 (D0)
// M20 horizontal : PC8 (D1)
// Kalibriert: 2026-03-26 [andre]

constexpr float SERVO_180_MIN_PULSE = 0.0303f;   // gemeinsamer unterer Anschlag
constexpr float SERVO_180_MAX_PULSE = 0.1204f;   // Servo A (horizontal)
constexpr float SERVO_180_ARM_V_HOME = 1.0f;     // normierte Home-Position M21
constexpr float SERVO_180_ARM_H_HOME = 0.0f;     // normierte Home-Position M20

// Sweep-Test-Parameter (calib_servo_180.cpp)
constexpr float SERVO_180_PULSE_LOW   = 0.010f;
constexpr float SERVO_180_PULSE_HIGH  = 0.150f;
constexpr int   SERVO_180_WAIT_LOOPS  = 50;      // 1 s bei 50 Hz
constexpr int   SERVO_180_SWEEP_DIV   = 4;       // alle 4 Loops ein Schritt
constexpr float SERVO_180_SWEEP_STEP  = 0.001f;
constexpr int   SERVO_180_GOTO_LOOPS  = 10;      // ~200 ms Anfahrwartezeit
constexpr int   SERVO_180_PRINT_EVERY_N = 5;     // ~100 ms zwischen Refreshes
constexpr int   SERVO_180_DISPLAY_LINES = 9;
