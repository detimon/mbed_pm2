# mbed_pm — PES Board Roboter (Nucleo F446RE)

## Aktueller Stand
_Wird am Ende jeder Session via `/sesh-end` aktualisiert._ (2026-05-06 Session 2)
- **`PROTOTYPE_03_V35_04_02` aktiv** in `test_config.h`. V34_04_01 als Backup.
<<<<<<< HEAD
- **Schmale Balken (STATE_SMALL_ARM_*): funktionieren perfekt** — alle 4 Farben, Ablage-Sequenz zuverlässig.
- **Breite Balken (STATE_ARM_*): 50/50 Problem beim ersten Balken** — wenn Ausrichtungsprogramm (STATE_BLIND) vorher läuft, macht der Arm keinen Jiggle; ohne Ausrichtung manchmal OK.
- **GELB breite Balken: Probleme** — entweder Farblesung oder Jiggle fehlerhaft (noch nicht isoliert).
- **Fixes dieser Session:**
  - Fallback-Counter-Bug in `STATE_CROSSING_STOP` + `STATE_SMALL_CROSSING_STOP` behoben: Dekrement VOR Fallback-Check (vorher: letzter Loop griff nie)
  - `REAL_APPROACH_GUARD = 20` (war `ACCEL_LOOPS=12` → zu früh, dann `50` → überfuhr Balken 1)
  - `STATE_REAL_START_PAUSE` liest jetzt auch Farbe → `m_approach_fallback` (falls Farbsensor über Aufkleber steht)
- **Root-Cause erster Balken (offen):** Farbsensor (vorne) ist geometrisch bereits am farbigen Aufkleber von Balken 1 vorbei wenn REAL_APPROACH startet. `m_approach_fallback = 0`. Im CROSSING_STOP-Fenster liest Sensor Matte. Beide Fallbacks = 0 → Default-Case → kein ARM-State → kein Jiggle.
=======
- **V35_04_02 Änderungen gegenüber V34_04_01 (heute getestet):**
  - `SF360_TIMEOUT_LOOPS=60` (war 30) — Tray hat 1.2s zur Drehung in Phase 0 Exit
  - `CROSSING_STOP_LOOPS=150` (war 100) — Color-Reading-Fenster bei wide bars 3s statt 2s
  - `REAL_APPROACH` setzt `m_color_fallback` direkt (statt Umweg über `m_approach_fallback`) — bar 1 jetzt konsistent zu bars 2-4
  - `STATE_CROSSING_STOP` body identisch zu `SMALL_CROSSING_STOP` — keine `m_approach_fallback`-Sonderlogik mehr
  - **`runDeliverPhase()` Phase 0 robuster:** kein quick-timeout mehr; wartet auf `isAtTarget()`; periodischer "kick" alle 25 loops (forciert `enable() + moveToAngle()`); Safety-Notfall nach 500 loops (10s)
- **STATUS: 3 von 4 breiten Balken funktionieren** — BLAU jiggelt jetzt ZUM ERSTEN MAL am ersten breiten Balken zuverlässig. GRÜN und ROT auch OK.
- **Schmale Balken (STATE_SMALL_ARM_*) funktionieren perfekt** — unverändert.
- **OFFEN: GELB beim 3. breiten Balken jiggelt nicht** — vermutlich erreicht Safety-Notfall (10s) nach Tray-Drehung von 270°. Kick-Mechanismus reicht nicht für längste Drehung.
>>>>>>> a7472cd667993ad4c9bef5386ae190df239b7cfe

## Stack
- Sprache: C++14
- Framework: mbed OS (ARM Mbed)
- Build-Tool: PlatformIO (`pio`)
- Board: STM32 Nucleo F446RE + PES Board (ZHAW PM2, 2. Semester)
- Upload: ST-Link (`upload_protocol = stlink`)
- Formatter: clang-format (Konfig: `.clang-format`)

## Commands
- Build:   `pio run`
- Flash:   `pio run --target upload`
- Monitor: `pio device monitor` (115200 baud)
- Clean:   `pio run --target clean`
- Format:  `clang-format -i src/*.cpp src/*.h lib/**/*.cpp lib/**/*.h`

## Architektur
Modulares Test-Framework für einen zweimotorigen Differentialantrieb-Roboter. Genau ein Test wird per `#define` in `src/test_config.h` aktiviert; jedes Modul implementiert vier Funktionen: `_init()`, `_task()`, `_reset()`, `_print()`. Wiederverwendbare Hardware-Treiber leben in `lib/` (DCMotor, LineFollower, ColorSensor, Servo, SensorBar usw.), Board-Pin-Definitionen in `include/PESBoardPinMap.h`. Main-Loop läuft mit 20 ms Periode (50 Hz).

## Code-Konventionen
- clang-format LLVM-Stil: 4 Spaces, kein Tab, max. 120 Zeichen/Zeile
- `{` bei Funktionen/Klassen auf neuer Zeile; bei Control-Statements auf selber Zeile
- Includes: System-Headers `<...>` vor lokalen `"..."`, sortiert
- Klassen-Member: `m_`-Präfix (z.B. `m_velocity`, `m_Kp`)
- Globale Singletons in Test-Files: `g_`-Präfix (z.B. `g_M1`, `g_lf`)
- Float-Literale immer mit `f`-Suffix: `0.0f`, `3.14f`
- Geschwindigkeit immer in RPS (Rotations Per Second)
- Static-lokale Objekte für Hardware in `_init()` (kein Heap-new)

## Verboten
- `TEST_LINE_FOLLOWER_FAST`-Parameter ohne ausdrückliche Anfrage ändern — er läuft stabil ("nobody touches the program", Commit 183603b)
- PB_9 oder PB_10 als DigitalOut für LED verwenden — `led1` ist entfernt, `user_led` (LED1) übernimmt überall
- `TIM1->BDTR |= TIM_BDTR_MOE` entfernen — MOE muss manuell gesetzt werden (STM32-Hardware-Requirement)
- `PERFORM_GPA_MEAS` und `PERFORM_CHIRP_MEAS` gleichzeitig aktivieren (DCMotor.h: "only use GPA or Chirp, not both")
- Mehr als ein `#define` in `test_config.h` gleichzeitig aktiv lassen

## Pin-Layout (vollständig)

| Pin | Mbed Name | Komponente |
|-----|-----------|------------|
| D1  | PC_8      | 180° Servo A |
| D2  | PC_6      | 180° Servo B |
| D3  | PB_12     | 360° Servo |
| A0  | PC_2      | Endschalter 360°-Servo (Drehteller) |
| A3  | PB_1      | IR-Sensor |
| D14/D15 | PB_9 / PB_8 | Line-Follower Array (I2C SDA/SCL) |
| —   | PB_3      | Farbsensor – Freq Out |
| —   | PC_3      | Farbsensor – LED Enable |
| —   | PA_4      | Farbsensor – S0 |
| —   | PB_0      | Farbsensor – S1 |
| —   | PC_1      | Farbsensor – S2 |
| —   | PC_0      | Farbsensor – S3 |

## Aktive Entscheidungen
- **ServoFeedback360-Library** (2026-04-27): `lib/ServoFeedback360/` — P-Regler mit PwmIn-Feedback. Konstruktor-Parameter: `(pwm_pin, fb_pin, kp, tolerance_deg, min_speed, angle_offset)`. PwmOut NICHT auf PB_12 verwenden (kein HW-PWM). Test-Modul: `TEST_PARALLAX_360`.
- **v21 ist aktive Hauptversion** (2026-04-22) — Kopie v20 als neuer Arbeits-Zweig, test_config.h auf `PROTOTYPE_02_V21`. v20 bleibt als Backup, wird nicht mehr editiert
- v21 (2026-04-22): `SMALL_FOLLOW_START_GUARD` Basis 706 → 781 (+1.5 s Blind-Sperre nach 4. Querbalken gegen Farb-/Balken-Trigger in der Kurve)
- v21 (2026-04-22 Session 2): STATE_BLIND setzt `m_click_target=3` → 3× Endschalter-Überfahrt bei Initial-Ausrichtung (nutzt bestehende Click-Counting-ISR)
- v21 (2026-04-22 Session 2): D2-Ablagetiefen ~4 mm flacher — `SERVO_D2_BLAU_DOWN=0.37f`, `SERVO_D2_GRUEN_DOWN=0.16f`. Faustregel: +0.01 Puls ≈ 1.3 mm
- v21 (2026-04-22 Session 2): ROT-Jiggle in ROT_GELB_PAUSE flippt nur an breiten Balken — `jiggle_rev = rev ^ (m_rot_gelb_color == 3 && !m_rot_gelb_is_small)`. An Schmallinien bleibt Standard-Richtung
- v21 (2026-04-22 Session 2): CROSSING_STOP Jiggle 2 als **Doppelimpuls** (6+3+6 Loops) statt 11-Loop Einzelimpuls — nur GRÜN/BLAU an breiten Balken. Schmallinien + ROT_GELB_PAUSE unverändert
- v21 (2026-04-22 Session 2): Finale Drehung nach letztem breitem Balken jetzt **5-Click** statt 90° — in CROSSING_STOP bedingungslos, in ROT_GELB_PAUSE via `do_5click = !m_rot_gelb_is_small`
- v21 (2026-04-22 Session 2): Zwei latente Syntax-Bugs aus v20 entfernt (Platzhalter-`if {` ohne `}`) in STATE_CROSSING_STOP + STATE_SMALL_CROSSING_STOP — wurden nie bemerkt weil v20 nie kompiliert wurde
- Claude Code Modell: `opus[1m]` in `~/.claude/settings.json` — bei Opus-Rate-Limit temporär auf `sonnet` wechseln (Opus-Sublimit ist unabhängig vom Gesamt-Limit)
- `/popupssound` toggled Sound via Flag-Datei `C:\Users\alexa\.claude\sound_disabled` — alle drei Popup-Scripts prüfen dieses Flag
- `/popups`-Befehl toggled Popups via Flag-Datei `C:\Users\alexa\.claude\popups_disabled` — kein settings.json-Edit nötig
- Oranges Popup via `PermissionRequest`-Hook → `permission_request.ps1` (erstellt Flag + fokussiert VSCode + startet Popup)
- Popup-Dismiss via `PostToolUse`-Hook → `dismiss_orange.ps1` (löscht Flag → Popup schliesst in <100ms)
- `PreToolUse` feuert VOR der Bestätigung — für Dismiss muss `PostToolUse` verwendet werden
- Grünes Popup: kein Timeout, schliesst nur bei Maus/Tastatur; Stimme 3x (sofort, +20s, +40s)
- Alle Popups: `WaitUntilDone(-1)` nach `ShowDialog()` — Stimme spricht zu Ende auch nach frühem Dismiss
- VSCode-Fokus via `AttachThreadInput` in `focus_vscode.ps1` — funktioniert aus nicht-fokussiertem Prozess
- `PROTOTYPE_02_V16` ist aktive Hauptversion (v15 Backup) — test_config.h aktuell auf TEST_SERVO_ALL (zurückstellen!)
- `led1` (PB_9/PB_10) aus `main.cpp` entfernt (2026-04-13) — `user_led` (LED1) übernimmt in allen Test-Funktionen
- Endschalter 360°-Servo: **A0 (PC_2)**, `DigitalIn` PullUp — in v11 integriert, stoppt 360° bei `read()==0` (korrigiert 2026-04-19, vorher fälschlich PC_5)
- roboter_v10: SMALL_REENTRY_GUARD=100 (2s) verhindert Stopp auf Farbkarte statt echtem Querbalken
- `src/test_files/` Unterordner erstellt — alle `test_*.cpp/.h` darin; Roboter-Files in `src/` als `prototype01_vX.cpp/.h` (v1–v8) und `prototype02_vX.cpp/.h` (v9–v10)
- Hue-Grenze GRÜN (2026-04-16): **180°** (vorher 215°, ursprünglich 175°) — Senkung nötig, weil WHITE-Hue bei 204°–210° sonst als GRÜN klassifiziert wurde. 37° Puffer über gemessenem GRÜN-Max (143°). Falls GRÜN wieder als BLAU gelesen wird → hochsetzen (195°).
- Kalibrierung mit echter Matte NICHT machen ohne gleichzeitige Hue-Grenzen-Anpassung — verschiebt alle Werte
- Servo-Kalibrierung abgeschlossen (2026-03-26): A=(0.0303–0.1204), B=(0.0314–0.1232), 360°=(0.0303–0.1223, Stop=0.0763)
- `TEST_LINE_FOLLOWER_FAST` ist stabil und funktioniert (Commit 183603b) — nicht anfassen
- Gear Ratio: 100:1 verbaut (`USE_GEAR_RATIO_78 = false`), KN = 140/12 ≈ 11.67 rpm/V
- v10: `VEL_SIGN=1.0f`, M1=physisch rechts, M2=physisch links (nach physischem Motortausch)
- v10: Linienfolgezuweisung gekreuzt: `g_cmd_M1=getLeftWheelVelocity()`, `g_cmd_M2=getRightWheelVelocity()` (Sensorbalken umgekehrt montiert)
- v10: STATE_FOLLOW Eintritt mit niedrigen Gains (KP_FOLLOW=0.8f, KP_NL_FOLLOW=0.5f), STATE_REAL/SMALL_FOLLOW stellen KP=2.8/KP_NL=4.55 wieder her
- SensorBar I2C1: SDA = PB_9, SCL = PB_8
- Eigen-Lib: `EIGEN_NO_DEBUG` + `EIGEN_DONT_VECTORIZE` (Overhead-Reduktion auf MCU)
- v10 (2026-04-15): Farbbremsrampe — wer bei `col ∈ {3,4,5,7}` bremst, nutzt `SLOWDOWN_LOOPS=25` / `SLOW_FACTOR=0.4f` mit Debouncing `COLOR_STABLE_CNT=5` und Nach-Stopp-Sperre `COLOR_READ_DELAY=50` (1 s)
- v11 (2026-04-18): Farbbremsrampe — `SLOWDOWN_LOOPS=13` / `SLOW_FACTOR=0.2f` (20%), `COLOR_STABLE_CNT=3`, `COLOR_READ_DELAY=50`
- v10 (2026-04-15): MAX_SPEED=1.6, CROSSING_STOP_LOOPS=100, SMALL_CROSSING_STOP_LOOPS=100, SERVO_ROT_LOOPS=75, STRAIGHT_LOOPS=70, BACKWARD_LOOPS=222, SMALL_FOLLOW_START_GUARD-Basis=656
- v11 (2026-04-18): SMALL_FOLLOW_START_GUARD-Basis=706, RESTART_ACCEL_LOOPS=50, wide_bar_active() ab 4/8 Sensoren
- v11 (2026-04-18): Angle Clamp in LineFollower — MAX_ANGLE_STEP=0.15f rad/Loop, `#define ANGLE_CLAMP_ENABLED` zum Deaktivieren auskommentieren
- v11 (2026-04-18): BLAU Servo-Sequenz: D1 ausfahren(0.2) → D2 senken(0.8) → D2 hoch(1.0) → D1 ein(0.0) → 360° langsam 10 Loops. D1: 0.0=ein, 1.0=aus. D2: 1.0=oben, 0.0=unten.
- v11 (2026-04-18): Servo-Ausrichtung bei Programmstart (alle 8 Sensoren aktiv): D1→0.0, D2→1.0, 360°→0.80 bis Endschalter
- NeoPixel (2026-04-16): WS2812B Adafruit Flora v2 an PB_2 (D0), 5V, Bit-Bang-Treiber in `lib/NeoPixel/` (DWT-Cycle-Counter + critical_section). GRB-Byte-Order. Test-Module: `TEST_NEOPIXEL` (Zyklus-Selbsttest) und `TEST_COLOR_NEOPIXEL` (Live-Validierung Farbsensor).
- Hue-Referenz (2026-04-16, weisses Papier, LED an): WHITE 204°–210°, BLAU 226°–232°, GRÜN 133°–143°, ROT 359°–1.5°, GELB 43.5°–44.5° — Quelle bei künftigen Hue-Grenzen-Entscheidungen (**obsolet seit 2026-04-19**, neue Halterung siehe unten).
- **Hue-Referenz (2026-04-19, neue Sensor-Halterung):** BLAU 348°–7° (wrap), ROT 7°–20°, WHITE ~30° (→ UNKNOWN 20°–32°), GELB 32°–52°, GRÜN 52°–80°. Grenzen in `lib/ColorSensor/ColorSensor.cpp`. Klassifikation: BLAU-Check (mit Wrap) kommt VOR satp-WHITE-Check, damit low-saturation Blau nicht als UNKNOWN landet.
- v11 (2026-04-19): `SERVO360_ALIGN_SPEED=0.77f` (einheitlich für ROT/GELB/BLAU/Initial-Ausrichtung). Zwischen Totzone (~0.75) und Überschiess-Bereich (≥0.80).
- v11 (2026-04-19): Aktive Reverse-Bremsung für 360°-Servo — Konstanten `SERVO360_BRAKE_PULSE=0.23f` / `SERVO360_BRAKE_LOOPS=2`. Bei Endschalter-Trigger: 2 Loops Gegenpuls (gespiegelt um 0.5), danach Stop-Mitte 0.5f. Kein `disable()` → Servo bleibt unter PWM-Kontrolle.
- v11 (2026-04-19): Endschalter-Debounce via `m_endstop_released` Flag — Endschalter muss erst einmal `read()==1` liefern bevor `==0` als Stopp zählt. Verhindert Sofort-Stopp wenn Servo auf Ausgangsposition steht.
- v11 (2026-04-19): ROT + GELB stoppen jetzt ebenfalls am Endschalter (nicht mehr zeitbasiert `SERVO_ROT_LOOPS`/`SERVO_GELB_LOOPS`). BLAU-Sequenz letzte Phase: 360° dreht bis Endschalter statt 10 Loops fix. D1 bei BLAU = 0.85f (war 0.2/0.6).
- **v14 (2026-04-20): 360°-Servo Brake-Delay nach Endschalter-Hit** — `SERVO360_ALIGN_SPEED=0.52f`, `SERVO360_BRAKE_LOOPS=0` (sofort stoppen). Servo dreht mit konstanter Geschwindigkeit 0.52 und stoppt beim Endschalter-Hit sofort. Gilt universell für Initial-Ausrichtung, CROSSING_STOP (4 Querbalken), SMALL_CROSSING_STOP (4 kleine Linien). Noch nicht getestet.
- **Servo-Positionierung Präzision-Strategie (2026-04-20):** Anforderung ±4° Genauigkeit nach 45° Rotation ab Endschalter-Hit. Erste Taktik: **Timing-Kalibrierung** — messe wie lange Servo für 45° braucht, nutze diese Konstante. Fallback wenn nicht ausreichend: **Zweiter Endschalter auf PC_5** (frei, war vorher selbst Endschalter-Pin). Zweiter Schalter wird 45° nach erstem platziert → Hit löst sofortigen Stop aus.
- **v27 ist aktiver Arbeits-Zweig** (2026-04-29) — Clean Rewrite mit ServoFeedback360, 20 States, trayMoveTo()-Wrapper-Pattern. v23.2 = funktionierender Referenzstand mit Endschalter-Logik. v24/v25 = Zwischenstände, nicht mehr aktiv.
- **v27 (2026-04-29): trayMoveTo() Wrapper** — Alle moveToAngle()-Calls gehen durch trayMoveTo(). Setzt m_tray_moving=true. serviceTray() ruft update() nur wenn moving, stop() wenn isAtTarget() → kein Jiggle.
- **v27 (2026-04-29): serviceTray() stop-when-at-target** — Muster aus TEST_PARALLAX_360 (P360_AT_TARGET → stop()). SF360_TIMEOUT_LOOPS=250 als Sicherheitsnetz.
- **v27 (2026-04-29): STATE_BLIND wartet auf m_sf360_ready** — verhindert Race Condition wo serviceTray() moveToAngle(0°) nach dispatchOnColor() aufruft und Ziel überschreibt.
- **v27 (2026-04-28): ServoFeedback360 disable()/enable()** — Neue Methoden in `lib/ServoFeedback360/`. Im Idle wird Bit-Bang-PWM komplett abgeschaltet (DigitalOut LOW). Grund: ISR-Kontention von PwmIn × 2 + Encoder × 2 + andere Servos macht die HIGH-Pulsbreite jittery (Timeout-IRQ wird verzögert) → Parallax 360 wackelt um die enge ~30 µs Totzone. `serviceTray()`: nach Warmup → disable(); bei isAtTarget → stop()+disable(). `trayMoveTo()`: enable(0.5f) vor moveToAngle().
- **v27 (2026-04-28): Schmallinien Assumption-Farbe durchreichen** — In SMALL_FOLLOW/SMALL_COLOUR_ASSUMPTION → SMALL_CROSSING_STOP wird `m_action_color = m_assumption_color` gesetzt statt 0. Grund: Farbsensor sitzt vorne, Sensorbalken mittig — beim Linientrigger schaut Farbsensor schon hinter der Linie, Standstill-Read erwischt zufällig die nächste Karte → falscher Slot. Fallback bleibt für Edge-Case ohne Assumption.
- **v27 (2026-04-28): Jiggle-Parameter beruhigt** — JIGGLE_AMPL_DEG 5°→3°, JIGGLE_LOOPS 75→100. D1_JIGGLE_OUT/IN ersetzt durch D1_JIGGLE_OFFSET=0.05f relativ zur Basis.
- **v27 (2026-04-28): D2 Pickup-Tiefen ~12.5 mm tiefer** — D2_DOWN_BLAU=0.28 (war 0.38), D2_DOWN_GRUEN=0.07 (war 0.22, User-Adjust noch tiefer als symmetrisch). Iterativ in 3 Schritten gesenkt (-4 mm, -5 mm, -3.5 mm).
- **v27 (2026-04-28): BLAU horizontaler Arm 13 mm kürzer** — `D1_EXTENDED_BLAU=0.85f` + `extensionForColor()` Helper. Andere Farben weiter bei 0.95.
- **v27 (2026-04-28): SMALL_CROSSING_STOP Timeout** — SF360_TIMEOUT_LOOPS=250 (5 s) konsistent zu CROSSING_STOP eingebaut.
- **v27 (2026-04-28): Pin-Schema von V23 übernommen** — Drahtzugliste-V10-Spec war zur ursprünglichen Hardware inkompatibel. Encoder M10=(PB_13,PA_6,PC_7), M11=(PA_9,PB_6,PB_7). ColorSensor S0=PA_4, S1=PB_0.
- **TEST_PARALLAX_360 (2026-04-29): Targets 90°** — {90°, 180°, 270°, 0°}, N_TARGETS=4 (war 8×45°).
- **v28 (2026-04-29): setMaxAcceleration NUR für D1** — D2 (vertikal) ohne Rampe. Grund: 0.3f/s Rampe braucht >3s für Hub 0.07→1.0, RAISE_LOOPS=30 (0.6s) reicht nicht → Arm fuhr erst beim nächsten Balken hoch.
- **v28 (2026-04-29): MAX_LINE_STEER=1.2f** — Clamp auf Radgeschwindigkeitsdifferenz in allen Linienfolge-States (LINE_FOLLOW, COLOUR_ASSUMPTION, SMALL_FOLLOW, SMALL_COLOUR_ASSUMPTION). Ghost-Reads erzeugen bis 3.2 RPS Diff → capped auf 1.2 = ~200mm Kurvenradius.
- **v28 (2026-04-29): Dreiphasen-Geschwindigkeitsprofil** — Phase A: Rampe 0→MAX_SPEED (RESTART_ACCEL_LOOPS=50), Phase B: Volle Geschwindigkeit Linienfolge (FOLLOW_FULL_LOOPS=50), Phase C: Cruise bei LINE_CRUISE_SCALE (aktuell 0.5f, zu tief für Kurven → erhöhen auf 0.8f).
- **v29 (2026-04-30): Boost-Rampenprofil** — Kopie von v28. Dreiphasenprofil ersetzt durch Zweiphasenprofil: Phase A: Gerade 0→LINE_BOOST_SCALE (0.65f) über 50 Loops. Phase B: Linienfolge bei BOOST, bei Farberkennung Rampe auf LINE_CRUISE_SCALE (0.5f) über 13 Loops. FOLLOW_FULL_LOOPS und SLOW_FACTOR entfernt. currentSlowRamp() rampiert BOOST→CRUISE statt 1.0→SLOW_FACTOR.
- **v29 (2026-05-01): STATE_BLIND Anfahrrampe + Linienfolger** — Zweiphasig: Phase A (RESTART_ACCEL_LOOPS=50): `driveStraight(BLIND_SPEED * ramp)` — beide Motoren gleich, kein Lenken. Phase B: `applyLineFollowSpeedClamped(BLIND_SPEED)` mit `KP_FOLLOW=1.1f / KP_NL_FOLLOW=0.7f` (identisch zu STATE_FOLLOW). Ziel: gleiche Endausrichtung wie STATE_FOLLOW → gerades Rückwärtsfahren.
- **v29 (2026-05-01): ROT_GELB_DRIVE_LOOPS=85** (war 55) — STATE_COLOUR_DRIVE_PAST speed-cap auf LINE_BOOST_SCALE=0.65f statt 1.0f. Distanz konstant halten: 55/1.0 ≈ 85/0.65 ≈ gleiche ~120mm. ROT_GELB_ACCEL_LOOPS=20 (war 10).
- **v29 (2026-04-30): ROT-Schwenk-Ursache identifiziert** — COLOUR_DRIVE_PAST: eingefrorener m_angle vom Balken-Standstill (wide_detection friert Winkel ein) + CURVE_BIAS=-0.26 rad in LineFollower. Fix (0.4s gerade fahren + applyLineFollowSpeedClamped) bekannt aber noch nicht aktiviert.
- **v30_1 (2026-05-01): Neuer Arbeits-Zweig basierend auf v23_2** — 16 States, Countdown-Arm-Sequenz ersetzt durch runPickupPhase()/runDeliverPhase() aus v29. advanceTray(+90°) bleibt (kein farbbasierter Winkel). SF360_OFFSET=110°.
- **v30_1 (2026-05-01): D1-Einfahren ohne Rampe** — setMaxAcceleration(1.0e6f) zu Beginn Phase 5 in runPickupPhase() und Phase 3 in runDeliverPhase(). Grund: 0.3f/s² Rampe braucht 2.94s für 0.65→0.0, RAISE_LOOPS=50 (1s) reicht nicht. setMaxAcceleration(1.0e6f) = sofort, dann 1s physische Bewegungszeit.
- **v30_1 (2026-05-01): Servo-Enable/Disable Pattern (final):** `trayMoveTo(deg)` → enable nur wenn !m_tray_moving, dann moveToAngle(). `trayStop()` → stop()+disable()+m_tray_moving=false. `serviceTray()` → kein Auto-Disable, nur update() wenn m_tray_moving. Alle Disables explizit: after m_home_timer (Ausrichtung), at tray_ok in allen Stop-States.
- **v30_1 (2026-05-01): runPickupPhase() 6-Phasen mit sofortigem Jiggle:** Jiggle via `applyJiggleTick(s_jiggle_tick)` in Phase 1-3 (static s_jiggle_tick, reset bei 0→1 Transition). Stoppt in Phase 4 (D2 hoch). D1-Jiggle synchron ab Phase 2 (D1 ausgefahren).
- **v30_1 (2026-05-01): ROT_GELB_PAUSE prev_state Fix:** `static State prev_state = STATE_BLIND` statt STATE_ROT_GELB_PAUSE — damit Entry-Code (trayMoveTo + Phase-Reset) beim ersten Eintritt wirklich läuft.
- **v30_1 (2026-05-01): Ausrichtung Drehteller:** Bei all_sensors_active() in STATE_BLIND → trayMoveTo(0.0f) + m_home_timer=150 (3s). Nach Timer: trayStop(). D1/D2 von Anfang an in Heimposition (enable in _init()).
- **v31_1 (2026-05-05): Sofortige Tray-Ausrichtung** — trayMoveTo() wird sofort bei Entry in CROSSING_STOP aufgerufen wenn `m_color_fallback` gesetzt ist (aus Anfahrt/Linienfolge). Zusätzlich: sobald `m_action_color` erstmals während Read-Phase gesetzt wird → sofort trayMoveTo(). Vorher: Tray wartete immer 25 Loops (0.5s) bevor er sich bewegt.
- **v32_02 (2026-05-05): Ablege-Sequenz überall** — `runPickupPhase()` komplett durch `runDeliverPhase()` ersetzt in CROSSING_STOP + ROT_GELB_PAUSE. SMALL_CROSSING_STOP war bereits deliver. Roboter legt jetzt an allen 8 Stops ab statt 4× aufnehmen + 4× ablegen.
- **v32_02 (2026-05-05): GRÜN+BLAU COLOR_STABLE_CNT=1** — Farberkennung für GRÜN (5) und BLAU (7) reicht nach 1 stabiler Lesung statt 3. ROT/GELB bleiben bei 3. Grund: ohne m_color_fallback aus REAL_APPROACH muss Farbe im 0.5s Stopp-Fenster sicher erkannt werden.
- **v32_02 (2026-05-05): Kein Farblesen in STATE_REAL_APPROACH** — m_color_fallback wird beim ersten Balken nicht mehr aus der Anfahrt gesetzt. Verhindert falsche Tray-Ausrichtung durch Phantom-Farbreads vor dem Stopp. Konsequenz: Tray bewegt sich erst nach Stopp wenn CROSSING_STOP Farbe liest.
- **v32_02 (2026-05-05): ROT_GELB_DRIVE_LOOPS=51** — war 55, ~0.8cm weniger Fahrdistanz (2.2mm/Loop bei ~12cm Gesamtstrecke).
- **v33_03 (2026-05-06): NeoPixel an PC_5** — Live-Farbanzeige 50Hz, dimm (r/g/b=60 statt 255). Für helle Anzeige beim Stopp: `setNeoColor()` in reading blocks. Überschrieben von Live-Loop, aber korrekt beim Halt sichtbar.
- **v33_03 (2026-05-06): Jiggle-Fix-Versuch — alle Code-Pfade behoben, Jiggle immernoch nicht sichtbar** — GRÜN/BLAU arm-Start bei CROSSING_STOP-Eintritt; GELB depthForColor-Bug (m_action_color=0 → m_rot_gelb_color restored); Phase-Reset-Guard; Timeout 250→100. Nächster Debug-Schritt: Serial Monitor ph=-Counter live beobachten.
- **v33_03 (2026-05-06): m_approach_fallback** — liest Farbe während REAL_APPROACH als letzten Fallback wenn CROSSING_STOP Lese-Fenster nichts erkennt. Wird nur in CROSSING_STOP ctr==75 als Notfall genutzt.
- **v35_04_02 (2026-05-06): Aktiver Arbeits-Zweig** — Kopie von v34_04_01 (Per-Farbe-States). Schmale Balken getestet und funktionieren perfekt. Breite Balken: 50/50 beim ersten Balken (Farb-Geometrie-Problem), GELB unklar.
- **v35_04_02 (2026-05-06 S2): REAL_APPROACH_GUARD=20** — Schutz-Loops vor wide_bar_active()-Check in STATE_REAL_APPROACH. 12 (ACCEL_LOOPS) war zu früh (Jiggle während Ausrichtung), 50 überfuhr Balken 1.
- **v35_04_02 (2026-05-06 S2): Farblesung in STATE_REAL_START_PAUSE** — `m_approach_fallback` wird auch während der Pause nach BACKWARD gesetzt. Hilft wenn Farbsensor zufällig über Aufkleber steht.
- **v35_04_02 (2026-05-06 S2): Fallback-Counter-Bug behoben** — In CROSSING_STOP + SMALL_CROSSING_STOP: `m_crossing_ctr--` jetzt VOR dem Fallback-Check. Vorher wurde Fallback im letzten Loop nie angewendet.
- **v34_04_01 (2026-05-06): Per-Farbe-States implementiert** — CROSSING_STOP/SMALL_CROSSING_STOP = Dispatch. 8 States: STATE_ARM_{ROT=0°,GRUEN=90°,BLAU=180°,GELB=270°} + SMALL_ARM_*. ROT/GELB: Phase A Fahrt (ROT_GELB_DRIVE_LOOPS=51) + Phase B Arm. Helper: resetArmExitVars(), exitWideBar(), exitSmallLine(). m_drive_ctr neu. colorToSlot/COLOR_ANGLE/m_rot_gelb_color/m_rot_gelb_is_small entfernt.
- **v34_04_01 (2026-05-06): Aktiver Arbeits-Zweig** — Kopie von v33_03. Enthält alle v33_03-Fixes. Nächster Schritt: Per-Farbe-States. V33_03 bleibt als Backup unverändert.
- **v34_04_01 (2026-05-06): Per-Farbe-States geplant** — CROSSING_STOP/SMALL_CROSSING_STOP werden reine Dispatch-States. 8 neue States: STATE_ARM_BLAU/GRUEN/ROT/GELB + STATE_SMALL_ARM_*. Jeder State hat hard-coded Tray-Winkel (BLAU=180°, GRÜN=90°, ROT=0°+Drive, GELB=270°+Drive). STATE_ROT_GELB_DRIVE + STATE_ROT_GELB_PAUSE werden entfernt. colorToSlot()/COLOR_ANGLE[] werden entfernt.
- **v34_04_01 (2026-05-06): COLOR_STOP_STABLE_CNT=1** — alle Farben brauchen nur 1 stabile Lesung im Stop (auch ROT/GELB).
- **v34_04_01 (2026-05-06): SF360_TIMEOUT_LOOPS=30** — Phase 0 (Tray-Warten) bricht nach 0.6s ab.
- **v34_04_01 (2026-05-06): m_approach_fallback bei REAL_APPROACH-Entry direkt als m_color_fallback** — `if (m_color_fallback == 0) m_color_fallback = m_approach_fallback;` vor dem Entry-Block. Löst bar-1-Problem (kein Fallback aus REAL_APPROACH in v32_02).
- **v31_1 (2026-05-05): m_color_fallback Reset bei allen Exits** — `m_color_fallback = 0` in CROSSING_STOP-Exit, SMALL_CROSSING_STOP-Exit und ROT_GELB_PAUSE-Exit. Verhindert dass alte Farbe beim nächsten Balken den Tray zum falschen Winkel schickt.
- **v30_1 (2026-05-02): trayStop() nur für GRÜN/BLAU vor Weiterfahrt** — ROT/GELB-Exit aus CROSSING_STOP und SMALL_CROSSING_STOP ruft trayStop() NICHT auf. Servo hält Farb-Winkel aktiv während ROT_GELB_DRIVE. trayStop() erst am Ende von ROT_GELB_PAUSE. Grund: ohne Torque dreht Tray während Fahrt weg → Phase 0 muss erst korrigieren → Jiggle verzögert oder fehlt.
- **v30_1 (2026-05-02): Phase 0 Stale-Error-Fix** — `fresh_at_target = (m_phase_ctr >= 2) && isAtTarget()` in runPickupPhase() und runDeliverPhase(). Verhindert sofortigen Phase-0-Exit mit eingefrorenem m_error vom letzten disable() (update() noch nicht gelaufen).
- **v30_1 (2026-05-02): ph=X im Print-Output** — printf zeigt jetzt `ph=m_phase_idx` für Serial Monitor Diagnose welche Arm-Phase gerade aktiv ist.
- **Servo setMaxAcceleration Kalibrierungs-Effekt (2026-05-01):** setMaxAcceleration(X) multipliziert X mit (pulse_max - pulse_min). Für D1 (0.050–0.105): effektive Beschleunigung = X * 0.055. Bei X=0.3 → 0.0165 raw/s². Distanz 0.65→0.0 = 0.03575 raw → 2.94s theoretisch. Für D2 (0.020–0.131): effektive Beschleunigung bei X=0.3 → 0.3 * 0.111 = 0.0333 raw/s².
- **TEST_ARM_SEQUENCE (2026-04-30): Arm-Isolationstest** — `src/test_files/test_arm_sequence.cpp`, Single-File ohne .h. 10 Schritte: Homing→Tray+30°→M20 ausfahren→M21 partial→Tray+330° CW→M21 voll→Jiggle ±10°→Tray 45°→Jiggle ±5°→Arm hoch. Button-Logik: Knopf1=Start, Knopf2=Stop+Reset auf S1, Knopf3=Start erneut. Kein setMaxAcceleration auf M20 (V27-Werte). Tray +330° als 2×165° Substeps (verhindert 180°-Ambiguität bei shortest-path-Berechnung).
- **Button-Pattern (2026-04-30):** Test-Module dürfen `BUTTON1` nicht direkt lesen — `main.cpp`-DebounceIn-ISR interceptiert. `_task()` läuft nur wenn `do_execute_main_task=true` (ungerade Knöpfe). `_reset()` wird bei geraden Knöpfen aufgerufen. `while(1)` in `_task()` blockiert main-Loop → stattdessen SEQ_DONE-State verwenden.
- **v25 ist aktiver Arbeits-Zweig** (2026-04-28) — leeres Skeleton, bereit zum Befüllen. v24 = Kopie v23 als prototype03-Prefix (Backup). v23 = letzter getesteter Stand, unverändert.
- **v23 war aktive Hauptversion** (2026-04-23 Session 2) — Kopie von v22. v22 bleibt als Backup unverändert, v21 ebenfalls
- **v23 (2026-04-23 S2): Wait-for-Release-Entprellung für 360°-Endschalter-Click** — neue Statics `m_servo360_wait_release` + `m_servo360_release_ctr`. Nach jedem akzeptierten Click muss Pin 3 Loops lang `read()==1` liefern, sonst werden Prell-Impulse als Geister-Clicks gezählt. Brems-/Stopp-Logik unverändert
- **v23 (2026-04-23 S2): Intro-Sequenz PID-Verbesserungen** — (a) harte `setRotationalVelocityControllerGains(KP, KP_NL)` beim Übergang FOLLOW → FOLLOW_1S (vorher blieben weiche KP_FOLLOW aktiv), (b) Bremsrampe mit Durchschnitt `(M1+M2)/2` statt individuellen Motor-Commands → symmetrisches Bremsen, (c) Fahrgeschwindigkeit in FOLLOW_1S halbiert (`* 0.5f`) für mehr PID-Ausrichtungszeit
- **v23 (2026-04-23 S2): STATE_BLIND Initial-Ausrichtung auf 5 Clicks** (vorher 3) — Servo überfährt Endschalter 4×, stoppt beim 5. Hit
- **v23 (2026-04-23 S2): Zwei Guard-Konstanten statt einer** — `START_GUARD=300` (6 s, nur TURN_SPOT→FOLLOW), `STOP_GUARD=75` (1.5 s, zwischen Balken in REAL/SMALL_FOLLOW). Vorher teilten sich beide Übergänge den gleichen `STOP_GUARD`
- **v23 (2026-04-23 S2): `all_sensors_active()` auf 7/8 Sensoren gelockert** — toleranter gegen schrägen Einlauf, gilt in STATE_BLIND und STATE_FOLLOW
- **v23 (2026-04-23 S2): Jiggle 2 in ROT_GELB_PAUSE auf 12 Loops** (vorher 7) — Counter 52 → 40 statt 52 → 45, D2 geht synchron mit Stopp hoch. Gilt ROT + GELB an allen Positionen
- **v23 (2026-04-23 S2): Semantik der Zählweise** — "5 Clicks" = 5 Endschalter-Hits (nach Entprellung). An breiten Balken dreht das Tablett 5 Clicks weiter (inkl. letztem), an Schmallinien 1×90°-Kick
- **v35_04_02 (2026-05-06): Aktiver Arbeits-Zweig + erster funktionierender Jiggle bei BLAU** — Kopie von V34_04_01 mit Timing-Fixes für breite Balken.
- **v35_04_02 (2026-05-06): WIDE crossing stop strikt wie SMALL** — `m_approach_fallback` aus REAL_APPROACH-Pfad und CROSSING_STOP-Body entfernt. `m_color_fallback` direkt in REAL_APPROACH gesetzt (gleiche Stelle wie REAL_FOLLOW). Behebt: bar 1 hatte zwei verschiedene Fallback-Variablen die durcheinander gerieten.
- **v35_04_02 (2026-05-06): `runDeliverPhase()` Phase 0 robuster** — kein quick-timeout mehr, wartet auf `isAtTarget()`. Periodischer Kick alle 25 loops (forciert `g_servoTray->enable(0.5f) + moveToAngle(m_target_angle)` direkt, umgeht `m_tray_moving`-Check von `trayMoveTo()`). Safety-Notfall nach 500 loops (10s). Wichtige Erkenntnis: der 360°-Servo ignoriert manchmal den ersten Befehl → periodischer Re-Trigger nötig.
- **v35_04_02 (2026-05-06): Konstanten erhöht für wide bars** — `CROSSING_STOP_LOOPS=150` (war 100, mehr Reading-Zeit), `SF360_TIMEOUT_LOOPS=60` (war 30, betrifft auch STATE_ARM_*-Exit-Wait).

## Projekt-Kontext
- **Kurs:** ZHAW 2. Semester, Modul PM2
- **Abgabe:** 23.05.2026 (Abend)
- **Team:** 6 Personen — 3x Elektronik & Programmierung, 3x Mechanik (CAD)

## Nächste Schritte
<<<<<<< HEAD
1. **Erster breiter Balken — Farblesung debuggen:** Serial Monitor während Test laufen lassen. Am ersten breiten Balken `act=X` und `col=X` beobachten. Wenn `act=0`: Farbe wird nicht erkannt → prüfen ob `m_approach_fallback` aus STATE_REAL_START_PAUSE gesetzt wurde (Serial-Print `fb=m_approach_fallback` temporär hinzufügen). Falls Farbe nie gelesen wird: Farblesung auch in `STATE_BLIND` nach `all_sensors_active()` und in `STATE_BACKWARD` starten (Farbsensor überfährt Balken 1 während Rückwärtsfahrt).
2. **GELB breite Balken isolieren:** Nur GELB-Karte auf Matte legen, `pio run --target upload`, prüfen ob `act=4` (GELB) gesetzt wird und `ph=0→1→2→3` im Serial erscheint. Falls `act=4` OK aber kein Jiggle: `STATE_ARM_GELB` Phase-Sequenz in Code prüfen.

## Offene Fragen
- **Erster breiter Balken — kein Jiggle nach Ausrichtung (50/50):** Geometrisches Problem — Farbsensor überfährt Aufkleber schon während BACKWARD/Ausrichtung. `m_approach_fallback=0` und CROSSING_STOP-Fenster liest Matte → `act=0` → Default → kein ARM-State. Fix-Kandidat: Farblesung in STATE_BACKWARD oder STATE_BLIND nach all_sensors_active() starten.
- **GELB breite Balken: Probleme** — unklar ob Farblesung (act=4 nicht gesetzt) oder Jiggle-Phase (ph hängt). Mit Serial Monitor isolieren.
- **ROT_GELB_DRIVE_LOOPS=51 nicht final validiert** — 0.8cm weniger als vorher (war 55). Evtl. +/−1 Loop ≈ 2.2mm nötig.
- **D2 Servo Pin unklar** — Code hat `servo_D2(PB_2)`, CLAUDE.md Pin-Layout sagt PC_6 (= PB_D2). Vor Inbetriebnahme prüfen.
=======
1. **GELB beim 3. breiten Balken zum Jiggeln bringen:** In `src/prototype03_v35_04_02.cpp` `runDeliverPhase()` Phase 0 die Kick-Frequenz erhöhen (von 25 auf 15 loops) UND/ODER den ersten Kick sofort statt erst nach Loop 25 absetzen. Dann `pio run --target upload` und Serial Monitor — beim 3. breiten Balken (GELB, 270°) prüfen ob `tray=` auf 270° läuft bevor `ph=` von 0 auf 1 wechselt. Falls immer noch Notfall-Timeout: `SF360_TIMEOUT_LOOPS` (Phase 0 Safety) auf 800 erhöhen oder Servo-Hardware checken.

## Offene Fragen
- **GELB beim 3. breiten Balken kein Jiggle** — Tray erreicht 270° nicht innerhalb 10s Notfall-Timeout. Kick-Mechanismus alle 25 loops reicht nicht für die längste Drehung (270° vs 90°/180° die funktionieren).
- **ROT_GELB_DRIVE_LOOPS=51 noch nicht validiert** — 0.8cm Fahrdistanz nach ROT/GELB-Erkennung. Bei ROT funktionierte es heute, GELB konnte nicht getestet werden.
- **D2 Servo Pin unklar** — Code hat `servo_D2(PB_2)`, Pin-Layout sagt PC_6. Vor Inbetriebnahme prüfen ob physisch auf PB_2 oder PC_6 verdrahtet.
>>>>>>> a7472cd667993ad4c9bef5386ae190df239b7cfe
- **Arm-Tiefen nicht final validiert:** D2_DOWN_GRUEN=0.07f, D2_DOWN_BLAU=0.24f. +0.01f ≈ 1.3mm höher falls zu tief.

## Session-Routine
Am Ende jeder Session:
1. Neue Architektur-Entscheidungen → Abschnitt "Aktive Entscheidungen" ergänzen
2. Neue Erkenntnisse / Bugs → Auto Memory speichern
3. Offene Punkte → Abschnitt "Offene Fragen" aktualisieren
