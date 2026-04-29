# mbed_pm — PES Board Roboter (Nucleo F446RE)

## Aktueller Stand
_Wird am Ende jeder Session via `/sesh-end` aktualisiert._ (2026-04-29)
- **`PROTOTYPE_03_V27` aktiv** — Clean Rewrite mit ServoFeedback360 (Parallax 360°), 20 States, vollständige PICKUP + DELIVER Logik. test_config.h auf `PROTOTYPE_03_V27`.
- **STATE_BLIND funktioniert (getestet)** — Fix: war timer-basiert (0.5 s), jetzt `sevenOfEightActive() && m_sf360_ready`. Roboter fährt zuverlässig bis zum Balken.
- **ColorSensor Pins fix** — S0 war PA_1 (falsch), S1 war PA_4 (falsch) → korrigiert auf S0=PA_4, S1=PB_0 (wie alle anderen Versionen).
- **serviceTray() + trayMoveTo() Refactor** — Muster von TEST_PARALLAX_360 übernommen: `stop()` im Ruhezustand (kein Jiggle), `update()` nur wenn `m_tray_moving=true`, `stop()` sobald `isAtTarget()`. Alle `moveToAngle()`-Calls gehen jetzt durch `trayMoveTo()`.
- **STATE_CROSSING_STOP Timeout** — SF360_TIMEOUT_LOOPS=250 (5 s) wie TEST_PARALLAX_360, verhindert unbegrenztes Hängen.
- **TEST_PARALLAX_360** — Targets auf 4×90° geändert (war 8×45°): `{90°, 180°, 270°, 0°}`.
- **Arm-Extension beim ersten Balken noch nicht abschliessend getestet** — serviceTray/trayMoveTo-Fixes wurden gerade implementiert, noch nicht geflasht.

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
- **TEST_PARALLAX_360 (2026-04-29): Targets 90°** — {90°, 180°, 270°, 0°}, N_TARGETS=4 (war 8×45°).
- **v25 ist aktiver Arbeits-Zweig** (2026-04-28) — leeres Skeleton, bereit zum Befüllen. v24 = Kopie v23 als prototype03-Prefix (Backup). v23 = letzter getesteter Stand, unverändert.
- **v23 war aktive Hauptversion** (2026-04-23 Session 2) — Kopie von v22. v22 bleibt als Backup unverändert, v21 ebenfalls
- **v23 (2026-04-23 S2): Wait-for-Release-Entprellung für 360°-Endschalter-Click** — neue Statics `m_servo360_wait_release` + `m_servo360_release_ctr`. Nach jedem akzeptierten Click muss Pin 3 Loops lang `read()==1` liefern, sonst werden Prell-Impulse als Geister-Clicks gezählt. Brems-/Stopp-Logik unverändert
- **v23 (2026-04-23 S2): Intro-Sequenz PID-Verbesserungen** — (a) harte `setRotationalVelocityControllerGains(KP, KP_NL)` beim Übergang FOLLOW → FOLLOW_1S (vorher blieben weiche KP_FOLLOW aktiv), (b) Bremsrampe mit Durchschnitt `(M1+M2)/2` statt individuellen Motor-Commands → symmetrisches Bremsen, (c) Fahrgeschwindigkeit in FOLLOW_1S halbiert (`* 0.5f`) für mehr PID-Ausrichtungszeit
- **v23 (2026-04-23 S2): STATE_BLIND Initial-Ausrichtung auf 5 Clicks** (vorher 3) — Servo überfährt Endschalter 4×, stoppt beim 5. Hit
- **v23 (2026-04-23 S2): Zwei Guard-Konstanten statt einer** — `START_GUARD=300` (6 s, nur TURN_SPOT→FOLLOW), `STOP_GUARD=75` (1.5 s, zwischen Balken in REAL/SMALL_FOLLOW). Vorher teilten sich beide Übergänge den gleichen `STOP_GUARD`
- **v23 (2026-04-23 S2): `all_sensors_active()` auf 7/8 Sensoren gelockert** — toleranter gegen schrägen Einlauf, gilt in STATE_BLIND und STATE_FOLLOW
- **v23 (2026-04-23 S2): Jiggle 2 in ROT_GELB_PAUSE auf 12 Loops** (vorher 7) — Counter 52 → 40 statt 52 → 45, D2 geht synchron mit Stopp hoch. Gilt ROT + GELB an allen Positionen
- **v23 (2026-04-23 S2): Semantik der Zählweise** — "5 Clicks" = 5 Endschalter-Hits (nach Entprellung). An breiten Balken dreht das Tablett 5 Clicks weiter (inkl. letztem), an Schmallinien 1×90°-Kick

## Projekt-Kontext
- **Kurs:** ZHAW 2. Semester, Modul PM2
- **Abgabe:** 23.05.2026 (Abend)
- **Team:** 6 Personen — 3x Elektronik & Programmierung, 3x Mechanik (CAD)

## Nächste Schritte
1. **v27 flashen und ersten breiten Balken testen:** Roboter auf Track stellen, Button drücken — prüfen ob (a) Drehteller zum Farb-Slot dreht und stoppt ohne Jiggle, (b) D1-Arm auf 0.95f ausfährt, (c) D2-Arm auf Pickup-Tiefe absenkt, (d) Drehteller-Jiggle ±5° läuft, (e) Arm einzieht und Roboter weiterfährt. Wenn Arm nicht ausfährt: `m_tray_moving`-Flag und `STATE_CROSSING_STOP`-Exit-Bedingung prüfen.
2. **Pickup-Tiefe kalibrieren (falls nötig):** `D2_DOWN_GRUEN=0.22f`, `D2_DOWN_BLAU=0.38f` aus v23.2 übernommen — physisch prüfen ob Päckchen korrekt gefasst wird. Faustregel +0.01 Puls ≈ 1.3 mm tiefer.
3. **DELIVER-Phase testen** (nach erfolgreicher PICKUP): Schmallinie-Erkennung und Ablage-Sequenz.

## Offene Fragen
- **Arm-Extension v27:** Wird D1 (0.95f) beim ersten breiten Balken ausgefahren? Hängt davon ab ob `m_tray_moving` beim CROSSING_STOP-Exit korrekt gesetzt ist und `isAtTarget()` nicht durch initial m_error=0.0f falsch-true liefert (durch `m_sf360_ready`-Bedingung in STATE_BLIND sollte das jetzt verhindert sein).
- **Drehteller-Jiggle-Abdeckung:** JIGGLE_AMPL_DEG=5°, D1_JIGGLE_OUT=1.00f / D1_JIGGLE_IN=0.90f — reicht das für 1.5 cm² Flächenabdeckung? Erst nach erfolgreichem Pickup-Test relevant.
- **DELIVER-Phase ungetestet:** Schmallinie-Detection (STATE_SMALL_FOLLOW), COLOUR_STOP, DELIVER-Sequenz — alles noch nicht hardware-validiert.
- **Angle Clamp 0.15 rad/Loop** (LineFollower) — noch nicht validiert.

## Session-Routine
Am Ende jeder Session:
1. Neue Architektur-Entscheidungen → Abschnitt "Aktive Entscheidungen" ergänzen
2. Neue Erkenntnisse / Bugs → Auto Memory speichern
3. Offene Punkte → Abschnitt "Offene Fragen" aktualisieren
