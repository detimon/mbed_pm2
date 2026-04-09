# mbed_pm — PES Board Roboter (Nucleo F446RE)

## Aktueller Stand
_Wird am Ende jeder Session via `/sesh-end` aktualisiert._ (2026-04-09)
- **Aktiv:** `TEST_ROBOTER_V4` in `test_config.h` (v1/v2/v3 als Backup erhalten)
- **roboter_v4 State Machine (gebaut, noch nicht auf Hardware getestet):**
  - Intro (identisch v3): BLIND → STRAIGHT(1.7s) → TURN_SPOT → FOLLOW → FOLLOW_1S(1s) → BRAKE(0.24s) → PAUSE(0.4s) → BACKWARD(3.9s)
  - Breite Balken: REAL_FOLLOW → 4× CROSSING_STOP(5s Platzhalter) → nach 4. Balken weiter zu kleinen Linien
  - Kleine Linien: SMALL_FOLLOW → 4× SMALL_CROSSING_STOP(3s Platzhalter, sensors 2–5 aktiv) → FINAL_HALT
- **Kleine Linien Erkennung:** `small_line_active()` = Sensoren 2,3,4,5 alle ≥ 0.5 (`getAvgBit`)
- **CROSSING_STOP_LOOPS=250 (5s), SMALL_CROSSING_STOP_LOOPS=150 (3s)** — beide Platzhalter für Arm/Servo
- **Farbsensor dauerhaft aktiv** in roboter_v4: erkennt WHITE (Basis), ROT/GELB/GRÜN/BLAU; steigende Flanke loggt in `m_color_log[8]`
- **LED Blinkcode** auf led1 (PB_10): schnell=kein Signal, 1/2/3/4 Blinks/2s = ROT/GELB/GRÜN/BLAU; Blinkmuster bleibt nach Erkennung aktiv
- **Pins Farbsensor:** PB_3(freq), PC_3(led), PA_4(S0), PB_0(S1), PC_1(S2), PC_0(S3)

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
- PB_9 als DigitalOut verwenden, wenn LINE_FOLLOWER aktiv ist — PB_9 ist I2C-SDA der SensorBar → `led1` muss dann auf PB_10
- `TIM1->BDTR |= TIM_BDTR_MOE` entfernen — MOE muss manuell gesetzt werden (STM32-Hardware-Requirement)
- `PERFORM_GPA_MEAS` und `PERFORM_CHIRP_MEAS` gleichzeitig aktivieren (DCMotor.h: "only use GPA or Chirp, not both")
- Mehr als ein `#define` in `test_config.h` gleichzeitig aktiv lassen

## Pin-Layout (vollständig)

| Pin | Mbed Name | Komponente |
|-----|-----------|------------|
| D1  | PC_8      | 180° Servo A |
| D2  | PC_6      | 180° Servo B |
| D3  | PB_12     | 360° Servo |
| A3  | PB_1      | IR-Sensor |
| D14/D15 | PB_9 / PB_8 | Line-Follower Array (I2C SDA/SCL) |
| —   | PB_3      | Farbsensor – Freq Out |
| —   | PC_3      | Farbsensor – LED Enable |
| —   | PA_4      | Farbsensor – S0 |
| —   | PB_0      | Farbsensor – S1 |
| —   | PC_1      | Farbsensor – S2 |
| —   | PC_0      | Farbsensor – S3 |

## Aktive Entscheidungen
- Claude Code Modell: `opus[1m]` in `~/.claude/settings.json` — bei Opus-Rate-Limit temporär auf `sonnet` wechseln (Opus-Sublimit ist unabhängig vom Gesamt-Limit)
- `/popupssound` toggled Sound via Flag-Datei `C:\Users\alexa\.claude\sound_disabled` — alle drei Popup-Scripts prüfen dieses Flag
- `/popups`-Befehl toggled Popups via Flag-Datei `C:\Users\alexa\.claude\popups_disabled` — kein settings.json-Edit nötig
- Oranges Popup via `PermissionRequest`-Hook → `permission_request.ps1` (erstellt Flag + fokussiert VSCode + startet Popup)
- Popup-Dismiss via `PostToolUse`-Hook → `dismiss_orange.ps1` (löscht Flag → Popup schliesst in <100ms)
- `PreToolUse` feuert VOR der Bestätigung — für Dismiss muss `PostToolUse` verwendet werden
- Grünes Popup: kein Timeout, schliesst nur bei Maus/Tastatur; Stimme 3x (sofort, +20s, +40s)
- Alle Popups: `WaitUntilDone(-1)` nach `ShowDialog()` — Stimme spricht zu Ende auch nach frühem Dismiss
- VSCode-Fokus via `AttachThreadInput` in `focus_vscode.ps1` — funktioniert aus nicht-fokussiertem Prozess
- `TEST_ROBOTER_V4` ist aktuell aktiv in `test_config.h` (v1/v2/v3 auskommentiert, als Backup erhalten)
- roboter_v4 breite Balken-Trigger: `all_sensors_active()` (alle 8); Guard nach BACKWARD→REAL_FOLLOW = 0 (nicht 75!)
- roboter_v4 kleine Linien-Trigger: `small_line_active()` = Sensoren 2,3,4,5 alle aktiv
- roboter_v4 Farbsensor: steigende Flanke (neutral→Farbe), log bis 8 Einträge, LED-Blinkcode 1–4 Blinks/2s
- Servo-Kalibrierung abgeschlossen (2026-03-26): A=(0.0303–0.1204), B=(0.0314–0.1232), 360°=(0.0303–0.1223, Stop=0.0763)
- `test_servo_calib`: Non-blocking stdin via `mbed::mbed_file_handle(STDIN_FILENO)->set_blocking(false)` + `getchar()` == EOF als No-Input-Guard
- `test_servo_all`: 360° Phasen alle 5s wechseln: CW=0.35f, Stop=0.50f, CCW=0.65f (Standardwerte vor Kalibrierung)
- `TEST_LINE_FOLLOWER_FAST` ist stabil und funktioniert (Commit 183603b) — nicht anfassen
- Gear Ratio: 100:1 verbaut (`USE_GEAR_RATIO_78 = false`), KN = 140/12 ≈ 11.67 rpm/V
- `VEL_SIGN = -1.0f` für beide Motoren (physikalische Einbaurichtung beider Motoren invertiert)
- SensorBar I2C1: SDA = PB_9, SCL = PB_8
- Physikalische Roboter-Parameter: D_WHEEL=0.0393m, B_WHEEL=0.179m, BAR_DIST=0.1836m
- Regler FAST: KP=4.0, KP_NL=15.0, MAX_SPEED=3.5 RPS
- Regler SLOW: KP=3.0, KP_NL=10.0, MAX_SPEED=0.5 RPS
- Eigen-Lib: `EIGEN_NO_DEBUG` + `EIGEN_DONT_VECTORIZE` (Overhead-Reduktion auf MCU)

## Projekt-Kontext
- **Kurs:** ZHAW 2. Semester, Modul PM2
- **Abgabe:** 23.05.2026 (Abend)
- **Team:** 6 Personen — 3x Elektronik & Programmierung, 3x Mechanik (CAD)

## Nächste Schritte
1. `TEST_ROBOTER_V4` auf Hardware flashen und testen: breite Balken (4× CROSSING_STOP), dann kleine Linien (4× SMALL_CROSSING_STOP) — prüfen ob `small_line_active()` (Sensoren 2–5) die schmalen Linien zuverlässig erkennt oder Threshold/Sensoren angepasst werden müssen
2. Farbsensor-Log im seriellen Monitor beobachten: prüfen ob Farberkennung korrekt (WHITE auf Boden, richtige Farbe bei Farbmarkierungen)
3. Arm/Servo-Sequenz in `STATE_CROSSING_STOP` und `STATE_SMALL_CROSSING_STOP` integrieren (Servo PC_8/PC_6/PB_12), `CROSSING_STOP_LOOPS` + `SMALL_CROSSING_STOP_LOOPS` durch echte Dauer ersetzen

## Offene Fragen
- Erkennt `small_line_active()` (Sensoren 2–5) die schmalen Linien zuverlässig? Muss evtl. OR statt AND verwendet werden oder andere Sensor-Indizes?
- Wie sieht die Arm/Servo-Aktion bei den Crossings konkret aus? (Ausfahren, Drehen, Einfahren — Zeiten/Winkel noch unbekannt)
- Was passiert bei FINAL_HALT nach den 4 kleinen Linien — braucht es noch eine finale Aktion?
- IR-Sensor noch nicht kalibriert — wird für spätere Integration benötigt
- Farbsensor-Kalibrierung: sind die Hard-coded Referenzwerte (`setCalibration()`) für den aktuellen Aufbau/Abstand passend?

## Session-Routine
Am Ende jeder Session:
1. Neue Architektur-Entscheidungen → Abschnitt "Aktive Entscheidungen" ergänzen
2. Neue Erkenntnisse / Bugs → Auto Memory speichern
3. Offene Punkte → Abschnitt "Offene Fragen" aktualisieren
