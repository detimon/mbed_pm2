# mbed_pm — PES Board Roboter (Nucleo F446RE)

## Aktueller Stand
_Wird am Ende jeder Session via `/sesh-end` aktualisiert._ (2026-03-26)
- **Aktiv:** `TEST_ROBOTER_V1` in `test_config.h`
- **Funktioniert:** BLIND → TURNING (60 Loops, TURN_SPEED=1.0/TURN_INNER=0.3) → REACQUIRE (0.4 RPS, wartet auf b3+b4) → FOLLOW (LineFollower, FOLLOW_SPEED=0.8, kein isLedActive-Check) — Roboter fährt die ganze Strecke ohne Linie zu verlieren
- **Problem offen:** STATE_STRAIGHT funktioniert nicht — `getAngleRadians()` taugt nicht zur Geraden-Erkennung: Winkel ist auch auf der Geraden zu gross (> 0.10–0.20 rad) oder schwankt zu stark. Alle Ansätze (Winkel-basiert, Counter-basiert, FOLLOW_MIN_LOOPS) schlugen fehl.
- **Aktueller Code:** Hat STATE_STRAIGHT mit FOLLOW_MIN_LOOPS=75 + STRAIGHT_ENTER_ANGLE=0.20f + STRAIGHT_ENTER_LOOPS=8 — noch nicht getestet ob diese Version die Gerade erkennt
- **Servo-Kalibrierung:** Abgeschlossen (2026-03-26) — Werte in `test_servo_all.cpp` eingetragen: A=(0.0303–0.1204), B=(0.0314–0.1232), 360°=(0.0303–0.1223, Stop=0.0763)

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
- `TEST_ROBOTER_V1` ist aktuell aktiv in `test_config.h`
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
1. `roboter_v1` — Gerade-Erkennung reparieren: Serial-Output (`a=` Winkel-Wert) auf der Geraden beobachten und prüfen ob `getAngleRadians()` dort tatsächlich unter 0.20 rad bleibt; falls nicht, alternativen Ansatz via Sensor-Zählung (`getAvgBit(0)` und `getAvgBit(7)` beide inaktiv = Gerade) probieren statt Winkel
2. `TEST_IR` aktivieren → IR-Sensor (PB_1/A3) Rohwerte in mV bei verschiedenen Abständen notieren → Kalibrierung bestimmen
3. Hauptprogramm: IR-Sensor + Servo in State Machine integrieren

## Offene Fragen
- Wie sieht die finale State Machine des Hauptprogramms aus?
- Ist `getAngleRadians()` auf der Geraden tatsächlich nahe 0? → Serial-Monitor Wert ablesen und entscheiden ob Winkel- oder Sensor-basierte Geraden-Erkennung besser ist
- Reicht `getAvgBit(0) < 0.5 && getAvgBit(7) < 0.5` als Geraden-Indikator?

## Session-Routine
Am Ende jeder Session:
1. Neue Architektur-Entscheidungen → Abschnitt "Aktive Entscheidungen" ergänzen
2. Neue Erkenntnisse / Bugs → Auto Memory speichern
3. Offene Punkte → Abschnitt "Offene Fragen" aktualisieren
