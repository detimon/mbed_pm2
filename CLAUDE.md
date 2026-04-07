# mbed_pm — PES Board Roboter (Nucleo F446RE)

## Aktueller Stand
_Wird am Ende jeder Session via `/sesh-end` aktualisiert._ (2026-04-07)
- **Aktiv:** `TEST_ROBOTER_V2` in `test_config.h` (`roboter_v1` bleibt unberührt als Backup)
- **roboter_v2 State Machine:** BLIND → STRAIGHT (1.7s) → TURN_SPOT (Pirouette) → FOLLOW (pure FAST) mit 4× 5s-Stop auf Querlinien → FOLLOW forever
- **Stop-Logik:** `m_stop_remaining=4` nach TURN_SPOT; erster Stop (schräger Eingang) triggert auf `isLedActive()`, die 3 weiteren auf `all_sensors_active()`; nach 4. Stop: kein weiterer Stop
- **Guard:** 1.5s (75 Loops) nach jedem Stop — verhindert Sofort-Retrigger auf gleicher Querlinie
- **Pivot-Drehung:** M1 vorwärts, M2 rückwärts (VEL_SIGN*+0.5f / VEL_SIGN*-0.5f) → dreht rechts auf der Stelle; endet wenn b3 oder b4 aktiv
- **Stabile Konstanten übernommen aus v1:** KP=2.8, KP_NL=4.55, MAX_SPEED=1.0, BLIND_SPEED=1.0, STRAIGHT_SPEED=1.0, TURN_SPEED=0.5
- **roboter_v1** bleibt in `src/roboter_v1.cpp` vollständig erhalten (auskommentiert in test_config.h)

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
- `TEST_ROBOTER_V2` ist aktuell aktiv in `test_config.h` (v1 auskommentiert, als Backup erhalten)
- roboter_v2 Stop-Trigger: erster Crossing = `isLedActive()` (schräger Eingang), weitere 3 = `all_sensors_active()`; Guard 75 Loops nach jedem Stop
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
1. `roboter_v2` auf der Strecke testen: Stop-Trigger für ersten Crossing (`isLedActive`) und die 3 normalen (`all_sensors_active`) verifizieren — falls Guard zu kurz (Sofort-Retrigger), `STOP_GUARD` in `roboter_v2.cpp:32` erhöhen; falls Pivot-Richtung falsch, Vorzeichen in `STATE_TURN_SPOT` tauschen
2. `TEST_IR` aktivieren → IR-Sensor (PB_1/A3) Rohwerte in mV bei verschiedenen Abständen notieren → Kalibrierung bestimmen
3. Hauptprogramm: IR-Sensor + Servo in roboter_v2 State Machine integrieren

## Offene Fragen
- Stimmt die Pivot-Drehrichtung in STATE_TURN_SPOT? (M1 vor / M2 zurück = dreht rechts — je nach Strecke evtl. links drehen nötig)
- Reicht `isLedActive()` als Trigger für den ersten (schrägen) Crossing, oder braucht es eine spezifischere Bedingung?
- Wie sieht die finale Integration von IR-Sensor + Servo in roboter_v2 aus?

## Session-Routine
Am Ende jeder Session:
1. Neue Architektur-Entscheidungen → Abschnitt "Aktive Entscheidungen" ergänzen
2. Neue Erkenntnisse / Bugs → Auto Memory speichern
3. Offene Punkte → Abschnitt "Offene Fragen" aktualisieren
