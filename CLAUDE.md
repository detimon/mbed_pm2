# mbed_pm — PES Board Roboter (Nucleo F446RE)

## Aktueller Stand
_Wird am Ende jeder Session via `/sesh-end` aktualisiert._ (2026-04-14)
- **Aktiv in test_config.h:** `PROTOTYPE_02_V10` — läuft, Linienfolgen funktioniert korrekt
- **roboter_v10** = aktive Version (Kopie von v9 mit gelösten Motor/Sensor-Problemen); **roboter_v9** = Backup
- **Neues Chassis (heute):** Motoren physisch getauscht — rechter Motor jetzt an M1-Port, linker Motor an M2-Port → `VEL_SIGN=1.0f`, kein Pointer-Swap
- **Linienfolgen-Fix (heute, mehrstufig):**
  - Root cause 1: Linienfolgezuweisung invertiert → positives Feedback → stabiles Tracking bei b7 statt Zentrum
  - Root cause 2: KP=2.8/KP_NL=4.55 verursachten Rückwärtslauf eines Rades bei grossem Eintrittswinkel aus TURN_SPOT → 180°-Spin
  - Fix 1: TURN_SPOT korrigiert auf CCW (`g_M1=−TURN`, `g_M2=+TURN`)
  - Fix 2: Bei TURN_SPOT→STATE_FOLLOW Gains auf `KP_FOLLOW=0.8f` / `KP_NL_FOLLOW=0.5f` reduziert (verhindert Rückwärtslauf)
  - Fix 3: Alle 4 Linienfolgezustände: `g_cmd_M1 = getLeftWheelVelocity()`, `g_cmd_M2 = getRightWheelVelocity()` (gekreuzt, weil Sensorbalken umgekehrt montiert)
  - Fix 4: STATE_REAL_FOLLOW und STATE_SMALL_FOLLOW stellen volle Gains (2.8/4.55) beim Eintritt wieder her
- **Motor-Zuordnung v10:** M1 = physisch rechts (PB_13/PA_6/PC_7), M2 = physisch links (PA_9/PB_6/PB_7)
- **Roboter-Parameter v10:** D_WHEEL=0.0291m, B_WHEEL=0.1493m, BAR_DIST=0.182m, MAX_SPEED=1.0 RPS, SENSOR_THRESHOLD=0.40f
- **Servo-Logik (v10, identisch v8):**
  - ROT (3): `g_servo->enable(0.10f)` — 360° schnell, 3s (`SERVO_ROT_LOOPS=150`)
  - GELB (4): `g_servo->enable(0.35f)` — 360° langsam, 1s (`SERVO_GELB_LOOPS=50`)
  - GRÜN (5): `g_servo_D1->enable(1.0f)` — 180° Servo A (PC_8), nach 1s zurück
  - BLAU (7): `g_servo_D2->enable(1.0f)` — 180° Servo B (PC_6), nach 1s zurück
  - Default: `g_servo->enable(0.25f)` — 360° Fallback, 1s
- **Timing:** `CROSSING_STOP_LOOPS=200` (4s), `SMALL_CROSSING_STOP_LOOPS=200` (4s), `SMALL_REENTRY_GUARD=100` (2s)
- **Farberkennung:** Mechanik wurde geändert — Farbsensor steht nun während `CROSSING_STOP` über der Karte (statt während Fahrt). Code liest `g_cs->getColor()` weiterhin bei `wide_bar_active()` / `small_line_active()` vor `setVelocity(0)` — ob Farberkennung im Stand zuverlässig funktioniert wurde noch NICHT getestet
- **Hue-Grenzen (lib/ColorSensor/ColorSensor.cpp):** ROT 0°–20°/330°–360°, GELB 20°–80°, GRÜN 80°–215°, BLAU 215°–280°
- **Kalibrierung:** Original (weisses Papier) aktiv — Matte-Kalibrierung verschiebt Hue-Werte → nicht machen

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
| A2  | PC_5      | Endschalter 360°-Servo (Drehteller) |
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
- `PROTOTYPE_02_V10` ist aktuell aktiv in `test_config.h`
- `led1` (PB_9/PB_10) aus `main.cpp` entfernt (2026-04-13) — `user_led` (LED1) übernimmt in allen Test-Funktionen
- Endschalter 360°-Servo: A2 (PC_5), `DigitalIn` PullUp, getestet — noch nicht in roboter_v10 integriert
- roboter_v10: SMALL_REENTRY_GUARD=100 (2s) verhindert Stopp auf Farbkarte statt echtem Querbalken
- `src/test_files/` Unterordner erstellt — alle `test_*.cpp/.h` darin; Roboter-Files in `src/` als `prototype01_vX.cpp/.h` (v1–v8) und `prototype02_vX.cpp/.h` (v9–v10)
- Hue-Grenze GRÜN: 215° (nicht 175°) — NIEMALS zurücksetzen, GRÜN→BLAU-Fehler kehrt sonst zurück
- Kalibrierung mit echter Matte NICHT machen ohne gleichzeitige Hue-Grenzen-Anpassung — verschiebt alle Werte
- Servo-Kalibrierung abgeschlossen (2026-03-26): A=(0.0303–0.1204), B=(0.0314–0.1232), 360°=(0.0303–0.1223, Stop=0.0763)
- `TEST_LINE_FOLLOWER_FAST` ist stabil und funktioniert (Commit 183603b) — nicht anfassen
- Gear Ratio: 100:1 verbaut (`USE_GEAR_RATIO_78 = false`), KN = 140/12 ≈ 11.67 rpm/V
- v10: `VEL_SIGN=1.0f`, M1=physisch rechts, M2=physisch links (nach physischem Motortausch)
- v10: Linienfolgezuweisung gekreuzt: `g_cmd_M1=getLeftWheelVelocity()`, `g_cmd_M2=getRightWheelVelocity()` (Sensorbalken umgekehrt montiert)
- v10: STATE_FOLLOW Eintritt mit niedrigen Gains (KP_FOLLOW=0.8f, KP_NL_FOLLOW=0.5f), STATE_REAL/SMALL_FOLLOW stellen KP=2.8/KP_NL=4.55 wieder her
- SensorBar I2C1: SDA = PB_9, SCL = PB_8
- Eigen-Lib: `EIGEN_NO_DEBUG` + `EIGEN_DONT_VECTORIZE` (Overhead-Reduktion auf MCU)

## Projekt-Kontext
- **Kurs:** ZHAW 2. Semester, Modul PM2
- **Abgabe:** 23.05.2026 (Abend)
- **Team:** 6 Personen — 3x Elektronik & Programmierung, 3x Mechanik (CAD)

## Nächste Schritte
1. **Kurventuning v10:** KP / KP_NL / MAX_SPEED in `roboter_v10.cpp` so tunen, dass der Roboter Kurven sauber durchfährt ohne die Linie zu verlieren — aktuelle Werte (KP=2.8, KP_NL=4.55, MAX_SPEED=1.0) auf neuem Chassis noch nicht getestet
2. **Gesamtprogramm zum Laufen bringen:** Nach Kurventuning kompletten Parcours fahren — alle 4 Querbalken + 4 kleine Linien + Servo-Aktionen in richtiger Reihenfolge
3. **Farbsensor im Stand auf schwarzem Strich prüfen:** Testen ob `g_cs->getColor()` zuverlässig erkennt wenn Roboter steht UND Sensor über dem schwarzen Strich (Linie) ist — nicht über der Farbkarte

## Offene Fragen
- Funktioniert `g_cs->getColor()` zuverlässig wenn Roboter steht (Mechanik geändert)? Bisher nur während Fahrt getestet
- Muss der Farblesezeitpunkt in `CROSSING_STOP` verschoben werden (z.B. nach 0.5s Stillstand)?
- Wie sieht die Arm-Bewegung mit den 180°-Servos bei den Crossings aus? (Winkel, Sequenz, Zeiten noch unbekannt)
- Was passiert bei FINAL_HALT nach den 4 kleinen Linien — braucht es noch eine finale Aktion?
- Endschalter (A2, PC_5) noch nicht in roboter_v10 integriert — wann soll der 360°-Servo per Endschalter statt Zeitbasis gestoppt werden?

## Session-Routine
Am Ende jeder Session:
1. Neue Architektur-Entscheidungen → Abschnitt "Aktive Entscheidungen" ergänzen
2. Neue Erkenntnisse / Bugs → Auto Memory speichern
3. Offene Punkte → Abschnitt "Offene Fragen" aktualisieren
