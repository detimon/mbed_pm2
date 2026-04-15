# mbed_pm — PES Board Roboter (Nucleo F446RE)

## Aktueller Stand
_Wird am Ende jeder Session via `/sesh-end` aktualisiert._ (2026-04-15)
- **Aktiv in test_config.h:** `PROTOTYPE_02_V10`
- **roboter_v10** = aktive Version; **roboter_v9** = Backup
- **Motor-Zuordnung v10:** M1 = physisch rechts, M2 = physisch links; `VEL_SIGN=1.0f`
- **MAX_SPEED jetzt 1.6 RPS** (zuvor 1.0) → `SPEED_SCALE = 1/1.6 = 0.625`
- **Timings heute angepasst:**
  - `CROSSING_STOP_LOOPS` 200 → **100** (2.0 s)
  - `SMALL_CROSSING_STOP_LOOPS` 200 → **100** (2.0 s)
  - `SERVO_ROT_LOOPS` 150 → **75** (1.5 s) — muss <= CROSSING_STOP_LOOPS sein
  - `STRAIGHT_LOOPS` 85 → **70** (1.4 s)
  - `BACKWARD_LOOPS` 210 → **222** (4.44 s)
  - `SMALL_FOLLOW_START_GUARD`-Basis 563 → **656** → 410 Loops = 8.2 s bei MAX_SPEED=1.6 (Strecke ≈ 1.2 m)
- **Neu: Farbbasierte Bremsrampe vor Querlinien & kurzen Linien** in `STATE_REAL_FOLLOW` und `STATE_SMALL_FOLLOW`:
  - `SLOWDOWN_LOOPS=25` (0.5 s Rampe), `SLOW_FACTOR=0.4f` (40 % der aktuellen Befehlsgeschwindigkeit)
  - Trigger: `g_cs->getColor()` liefert 3/4/5/7 für `COLOR_STABLE_CNT=5` Loops in Folge (Debouncing gegen Fehlmessungen)
  - `COLOR_READ_DELAY=50` (1.0 s): nach jedem Linien-Stopp wird `getColor()` für 1 s komplett ignoriert, damit alte Farbkarte unter dem Sensor keinen Retrigger auslöst
  - `m_action_color` wird direkt beim Slowdown-Start gesetzt; Linien-Trigger (wide_bar/small_line) liest nur noch als Fallback nach
  - Reset der Slow/Delay/Pending-Counter beim Austritt aus `CROSSING_STOP` / `SMALL_CROSSING_STOP`
- **Gelöst (2026-04-15):** Früh-Stopp nach 3. kurzer Linie — behoben durch verlängerte `SMALL_FOLLOW_START_GUARD`-Basis (563 → 656, jetzt 8.2 s Sperrzeit nach 4. Querbalken). Die Phantom-Detektion kam also aus dem Bereich unmittelbar nach dem 4. Querbalken, nicht zwischen den echten kurzen Linien.
- **Offene Probleme:**
  1. **Roboter findet die Folgelinie nach dem 1. breiten Balken nicht** — konkretes Verhalten noch nicht diagnostiziert (Stoppt er? Fährt er geradeaus weg? In welche Richtung?). Möglicher Kontext: `STOP_GUARD = 75 * SPEED_SCALE` (1.5 s bei MAX_SPEED=1.0 / ~47 Loops = 0.94 s bei MAX_SPEED=1.6) könnte zu kurz sein, sodass der Line-Follower die Linie im Guard-Fenster noch nicht wiederfindet, oder der Roboter nach dem Stopp seitlich versetzt ist.
  2. **Farbbremsung braucht weiteres Tuning** — Parameter `SLOWDOWN_LOOPS=25` (0.5 s) und `SLOW_FACTOR=0.4f` noch nicht final. `COLOR_STABLE_CNT=5` evtl. nicht ausreichend gegen Phantom-Reads.
- **Servo-Logik (unverändert):**
  - ROT (3): `g_servo->enable(0.10f)` — 360° schnell, jetzt 1.5 s
  - GELB (4): `g_servo->enable(0.35f)` — 360° langsam, 1 s (`SERVO_GELB_LOOPS=50`)
  - GRÜN (5): `g_servo_D1->enable(1.0f)` — 180° Servo A, nach 1 s zurück
  - BLAU (7): `g_servo_D2->enable(1.0f)` — 180° Servo B, nach 1 s zurück
- **Hue-Grenzen:** ROT 0°–20°/330°–360°, GELB 20°–80°, GRÜN 80°–215°, BLAU 215°–280° (unverändert)
- **Kalibrierung:** Original (weisses Papier) — Matte-Kalibrierung nicht machen

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
- v10 (2026-04-15): Farbbremsrampe — wer bei `col ∈ {3,4,5,7}` bremst, nutzt `SLOWDOWN_LOOPS=25` / `SLOW_FACTOR=0.4f` mit Debouncing `COLOR_STABLE_CNT=5` und Nach-Stopp-Sperre `COLOR_READ_DELAY=50` (1 s)
- v10 (2026-04-15): MAX_SPEED=1.6, CROSSING_STOP_LOOPS=100, SMALL_CROSSING_STOP_LOOPS=100, SERVO_ROT_LOOPS=75, STRAIGHT_LOOPS=70, BACKWARD_LOOPS=222, SMALL_FOLLOW_START_GUARD-Basis=656

## Projekt-Kontext
- **Kurs:** ZHAW 2. Semester, Modul PM2
- **Abgabe:** 23.05.2026 (Abend)
- **Team:** 6 Personen — 3x Elektronik & Programmierung, 3x Mechanik (CAD)

## Nächste Schritte
1. **Linienwiedererkennung nach 1. breiten Balken debuggen:** `PROTOTYPE_02_V10` mit `pio device monitor` laufen lassen und beobachten, was nach dem Stopp am 1. Querbalken passiert — erkennt `STATE_REAL_FOLLOW` die Linie nicht mehr (getAvgBit zeigt nichts), oder rollt der Roboter seitlich von der Linie weg während des `STOP_GUARD`? Je nach Befund: `STOP_GUARD` verlängern, den State beim Eintritt mit 0-Velocity-Pause versehen, oder den Roboter vor Weiterfahrt auf die Linie zurückrotieren lassen.
2. **Farbbremsung tunen:** Beobachten, ob `SLOW_FACTOR=0.4f` zu langsam/zu schnell ist und ob `SLOWDOWN_LOOPS=25` (0.5 s Rampe) zur Abstands-Geometrie Farbkarte→Linie passt. Fehltrigger-Rate von `COLOR_STABLE_CNT=5` protokollieren und ggf. erhöhen.
3. **Gesamtprogramm durchgehend fahren:** 4 Querbalken + 4 kurze Linien + Servo-Aktionen in richtiger Reihenfolge ohne Fehler.

## Offene Fragen
- Warum findet der Roboter die Folgelinie nach dem 1. breiten Balken nicht? Seitlicher Versatz beim Stopp, zu kurzer `STOP_GUARD`, oder sieht der Sensorbalken den Rest des Querbalkens und interpretiert ihn falsch?
- Ist `COLOR_STABLE_CNT=5` (0.1 s) genug Debouncing, oder liefert der Farbsensor länger als 4 Loops Fehlmessungen (3/4/5/7 ohne echte Karte)?
- Farbkarte liegt laut User physisch VOR der Linie — wie breit ist der Abstand Karte→Linie? Reicht `SLOWDOWN_LOOPS=25` (0.5 s bei SLOW_FACTOR-reduzierter Geschwindigkeit) um vor der Linie zu enden?
- Wie sieht die Arm-Bewegung mit den 180°-Servos bei den Crossings aus? (Winkel, Sequenz, Zeiten noch unbekannt)
- Was passiert bei FINAL_HALT nach den 4 kleinen Linien — braucht es noch eine finale Aktion?
- Endschalter (A2, PC_5) noch nicht in roboter_v10 integriert — wann soll der 360°-Servo per Endschalter statt Zeitbasis gestoppt werden?

## Session-Routine
Am Ende jeder Session:
1. Neue Architektur-Entscheidungen → Abschnitt "Aktive Entscheidungen" ergänzen
2. Neue Erkenntnisse / Bugs → Auto Memory speichern
3. Offene Punkte → Abschnitt "Offene Fragen" aktualisieren
