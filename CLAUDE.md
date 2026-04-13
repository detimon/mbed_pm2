# mbed_pm — PES Board Roboter (Nucleo F446RE)

## Aktueller Stand
_Wird am Ende jeder Session via `/sesh-end` aktualisiert._ (2026-04-13)
- **Aktiv in test_config.h:** `TEST_ENDSCHALTER` — muss auf `TEST_ROBOTER_V8` zurückgesetzt werden vor nächstem Roboter-Test
- **roboter_v8** = aktive Roboter-Version, funktioniert vollständig; **roboter_v7** = Backup (nicht löschen)
- **Neu (heute):** `test_endschalter.cpp/.h` erstellt — Endschalter für 360°-Servo (Drehteller) an **A2 (PC_5)**, getestet, funktioniert
  - `DigitalIn` mit internem PullUp, Polling alle 20ms, fallende Flanke = gedrückt
  - A0 (PC_2) war physisch zu eng → A2 (PC_5) gewählt; A1 (PC_3) belegt durch ColorSensor LED Enable
- **Aufgeräumt (heute):** `led1` (PB_9/PB_10) aus `main.cpp` entfernt — `user_led` (LED1 onboard) übernimmt überall
- **Dateistruktur:** Alle `test_*`-Files in `src/test_files/`; `roboter_v*.cpp/.h` bleiben direkt in `src/`
- **Servo-Logik (v8):**
  - ROT (3): `g_servo->enable(0.10f)` — 360° schnell, **3s** (`SERVO_ROT_LOOPS=150`)
  - GELB (4): `g_servo->enable(0.35f)` — 360° langsam, **1s** (`SERVO_GELB_LOOPS=50`)
  - GRÜN (5): `g_servo_D1->enable(1.0f)` — 180° Servo A (PC_8), nach 1s zurück
  - BLAU (7): `g_servo_D2->enable(1.0f)` — 180° Servo B (PC_6), nach 1s zurück
  - Default: `g_servo->enable(0.25f)` — 360° Fallback, 1s
- **Timing:** `CROSSING_STOP_LOOPS=200` (4s), `SMALL_CROSSING_STOP_LOOPS=200` (4s), `SMALL_REENTRY_GUARD=100` (2s)
- **Hue-Grenzen (lib/ColorSensor/ColorSensor.cpp):**
  - ROT: 0°–20° und 330°–360°
  - GELB: 20°–80°
  - GRÜN: 80°–**215°** (war 175° — verbreitert gegen BLAU-Verwechslung)
  - BLAU: 215°–280°
- **Kalibrierung:** Original (weisses Papier) aktiv — Matte-Kalibrierung verschiebt Hue-Werte → nicht machen
- **Farblesung:** `m_action_color = g_cs->getColor()` im selben Loop wie `wide_bar_active()` / `small_line_active()` — vor `setVelocity(0)` → Sensor steht noch über Karte

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
- `TEST_ENDSCHALTER` ist aktuell aktiv in `test_config.h` — vor Roboter-Test auf `TEST_ROBOTER_V8` zurücksetzen
- `led1` (PB_9/PB_10) aus `main.cpp` entfernt (2026-04-13) — `user_led` (LED1) übernimmt in allen Test-Funktionen
- Endschalter 360°-Servo: A2 (PC_5), `DigitalIn` PullUp, getestet — noch nicht in roboter_v8 integriert
- roboter_v8: SMALL_REENTRY_GUARD=100 (2s) verhindert Stopp auf Farbkarte statt echtem Querbalken
- `src/test_files/` Unterordner erstellt — alle `test_*.cpp/.h` darin; `roboter_v*.cpp/.h` bleiben in `src/`
- `test_config.h` neu gegliedert: Roboter oben → Farbsensor → Hardware Tests → Liniensensor → Sonstiges
- Hue-Grenze GRÜN: 215° (nicht 175°) — NIEMALS zurücksetzen, GRÜN→BLAU-Fehler kehrt sonst zurück
- Kalibrierung mit echter Matte NICHT machen ohne gleichzeitige Hue-Grenzen-Anpassung — verschiebt alle Werte
- Neukalibierung mit Matte getestet: GRÜN wurde als GELB erkannt → sofort zurückgesetzt auf Paper-Kalibrierung
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
1. **TEST_ROBOTER_V8 reaktivieren:** In `test_config.h` `#define TEST_ENDSCHALTER` auskommentieren und `#define TEST_ROBOTER_V8` einkommentieren → flashen → Roboter-Durchgang testen
2. **Endschalter in roboter_v8 integrieren:** `DigitalIn sw(PC_5, PullUp)` einbauen, bei fallender Flanke Servo stoppen statt zeitbasiert warten
3. **Robustheit testen:** Mindestens 10 Durchgänge mit allen 4 Farben auf dem echten Parcours → Fehlerrate notieren

## Offene Fragen
- Wie sieht die Arm-Bewegung mit den 180°-Servos bei den Crossings aus? (Winkel, Sequenz, Zeiten noch unbekannt — aktuell fährt Servo einfach auf 1.0f und zurück)
- Was passiert bei FINAL_HALT nach den 4 kleinen Linien — braucht es noch eine finale Aktion?
- IR-Sensor noch nicht kalibriert — wird für spätere Integration benötigt
- Kalibrierung mit echter Matte verschiebt alle Hue-Werte → nicht empfohlen ohne gleichzeitige Hue-Grenzen-Anpassung
- Endschalter (A2, PC_5) noch nicht in roboter_v8 integriert — wann soll der 360°-Servo per Endschalter statt Zeitbasis gestoppt werden?

## Session-Routine
Am Ende jeder Session:
1. Neue Architektur-Entscheidungen → Abschnitt "Aktive Entscheidungen" ergänzen
2. Neue Erkenntnisse / Bugs → Auto Memory speichern
3. Offene Punkte → Abschnitt "Offene Fragen" aktualisieren
