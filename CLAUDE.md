# mbed_pm — PES Board Roboter (Nucleo F446RE)

## Aktueller Stand
_Wird am Ende jeder Session via `/sesh-end` aktualisiert._ (2026-04-18)
- **Aktiv in test_config.h:** `PROTOTYPE_02_V11`
- **roboter_v11** = aktive Version; **roboter_v10** = Backup (unverändert seit Kopie)
- **v11 erstellt als Kopie von v10** — alle Änderungen dieser Session in v11, v10 bleibt als Rollback
- **Änderungen v10→v11 (2026-04-18):**
  - **360° Servo Richtung umgekehrt:** ROT 0.10→0.90, GELB 0.20→0.80
  - **Anfahrrampe nach Crossing-Stops:** `RESTART_ACCEL_LOOPS=50` (1.0 s), geradeaus mit linearer Beschleunigung, kein LineFollower während Rampe. Danach erst LineFollower mit vollen Gains.
  - **Farbbremsung:** `SLOW_FACTOR=0.2f` (20%), `SLOWDOWN_LOOPS=13` (0.26 s), `COLOR_READ_DELAY=50` (1.0 s)
  - **wide_bar_active():** Alle 8 Sensoren (0–7) statt 1–7, Schwelle 4 statt 5 LEDs
  - **SMALL_FOLLOW_START_GUARD:** Basis 706 (vorher 656, +1 s)
  - **BLAU Servo-Sequenz (mehrstufig):** D1 ausfahren 20% → D2 senken 20% → D2 hoch → D1 einfahren → 360° drehen 10 Loops. D1=ausfahren/einfahren (0.0=ein, 1.0=aus), D2=heben/senken (1.0=oben, 0.0=unten)
  - **Servo-Ausrichtung beim Start:** Bei alle-8-Sensoren-aktiv (STATE_BLIND→STRAIGHT): D1 auf 0.0, D2 auf 1.0, 360° dreht langsam (0.80) bis Endschalter (PC_5, PullUp, aktiv=0)
  - **Endschalter integriert:** `DigitalIn(PC_5, PullUp)`, wird jeden Loop geprüft, stoppt 360°-Servo bei `read()==0`
  - **Angle Clamp im LineFollower:** `MAX_ANGLE_STEP=0.15f` rad pro Loop in `lib/LineFollower/LineFollower.cpp` — begrenzt Phantom-Winkelsprünge durch isolierte Sensor-Readings. Deaktivierbar via `#define ANGLE_CLAMP_ENABLED` auskommentieren.
- **Offene Probleme:**
  1. **360°-Servo stoppt nicht beim Endschalter** — LED zeigt Endschalter aktiv, aber Servo dreht weiter. Polarität (PullUp `read()==0`) und Geschwindigkeit (0.80) sind gesetzt, trotzdem kein Stopp. Muss debuggt werden.
  2. **GELB 360°-Servo (0.80)** — liegt möglicherweise noch in/nahe Totzone, muss getestet werden
  3. **Angle Clamp Auswirkung** — noch nicht verifiziert ob 0.15 rad der richtige Wert ist oder ob er Kurvenfahrt beeinträchtigt

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
- `PROTOTYPE_02_V11` ist aktuell aktiv in `test_config.h` (v10 bleibt als Backup)
- `led1` (PB_9/PB_10) aus `main.cpp` entfernt (2026-04-13) — `user_led` (LED1) übernimmt in allen Test-Funktionen
- Endschalter 360°-Servo: A2 (PC_5), `DigitalIn` PullUp — in v11 integriert, stoppt 360° bei `read()==0`
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
- Hue-Referenz (2026-04-16, weisses Papier, LED an): WHITE 204°–210°, BLAU 226°–232°, GRÜN 133°–143°, ROT 359°–1.5°, GELB 43.5°–44.5° — Quelle bei künftigen Hue-Grenzen-Entscheidungen.

## Projekt-Kontext
- **Kurs:** ZHAW 2. Semester, Modul PM2
- **Abgabe:** 23.05.2026 (Abend)
- **Team:** 6 Personen — 3x Elektronik & Programmierung, 3x Mechanik (CAD)

## Nächste Schritte
1. **Endschalter-Stopp debuggen:** 360°-Servo stoppt nicht bei Endschalter obwohl LED aktiv. Mögliche Ursachen: (a) `g_endstop->read()` liefert nicht 0 bei gedrücktem Schalter — `printf("endstop=%d\n", g_endstop->read())` in `roboter_v11_print()` einbauen und im Monitor beobachten. (b) Falls read() == 1 bei gedrückt → Bedingung auf `== 1` ändern. (c) Falls Servo trotz korrektem Read nicht stoppt → prüfen ob `g_servo->disable()` bei 360°-Servo tatsächlich stoppt (evtl. `setPulseWidth(0.5f)` + `disable()` nötig).
2. **GELB 360°-Servo testen** — aktueller Wert 0.80 könnte in Totzone liegen, ggf. auf 0.90 erhöhen.
3. **Angle Clamp validieren** — Roboter auf gerader Strecke und in Kurven fahren lassen, prüfen ob LineFollower-Verhalten sich verschlechtert hat. Falls ja: `MAX_ANGLE_STEP` in `lib/LineFollower/LineFollower.cpp` erhöhen (z.B. 0.25f) oder `ANGLE_CLAMP_ENABLED` auskommentieren.
4. **Gesamtprogramm durchgehend fahren:** 4 Querbalken + 4 kurze Linien + Servo-Aktionen in richtiger Reihenfolge ohne Fehler.

## Offene Fragen
- **Endschalter (PC_5) stoppt 360°-Servo nicht** — LED zeigt aktiv, aber `g_endstop->read()==0` scheint nicht zu triggern. Ist der Pin korrekt? Liefert der Endschalter wirklich 0 bei gedrückt (PullUp)?
- **Angle Clamp 0.15 rad/Loop** — reicht das für schnelle Kurven oder wird der LineFollower zu träge? Muss mit echter Fahrt verifiziert werden.
- **GELB 360°-Servo Totzone** — 0.80 funktioniert bei BLAU-Sequenz, aber GELB hat denselben Wert. War vorher 0.20 (andere Seite), jetzt gespiegelt auf 0.80. Muss getestet werden.
- Was passiert bei FINAL_HALT nach den 4 kleinen Linien — braucht es noch eine finale Aktion?

## Session-Routine
Am Ende jeder Session:
1. Neue Architektur-Entscheidungen → Abschnitt "Aktive Entscheidungen" ergänzen
2. Neue Erkenntnisse / Bugs → Auto Memory speichern
3. Offene Punkte → Abschnitt "Offene Fragen" aktualisieren
