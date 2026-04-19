# mbed_pm — PES Board Roboter (Nucleo F446RE)

## Aktueller Stand
_Wird am Ende jeder Session via `/sesh-end` aktualisiert._ (2026-04-19)
- **Aktiv in test_config.h:** `PROTOTYPE_02_V11`
- **roboter_v11** = aktive Version; **roboter_v10** = Backup
- **Endschalter-Bug gelöst (2026-04-19):** Pin war falsch — SIG ist physisch an **PC_2** (A0 auf PES-Board), nicht PC_5. Korrigiert in `src/prototype02_v11.cpp:204` und `src/test_files/test_endschalter.cpp:18`. Endschalter liest jetzt korrekt: `raw=1` offen, `raw=0` gedrückt. Test mit `TEST_ENDSCHALTER` bestätigt.
- **360°-Servo Stopp-Mechanik komplett überarbeitet (2026-04-19):**
  - **Debounce-Logik:** `m_endstop_released` Flag — Endschalter muss erst einmal losgelassen werden (`read()==1`) bevor `==0` als Stopp-Signal zählt. Verhindert sofortiges Stoppen wenn Servo auf Ausgangs-Endschalter steht.
  - **Aktive Reverse-Bremsung:** Bei Endschalter-Trigger → 2 Loops (40 ms) Gegenpuls `SERVO360_BRAKE_PULSE=0.23f` (gespiegelt um 0.5) → danach Stop-Mitte `0.5f`. Kein `disable()` — Servo bleibt unter PWM-Kontrolle, coastet nicht.
  - **Einheitliche Geschwindigkeit:** Konstante `SERVO360_ALIGN_SPEED=0.77f` für ROT, GELB, BLAU-Phase und Initial-Ausrichtung. Zwischen Totzone (~0.75) und Überschiess-Bereich (0.80).
  - **ROT + GELB stoppen jetzt auch am Endschalter** (statt zeitbasiert `SERVO_ROT_LOOPS`/`SERVO_GELB_LOOPS`).
- **Farbsensor komplett neu kalibriert (2026-04-19):** Sensor-Halterung wurde physisch umdesignt → alle Hue-Werte verschoben:
  - **BLAU 348°–7°** (wrap um 0°, war 218°–280°) — Priorität VOR WHITE-Check, damit low-sat Blau nicht als UNKNOWN klassifiziert wird
  - **ROT 7°–20°** (war 0°–20° + 330°–360°)
  - **WHITE/UNKNOWN 20°–32°** (war 180°–218°)
  - **GELB 32°–52°** (war 20°–80°, erweitert um ±2° um Messbereich 37°–40°)
  - **GRÜN 52°–80°** (war 80°–180°)
  - **WHITE → UNKNOWN (candidate=0):** satp-Check liefert jetzt 0 statt 2, weisse Karte triggert keine Aktion
- **BLAU-Sequenz:** D1 fährt jetzt **85% aus** (0.85f, war 0.2/0.6) → D2 senken/heben → D1 ein → 360° dreht bis Endschalter (nicht mehr 10 Loops fix).
- **Offene Probleme:**
  1. **Überschiessen noch zu prüfen** — aktive Reverse-Bremsung mit 2 Loops eingebaut, aber noch nicht am echten Roboter validiert. Falls 90° statt 180° Überschiessen: `SERVO360_BRAKE_LOOPS` erhöhen.
  2. **Angle Clamp Auswirkung** — noch nicht verifiziert ob 0.15 rad richtig ist.

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

## Projekt-Kontext
- **Kurs:** ZHAW 2. Semester, Modul PM2
- **Abgabe:** 23.05.2026 (Abend)
- **Team:** 6 Personen — 3x Elektronik & Programmierung, 3x Mechanik (CAD)

## Nächste Schritte
1. **Flash `PROTOTYPE_02_V11` und validieren ob aktive Reverse-Bremsung das Überschiessen behebt.** Erwartung: 360°-Servo stoppt präzise ~90° (nicht mehr 180°) am Endschalter. Falls weiterhin Überschiessen: in `src/prototype02_v11.cpp:75–77` Parameter anpassen — `SERVO360_BRAKE_LOOPS` schrittweise auf 3–4 erhöhen und/oder `SERVO360_BRAKE_PULSE` weiter von 0.5 entfernen (z.B. 0.18f für stärkeren Gegenpuls). Test-Farben: ROT, GELB, BLAU, sowie Initial-Ausrichtung beim Start.
2. **GELB 360°-Servo Richtung prüfen** — mit `SERVO360_ALIGN_SPEED=0.77f` dreht GELB jetzt in die gleiche Richtung wie ROT/BLAU. Falls er in die falsche Richtung dreht: für GELB eigenen Wert `< 0.5f` verwenden (z.B. 0.23f).
3. **Angle Clamp validieren** — Roboter auf gerader Strecke und in Kurven fahren lassen. Falls träge: `MAX_ANGLE_STEP` in `lib/LineFollower/LineFollower.cpp` erhöhen oder `ANGLE_CLAMP_ENABLED` auskommentieren.
4. **Gesamtprogramm durchgehend fahren:** 4 Querbalken + 4 kurze Linien + Servo-Aktionen in richtiger Reihenfolge.

## Offene Fragen
- **Reicht 2 Loops Reverse-Bremsung?** — Bei 0.77f Geschwindigkeit und Servo-Masse → Erfahrungswert prüfen. Option: Bremspuls-Dauer dynamisch nach `SERVO360_ALIGN_SPEED` skalieren.
- **GELB Drehrichtung** — vor dem Pin-Fix war GELB bei 0.80 und hat sich gar nicht korrekt gedreht. Jetzt mit 0.77f identisch zu ROT/BLAU — soll GELB in die gleiche Richtung drehen oder in die entgegengesetzte? Falls entgegengesetzt: eigene Konstante.
- **Angle Clamp 0.15 rad/Loop** — noch nicht validiert.
- Was passiert bei FINAL_HALT nach den 4 kleinen Linien — braucht es eine finale Aktion?

## Session-Routine
Am Ende jeder Session:
1. Neue Architektur-Entscheidungen → Abschnitt "Aktive Entscheidungen" ergänzen
2. Neue Erkenntnisse / Bugs → Auto Memory speichern
3. Offene Punkte → Abschnitt "Offene Fragen" aktualisieren
