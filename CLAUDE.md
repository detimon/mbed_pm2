# mbed_pm — PES Board Roboter (Nucleo F446RE)

## Aktueller Stand
_Wird am Ende jeder Session via `/sesh-end` aktualisiert._ (2026-04-10)
- **Aktiv:** `TEST_ROBOTER_V6` in `test_config.h`
- **roboter_v6** = roboter_v5 + farbbasierte Servo-Steuerung (Majority-Vote System)
- **Farbsensor root cause gefunden + gefixt (noch nicht getestet im Workshop):**
  - **PROBLEM:** Boden (Wettkampf-Matte) wurde als GELB klassifiziert weil Kalibrierung mit weissem Papier gemacht wurde, nicht mit der echten Matte → floor satp > 0, hue ≈ 20-60° → YELLOW → Hunderte GELB-Stimmen während Fahrt → immer m_action_color=4
  - **Fix 1:** `SATP_GRAY_MAX=0.01` (war 0.08) in `lib/ColorSensor/ColorSensor.cpp` — verhindert dass dilutierte Karten als WHITE durchfallen
  - **Fix 2:** `NORM_DELTA_MIN=0.03` (war 0.08) in `lib/ColorSensor/ColorSensor.cpp`
  - **Fix 3:** Hue-basierte Klassifikation in `getColor()` implementiert (statt g0/r0-Ratios)
  - **Fix 4:** Vote-Delta-Guard in `roboter_v6.cpp`: nur vote wenn `norm_max - norm_min > 0.20f` — verhindert dass near-neutral floor (miscalibriert als GELB) Stimmen sammelt
- **roboter_v6 Servo-Logik bei Crossing:**
  - ROT (3): `g_servo->enable(0.10f)` — 360° schnell
  - GELB (4): `g_servo->enable(0.35f)` — 360° langsam
  - GRÜN (5): `g_servo_D1->enable(1.0f)` — 180° Servo A (PC_8) ausfahren, nach 1s zurück
  - BLAU (7): `g_servo_D2->enable(1.0f)` — 180° Servo B (PC_6) ausfahren, nach 1s zurück
  - Default (kein Signal): `g_servo->enable(0.25f)` — 360° Fallback
- **Print in v6 zeigt:** State | wide= small= | Color hue= satp= r= g= b= | action= votes R= Y= G= B=
- **MUSS NOCH GEMACHT WERDEN:** Kalibrierung mit echter Wettkampf-Matte neu messen

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
- `TEST_ROBOTER_V6` ist aktuell aktiv in `test_config.h` (v1–v5 auskommentiert, als Backup erhalten)
- roboter_v6 = v5 + Majority-Vote Farbsensor-Steuerung + 3 Servos (360° + 2× 180°)
- roboter_v6 breite Balken-Trigger: `wide_bar_active()` = min. 5 von 8 Sensoren aktiv (Sensoren 1–7)
- roboter_v6 kleine Linien-Trigger: `small_line_active()` = Sensoren 3,4,5 ODER Sensoren 2,3,4 alle aktiv
- roboter_v6 Majority-Vote: Stimmen nur in REAL_APPROACH/REAL_FOLLOW/SMALL_FOLLOW; nur wenn `vote_delta > 0.20f`; pop_voted_color() beim Eintritt in CROSSING_STOP
- Farbsensor Root Cause: Boden-Kalibrierung falsch → Boden wird als GELB klassifiziert → immer GELB gewonnen. Fix: Neu kalibrieren mit echter Matte.
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
1. **KALIBRIERUNG ZUERST:** `TEST_COLOR_SENSOR_CALIB` in `test_config.h` aktivieren → flashen → Sensor im montierten Roboter-Zustand (exakte Einbauhöhe!) über die **Wettkampf-Bodenmatte** halten → Avg Hz R/G/B/W notieren (= neue White-Referenz) → Sensor über schwarze Fläche (schwarze Linie oder abgedeckt) halten → Avg Hz notieren (= neue Black-Referenz) → `setCalibration()` in `lib/ColorSensor/ColorSensor.cpp` mit diesen Werten aktualisieren
2. **TEST_ROBOTER_V6** aktivieren → flashen → Roboter fahren lassen mit jeder der 4 Farbkarten → Serial Monitor prüfen: `votes R=x Y=x G=x B=x` und `action=ROT/GELB/GRÜN/BLAU` — jede Karte muss ihren eigenen Servo auslösen
3. Falls vote_delta-Threshold 0.20f zu hoch/niedrig: Serial Monitor beobachten, `hue` und `r= g= b=` Werte für Boden vs. Karten vergleichen, Threshold anpassen
4. 180°-Servos (PC_8, PC_6) Winkel und Bewegungsablauf definieren und testen

## Offene Fragen
- **Kalibrierung noch nicht gemacht:** Hard-coded Referenzwerte in `setCalibration()` wurden mit weissem Papier gemessen, nicht mit echter Wettkampf-Matte → MUSS neu kalibriert werden (Schritt 1 oben)
- vote_delta-Threshold 0.20f: ungetesteter Wert — nach Kalibrierung prüfen ob er Boden filtert aber Karten durchlässt; ggf. anpassen
- Erkennt `small_line_active()` (Sensoren 2–5) die schmalen Linien zuverlässig? Muss evtl. OR statt AND verwendet werden oder andere Sensor-Indizes?
- Wie sieht die Arm-Bewegung mit den 180°-Servos bei den Crossings aus? (Winkel, Sequenz, Zeiten noch unbekannt)
- Was passiert bei FINAL_HALT nach den 4 kleinen Linien — braucht es noch eine finale Aktion?
- IR-Sensor noch nicht kalibriert — wird für spätere Integration benötigt

## Session-Routine
Am Ende jeder Session:
1. Neue Architektur-Entscheidungen → Abschnitt "Aktive Entscheidungen" ergänzen
2. Neue Erkenntnisse / Bugs → Auto Memory speichern
3. Offene Punkte → Abschnitt "Offene Fragen" aktualisieren
