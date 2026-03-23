# mbed_pm — PES Board Roboter (Nucleo F446RE)

## Aktueller Stand
_Wird am Ende jeder Session via `/sesh-end` aktualisiert._
- **Zuletzt:** `-Wreorder` Warnings in `ColorSensor.cpp` gefixt — 0 Problems (2026-03-23)
- **Fix:** Initialisierungsreihenfolge in beiden ColorSensor-Konstruktoren korrigiert (`m_S0`-`m_S3` vor `m_Led`), damit sie der Deklarationsreihenfolge im Header entspricht — kein Verhaltensunterschied
- **mbed:** Kein neuer Funktionscode — `TEST_SERVO_CALIB` ist aktiv, bereit zum Flashen in der Werkstatt
- **Offen:** Noch keine Hardware-Tests durchgeführt (nicht in der Werkstatt)

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
- `TEST_SERVO_CALIB` ist aktuell aktiv in `test_config.h` (bereit für Werkstatt-Kalibrierung)
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
1. `TEST_SERVO_CALIB` ist bereits aktiv → Flashen, Serial Monitor öffnen (115200 baud), alle 3 Servos kalibrieren (`+`/`-`/`c`/`f`), dann die ausgegebenen `calibratePulseMinMax()`-Werte in `src/test_servo_all.cpp` eintragen
2. `TEST_SERVO_ALL` aktivieren → alle 3 Servos zusammen testen und Bewegung verifizieren
3. `TEST_IR` aktivieren → IR-Sensor (PB_1/A3) Rohwerte in mV bei verschiedenen Abständen notieren → Kalibrierung mit `setCalibration(a, b)` bestimmen
4. Hauptprogramm schreiben: State Machine aus Line-Follower + Servo + IR-Sensor

## Offene Fragen
- Wie sieht die finale State Machine des Hauptprogramms aus?

## Session-Routine
Am Ende jeder Session:
1. Neue Architektur-Entscheidungen → Abschnitt "Aktive Entscheidungen" ergänzen
2. Neue Erkenntnisse / Bugs → Auto Memory speichern
3. Offene Punkte → Abschnitt "Offene Fragen" aktualisieren
