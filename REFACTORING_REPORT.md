# Refactoring Report — LineXpress FS26

**Datum:** 2026-05-12
**Scope:** Aufräumen der Test-Files, Anlegen eines zentralen Kalibrierungs-Ordners, Inbetriebnahmeprotokolle.
**Build-Status:** noch zu verifizieren (`pio run`).

---

## Umbenannte Dateien

| Alt (in `src/test_files/`) | Neu | Zielordner |
|----------------------------|-----|------------|
| `test_dc_motor.*`               | `test_motor_drive.*`        | `/test/` |
| `test_ir.*`                     | `test_ir_sensor.*`          | `/test/` |
| `test_servo.*`                  | `test_servo_180.*`          | `/test/` |
| `test_drehservo_90.*`           | `test_servo_180_angle.*`    | `/test/` |
| `test_servo_all.*`              | `test_servo_combined.*`     | `/test/` |
| `test_line_follower.*`          | `test_linefollower.*`       | `/test/` |
| `test_line_follower_SLOW.*`     | `test_linefollower_slow.*`  | `/test/` |
| `test_line_follower_FAST.*`     | `test_linefollower_fast.*`  | `/test/` |
| `test_line_follower_BACKWARD.*` | `test_linefollower_bwd.*`   | `/test/` |
| `test_color_sensor.*`           | `test_color_detect.*`       | `/test/` |
| `test_neopixel.*`               | `test_neopixel_basic.*`     | `/test/` |
| `test_color_neopixel.*`         | `test_neopixel_color.*`     | `/test/` |
| `test_logic_arm_standard.*`     | `test_arm_logic.*`          | `/test/` |
| `test_arm_sequence.*`           | `test_arm_sequence.*` (Name unverändert) | `/test/` |
| `test_all.*`                    | `test_system_all.*`         | `/test/` |
| `test_servo_calib.*`            | `calib_servo_180.*`         | `/calibration/` |
| `test_parallax_360.*`           | `calib_servo_360.*`         | `/calibration/` |
| `test_color_sensor_calib.*`     | `calib_color_hue.*`         | `/calibration/` |

Funktionsnamen wurden synchron zu den Dateinamen mitgezogen (z. B. `dc_motor_init/_task/_reset/_print` → `motor_drive_*`).
`#ifdef`-Gates wurden umbenannt (z. B. `TEST_DC_MOTOR` → `TEST_MOTOR_DRIVE`, `TEST_SERVO_CALIB` → `CALIB_SERVO_180`).

## Erstellte Dateien

| Datei | Zweck |
|-------|-------|
| `/calibration/calibration_values.h` | Zentrale Kalibrierungskonstanten (Hue-Schwellen, Servo-Pulsbreiten, P-Regler-Tuning, Print-Throttle) |
| `/test/`-Ordner | Sammlung aller Hardware-Tests |
| `/calibration/`-Ordner | Sammlung der Kalibrierungs-Skripte + Werte-Header |
| `REFACTORING_REPORT.md` | dieses Dokument |

## Geänderte Build-/Konfig-Dateien

| Datei | Änderung |
|-------|----------|
| `src/main.cpp` | 16 `#include`-Blöcke + Funktions-Aliasse + ein `#ifdef` umgestellt; Includes greifen jetzt auf `../test/` bzw. `../calibration/` zu |
| `src/test_config.h` | Alle Test-`#define`-Bezeichner umbenannt (TEST_DC_MOTOR → TEST_MOTOR_DRIVE etc.); 3 Kalibrierungs-Defines neu (CALIB_*) |
| `platformio.ini` | `build_src_filter` und `-I`-Pfade für `/test/` + `/calibration/` ergänzt; `test_dir = unit_tests` global gesetzt (verhindert Konflikt mit PlatformIO-Unit-Test-Konvention) |

## Ersetzte Magic Numbers

| Datei | Alter Wert (lokal) | Neue Konstante (in `calibration_values.h`) |
|-------|--------------------|-----|
| `calib_color_hue.cpp`  | `DISPLAY_LINES = 10`            | `HUE_DISPLAY_LINES` |
| `calib_color_hue.cpp`  | `PRINT_EVERY_N = 25`            | `HUE_PRINT_EVERY_N` |
| `calib_servo_360.cpp`  | `KP = 0.005f`                   | `SERVO_360_KP` |
| `calib_servo_360.cpp`  | `TOLERANCE_DEG = 2.1f`          | `SERVO_360_TOLERANCE_DEG` |
| `calib_servo_360.cpp`  | `MIN_SPEED = 0.01f`             | `SERVO_360_MIN_SPEED` |
| `calib_servo_360.cpp`  | `ANGLE_OFFSET = 10.0f`          | `SERVO_360_ANGLE_OFFSET` |
| `calib_servo_360.cpp`  | `PAUSE_LOOPS = 50`              | `SERVO_360_PAUSE_LOOPS` |
| `calib_servo_360.cpp`  | `WARMUP_LOOPS = 25`             | `SERVO_360_WARMUP_LOOPS` |
| `calib_servo_360.cpp`  | `TIMEOUT_LOOPS = 250`           | `SERVO_360_TIMEOUT_LOOPS` |
| `calib_servo_360.cpp`  | `PRINT_EVERY = 10`              | `SERVO_360_PRINT_EVERY` |
| `calib_servo_180.cpp`  | `PULSE_LOW = 0.010f`            | `SERVO_180_PULSE_LOW` |
| `calib_servo_180.cpp`  | `PULSE_HIGH = 0.150f`           | `SERVO_180_PULSE_HIGH` |
| `calib_servo_180.cpp`  | `WAIT_LOOPS = 50`               | `SERVO_180_WAIT_LOOPS` |
| `calib_servo_180.cpp`  | `SWEEP_DIV = 4`                 | `SERVO_180_SWEEP_DIV` |
| `calib_servo_180.cpp`  | `SWEEP_STEP = 0.001f`           | `SERVO_180_SWEEP_STEP` |
| `calib_servo_180.cpp`  | `>= 10` (Goto-Loops)            | `SERVO_180_GOTO_LOOPS` |
| `calib_servo_180.cpp`  | `PRINT_EVERY_N = 5`             | `SERVO_180_PRINT_EVERY_N` |
| `calib_servo_180.cpp`  | `DISPLAY_LINES = 9`             | `SERVO_180_DISPLAY_LINES` |

`calibration_values.h` enthält darüber hinaus Hue-Grenzen (aus `lib/ColorSensor/ColorSensor.cpp`, Stand 2026-05-12) und Servo-360-Speed-Konstanten als Dokumentations-Anker. Diese werden derzeit von `lib/ColorSensor` selbst gehalten; eine Auslagerung dorthin würde die Library berühren (Sperrzone) und wurde nicht vorgenommen.

## Inbetriebnahmeprotokoll

In jede der 18 bearbeiteten `.cpp`-Dateien wurde ein Kopf-Block eingefügt mit folgenden Feldern:
**Datei**, **Komponente**, **BMK**, **Pin(s)**, **Testziel**, **Voraussetzungen**, **Erwartetes Verhalten**, **Testdatum**, **Tester**, **Ergebnis**, **Bemerkungen**.

Jeder Block ist dateispezifisch ausgefüllt (kein Copy-Paste). Testdatum/Tester/Ergebnis sind als Klartext-Platzhalter zum Ausfüllen.

## Gesperrte Dateien (unberührt)

- `src/main.cpp`           → freigegeben für `#include`-Anpassung, sonst unverändert
- `src/main.h`              → keine Datei vorhanden, kein Handlungsbedarf
- `include/PESBoardPinMap.h` → nur gelesen
- `src/test_files/test_endschalter.cpp` + `.h` → Konzept geändert, vollständig unberührt
- `src/cargosweep_team5_20260512.*`         → aktives Hauptprogramm, unberührt
- `src/old_prototyping_files/**`            → ältere Roboter-Versionen, unberührt

## Offene TODOs

| Datei | Beschreibung |
|-------|--------------|
| `calibration_values.h` | `SERVO_360_FWD_FAST/SLOW`, `SERVO_360_BWD_FAST/SLOW`, `SERVO_360_DEADBAND` sind Schätzwerte — bei Bedarf mit `calib_servo_360` nachmessen und Datum/Initialen aktualisieren. |
| `calibration_values.h` | `HUE_*` und `SERVO_180_*_PULSE`-Werte sind die jeweils aktuellen — bei jeder Neukalibrierung Datum/Initialen mitführen. |
| Alle 18 Files | Inbetriebnahmeprotokoll-Felder *Testdatum*, *Tester*, *Ergebnis*, *Bemerkungen* nach realem Test ausfüllen. |
| `lib/ColorSensor/ColorSensor.cpp` | Hue-Grenzen leben aktuell dort. Eine Single-Source-of-Truth via `calibration_values.h` würde die Library berühren — nicht im Scope dieses Refactors. |
| (optional) | Funktions-Längen-Audit (Regel: max. 30 Zeilen) wurde nicht durchgeführt; bestehende Sequenzen waren zu komplex, um sie ohne Verhaltensänderung zu zerlegen. Bei nächstem Refactor angehen. |
| (optional) | Bestehende deutsche Inline-Kommentare in den Test-Files wurden NICHT auf Englisch übersetzt — würde Reviewbarkeit gegen Diffs der vorigen Sessions verschlechtern. Neue Kommentare (im Inbetriebnahmekopf) sind Deutsch-konform mit den restlichen Files. |

## Hinweis zum Build

Da PlatformIO standardmässig nur `src/` baut, wurde `build_src_filter` so erweitert, dass auch `/test/` und `/calibration/` einbezogen werden. Inkludiert wird in main.cpp via relativem Pfad `../test/` bzw. `../calibration/`.

`test_dir` wurde in `[platformio]` auf `unit_tests` umgebogen, weil PlatformIO standardmässig den Ordnernamen `test/` für sein eigenes Unit-Test-Framework verwendet. Mit dieser Umlenkung bleibt unser `/test/` ein reiner Hardware-Test-Ordner.
