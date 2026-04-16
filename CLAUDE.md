# mbed_pm — PES Board Roboter (Nucleo F446RE)

## Aktueller Stand
_Wird am Ende jeder Session via `/sesh-end` aktualisiert._ (2026-04-16)
- **Aktiv in test_config.h:** `TEST_COLOR_NEOPIXEL` (zum Validieren der Farberkennung während der Fahrt)
- **roboter_v10** = aktive Version; **roboter_v9** = Backup — **unverändert diese Session**
- **Neu: WS2812B NeoPixel-Treiber + Validierungs-Testmodul**
  - `lib/NeoPixel/` — Bit-Bang WS2812B-Treiber via DWT-Cycle-Counter (GRB-Byte-Order, T1H=122/T0H=58/BIT=220 Zyklen @ 180 MHz, `core_util_critical_section` um Timing). API: `NeoPixel(PinName)`, `setRGB()`, `show()`, `clear()`. Pin: **PB_2** (D0), 5 V-Versorgung.
  - `src/test_files/test_neopixel.{h,cpp}` — Zyklus-Test: ROT/GELB/GRÜN/BLAU/OFF je 1 s. Beweist die Ansteuerung.
  - `src/test_files/test_color_neopixel.{h,cpp}` — **Live-Spiegelung**: `ColorSensor` + `NeoPixel` zusammen. `getColor()` → gedimmte Zielfarbe (60/255 max) nur für die 4 Aktions-Farben 3/4/5/7, sonst AUS. Print-Zeile: `Color=NAME  Hue=XX.X°  RGBn=(...)`. Ziel: während der Fahrt manuell prüfen, ob der Sensor die richtige Farbe liest.
  - Aktivierungen in `src/test_config.h` (beide auskommentiert einsehbar), `#elif`-Zweige in `src/main.cpp` nach `TEST_ENDSCHALTER`.
- **Neu: Hue-Messungen (2026-04-16, weisses Papier, LED an):**
  - WHITE: 204°–210°  *(← problematisch, lag vorher im GRÜN-Korridor)*
  - BLAU: 226°–232°
  - GRÜN: 133°–143°
  - ROT: 359°–1.5° (wrap-around)
  - GELB: 43.5°–44.5°
- **Geändert: `HUE_GREEN_MAX` 215° → 180°** in [lib/ColorSensor/ColorSensor.cpp:291](lib/ColorSensor/ColorSensor.cpp#L291). BLAU beginnt damit bei 180°. Grund: WHITE-Hue (204°–210°) lag im 215°-Fenster als GRÜN und hat vermutlich die Phantom-Bremstrigger der letzten Session ausgelöst. Mit 180° liegt WHITE jetzt im BLAU-Korridor — Risiko nur noch relevant wenn der Sättigungs-Filter (`SATP_GRAY_MAX=0.01`, `C0_WHITE_MIN=650`) WHITE nicht abfängt.
- **Achtung:** bisherige CLAUDE.md-Regel "GRÜN-Max NIEMALS zurücksetzen (175° → GRÜN→BLAU-Fehler)" wurde bewusst überschrieben. Mit aktueller GRÜN-Messung (max 143°) bleibt 37° Puffer. Falls GRÜN-Karte erneut als BLAU gelesen wird → wieder hochsetzen (z.B. 195°).
- **Unverändert:**
  - MAX_SPEED=1.6, Motor-Zuordnung M1=rechts/M2=links, VEL_SIGN=1.0f, SPEED_SCALE=0.625
  - Timings: CROSSING_STOP_LOOPS=100, SMALL_CROSSING_STOP_LOOPS=100, SERVO_ROT_LOOPS=75, STRAIGHT_LOOPS=70, BACKWARD_LOOPS=222, SMALL_FOLLOW_START_GUARD-Basis=656
  - Farbbremsrampe: `SLOWDOWN_LOOPS=25`, `SLOW_FACTOR=0.4f`, `COLOR_READ_DELAY=50`, `COLOR_STABLE_CNT=5`
  - Servo-Logik (ROT=360°-schnell, GELB=360°-langsam, GRÜN=180°-A, BLAU=180°-B)
  - Gelöst 2026-04-15: 3.-Linie-Früh-Stopp durch SMALL_FOLLOW_START_GUARD-Basis 656
- **Offene Probleme:**
  1. **Roboter findet Folgelinie nach 1. breiten Balken nicht** — noch nicht diagnostiziert (seitlicher Versatz, zu kurzer STOP_GUARD, oder Sensor sieht Rest des Querbalkens?).
  2. **WHITE→BLAU-Risiko** nach der GRÜN-Grenzen-Änderung — muss mit TEST_COLOR_NEOPIXEL überfahren werden, um zu prüfen ob WHITE jetzt Phantom-BLAU triggert.
  3. **Farbbremsung** braucht noch Feintuning (SLOW_FACTOR, SLOWDOWN_LOOPS), vorher aber Farberkennung stabilisieren.

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
- v10 (2026-04-15): MAX_SPEED=1.6, CROSSING_STOP_LOOPS=100, SMALL_CROSSING_STOP_LOOPS=100, SERVO_ROT_LOOPS=75, STRAIGHT_LOOPS=70, BACKWARD_LOOPS=222, SMALL_FOLLOW_START_GUARD-Basis=656
- NeoPixel (2026-04-16): WS2812B Adafruit Flora v2 an PB_2 (D0), 5V, Bit-Bang-Treiber in `lib/NeoPixel/` (DWT-Cycle-Counter + critical_section). GRB-Byte-Order. Test-Module: `TEST_NEOPIXEL` (Zyklus-Selbsttest) und `TEST_COLOR_NEOPIXEL` (Live-Validierung Farbsensor).
- Hue-Referenz (2026-04-16, weisses Papier, LED an): WHITE 204°–210°, BLAU 226°–232°, GRÜN 133°–143°, ROT 359°–1.5°, GELB 43.5°–44.5° — Quelle bei künftigen Hue-Grenzen-Entscheidungen.

## Projekt-Kontext
- **Kurs:** ZHAW 2. Semester, Modul PM2
- **Abgabe:** 23.05.2026 (Abend)
- **Team:** 6 Personen — 3x Elektronik & Programmierung, 3x Mechanik (CAD)

## Nächste Schritte
1. **`TEST_COLOR_NEOPIXEL` flashen und mit allen 5 Karten + weissem Untergrund prüfen** (`pio run --target upload` → `pio device monitor`, User-Button zum Aktivieren). Dabei verifizieren: (a) bei ROT/GELB/GRÜN/BLAU-Karte leuchtet die NeoPixel in der richtigen Farbe und der Monitor zeigt die passende Hue im erwarteten Bereich; (b) über weissem Untergrund bleibt die NeoPixel **aus** (kein Phantom-BLAU nach der Hue-Grenzen-Änderung 215°→180°); (c) GRÜN-Karte wird nicht mehr mit BLAU verwechselt (altes Risiko der 175°-Grenze). Bei Phantom-BLAU über WHITE → `SATP_GRAY_MAX` in [ColorSensor.cpp:277](lib/ColorSensor/ColorSensor.cpp#L277) leicht erhöhen (z.B. auf 0.02f).
2. **Danach zurück auf `PROTOTYPE_02_V10`**, Roboter fahren lassen und gezielt die Linienwiedererkennung nach dem 1. breiten Balken debuggen.
3. **Farbbremsung final tunen** (SLOW_FACTOR, SLOWDOWN_LOOPS) — erst wenn Farberkennung bestätigt sauber ist.
4. **Gesamtprogramm durchgehend fahren:** 4 Querbalken + 4 kurze Linien + Servo-Aktionen in richtiger Reihenfolge ohne Fehler.

## Offene Fragen
- **Nach GRÜN-Grenze 215°→180°:** wird WHITE (Hue 204°–210°) jetzt als BLAU (7) klassifiziert, falls der Sättigungs-Filter versagt? → mit `TEST_COLOR_NEOPIXEL` über weissem Untergrund beobachten.
- Warum findet der Roboter die Folgelinie nach dem 1. breiten Balken nicht? Seitlicher Versatz beim Stopp, zu kurzer `STOP_GUARD`, oder sieht der Sensorbalken den Rest des Querbalkens und interpretiert ihn falsch?
- Ist `COLOR_STABLE_CNT=5` (0.1 s) genug Debouncing, oder liefert der Farbsensor länger als 4 Loops Fehlmessungen? (Die aktuelle Phantom-Bremsung kann jetzt auch durch die WHITE-Fehlklassifikation erklärt sein — nach Hue-Grenze-Fix erneut bewerten.)
- Farbkarte liegt laut User physisch VOR der Linie — wie breit ist der Abstand Karte→Linie? Reicht `SLOWDOWN_LOOPS=25` (0.5 s bei SLOW_FACTOR-reduzierter Geschwindigkeit) um vor der Linie zu enden?
- Wie sieht die Arm-Bewegung mit den 180°-Servos bei den Crossings aus? (Winkel, Sequenz, Zeiten noch unbekannt)
- Was passiert bei FINAL_HALT nach den 4 kleinen Linien — braucht es noch eine finale Aktion?
- Endschalter (A2, PC_5) noch nicht in roboter_v10 integriert — wann soll der 360°-Servo per Endschalter statt Zeitbasis gestoppt werden?

## Session-Routine
Am Ende jeder Session:
1. Neue Architektur-Entscheidungen → Abschnitt "Aktive Entscheidungen" ergänzen
2. Neue Erkenntnisse / Bugs → Auto Memory speichern
3. Offene Punkte → Abschnitt "Offene Fragen" aktualisieren
