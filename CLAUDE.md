# mbed_pm — PES Board Roboter (Nucleo F446RE)

## Aktueller Stand
_Wird am Ende jeder Session via `/sesh-end` aktualisiert._ (2026-04-23)
- **v22 erstellt** als Kopie von v21 ([src/prototype02_v22.cpp](src/prototype02_v22.cpp), [src/prototype02_v22.h](src/prototype02_v22.h)); main.cpp + test_config.h entsprechend erweitert. v21 bleibt als Backup unverändert
- **D2-Höhen auf dem Roboter validiert und nachjustiert** (2026-04-23): `SERVO_D2_BLAU_DOWN = 0.38f`, `SERVO_D2_GRUEN_DOWN = 0.22f` in v22 ([src/prototype02_v22.cpp:72-73](src/prototype02_v22.cpp#L72-L73)) — getestet, passen
- **GRÜN/GELB Hue-Grenze gefixt** ([lib/ColorSensor/ColorSensor.cpp:292-293](lib/ColorSensor/ColorSensor.cpp#L292-L293)): `HUE_YELLOW_MAX = 38.5f` (vorher 52.0) — GRÜN wurde als GELB erkannt, neue Grenze basiert auf Messung GELB max 38° / GRÜN min 39°. Getestet, funktioniert
- **Aus v21/Session 2 (2026-04-22) übernommen und immer noch aktiv:**
  - STATE_BLIND 3×Endschalter ([src/prototype02_v22.cpp:405-411](src/prototype02_v22.cpp#L405-L411))
  - Zwei Latent-Bugs aus v20 gefixt (Platzhalter-`if {` in CROSSING_STOP + SMALL_CROSSING_STOP)
  - ROT-Jiggle-Flip in ROT_GELB_PAUSE nur bei breiten Balken
  - Doppelimpuls-Jiggle (6+3+6) in CROSSING_STOP
  - 5-Click auch am letzten breiten Balken
- **Noch offen (Hauptproblem):** Jiggle muss für ALLE Farben funktionieren — evt. D1 mit-jiggeln

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
- **v21 ist aktive Hauptversion** (2026-04-22) — Kopie v20 als neuer Arbeits-Zweig, test_config.h auf `PROTOTYPE_02_V21`. v20 bleibt als Backup, wird nicht mehr editiert
- v21 (2026-04-22): `SMALL_FOLLOW_START_GUARD` Basis 706 → 781 (+1.5 s Blind-Sperre nach 4. Querbalken gegen Farb-/Balken-Trigger in der Kurve)
- v21 (2026-04-22 Session 2): STATE_BLIND setzt `m_click_target=3` → 3× Endschalter-Überfahrt bei Initial-Ausrichtung (nutzt bestehende Click-Counting-ISR)
- v21 (2026-04-22 Session 2): D2-Ablagetiefen ~4 mm flacher — `SERVO_D2_BLAU_DOWN=0.37f`, `SERVO_D2_GRUEN_DOWN=0.16f`. Faustregel: +0.01 Puls ≈ 1.3 mm
- v21 (2026-04-22 Session 2): ROT-Jiggle in ROT_GELB_PAUSE flippt nur an breiten Balken — `jiggle_rev = rev ^ (m_rot_gelb_color == 3 && !m_rot_gelb_is_small)`. An Schmallinien bleibt Standard-Richtung
- v21 (2026-04-22 Session 2): CROSSING_STOP Jiggle 2 als **Doppelimpuls** (6+3+6 Loops) statt 11-Loop Einzelimpuls — nur GRÜN/BLAU an breiten Balken. Schmallinien + ROT_GELB_PAUSE unverändert
- v21 (2026-04-22 Session 2): Finale Drehung nach letztem breitem Balken jetzt **5-Click** statt 90° — in CROSSING_STOP bedingungslos, in ROT_GELB_PAUSE via `do_5click = !m_rot_gelb_is_small`
- v21 (2026-04-22 Session 2): Zwei latente Syntax-Bugs aus v20 entfernt (Platzhalter-`if {` ohne `}`) in STATE_CROSSING_STOP + STATE_SMALL_CROSSING_STOP — wurden nie bemerkt weil v20 nie kompiliert wurde
- Claude Code Modell: `opus[1m]` in `~/.claude/settings.json` — bei Opus-Rate-Limit temporär auf `sonnet` wechseln (Opus-Sublimit ist unabhängig vom Gesamt-Limit)
- `/popupssound` toggled Sound via Flag-Datei `C:\Users\alexa\.claude\sound_disabled` — alle drei Popup-Scripts prüfen dieses Flag
- `/popups`-Befehl toggled Popups via Flag-Datei `C:\Users\alexa\.claude\popups_disabled` — kein settings.json-Edit nötig
- Oranges Popup via `PermissionRequest`-Hook → `permission_request.ps1` (erstellt Flag + fokussiert VSCode + startet Popup)
- Popup-Dismiss via `PostToolUse`-Hook → `dismiss_orange.ps1` (löscht Flag → Popup schliesst in <100ms)
- `PreToolUse` feuert VOR der Bestätigung — für Dismiss muss `PostToolUse` verwendet werden
- Grünes Popup: kein Timeout, schliesst nur bei Maus/Tastatur; Stimme 3x (sofort, +20s, +40s)
- Alle Popups: `WaitUntilDone(-1)` nach `ShowDialog()` — Stimme spricht zu Ende auch nach frühem Dismiss
- VSCode-Fokus via `AttachThreadInput` in `focus_vscode.ps1` — funktioniert aus nicht-fokussiertem Prozess
- `PROTOTYPE_02_V16` ist aktive Hauptversion (v15 Backup) — test_config.h aktuell auf TEST_SERVO_ALL (zurückstellen!)
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
- **v14 (2026-04-20): 360°-Servo Brake-Delay nach Endschalter-Hit** — `SERVO360_ALIGN_SPEED=0.52f`, `SERVO360_BRAKE_LOOPS=0` (sofort stoppen). Servo dreht mit konstanter Geschwindigkeit 0.52 und stoppt beim Endschalter-Hit sofort. Gilt universell für Initial-Ausrichtung, CROSSING_STOP (4 Querbalken), SMALL_CROSSING_STOP (4 kleine Linien). Noch nicht getestet.
- **Servo-Positionierung Präzision-Strategie (2026-04-20):** Anforderung ±4° Genauigkeit nach 45° Rotation ab Endschalter-Hit. Erste Taktik: **Timing-Kalibrierung** — messe wie lange Servo für 45° braucht, nutze diese Konstante. Fallback wenn nicht ausreichend: **Zweiter Endschalter auf PC_5** (frei, war vorher selbst Endschalter-Pin). Zweiter Schalter wird 45° nach erstem platziert → Hit löst sofortigen Stop aus.

## Projekt-Kontext
- **Kurs:** ZHAW 2. Semester, Modul PM2
- **Abgabe:** 23.05.2026 (Abend)
- **Team:** 6 Personen — 3x Elektronik & Programmierung, 3x Mechanik (CAD)

## Nächste Schritte
1. **Flash v21 und volle Runde fahren (Golden-Path-Test):** `pio run --target upload`, dann beobachten: (a) Initial-Ausrichtung stoppt beim 3. Endschalter-Hit statt beim 1., (b) D2-Ablage 4 mm flacher als vorher sichtbar, (c) an Balken 4 macht das Tablett 5-Click statt 90°, (d) Jiggle an breiten Balken: ein kurzer Puls + zwei Gegenpulse, (e) ROT vs GELB Jiggle-Richtung: an breiten Balken gegensätzlich, an Schmallinie identisch. Log: welche Farbe → welche Pulse-Richtung.

   **Reliability-Issues (aus Testrunde offen):**
   - **!!! Hauptproblem !!! Jiggle muss für ALLE Farben funktionieren** (evt. D1 mit-jiggeln)
   - ~~D2-Höhe runtersetzen~~ ✅ gefixt 2026-04-23: BLAU 0.38, GRÜN 0.22
   - ~~Hue-Wert GRÜN kalibrieren~~ ✅ gefixt 2026-04-23: `HUE_YELLOW_MAX = 38.5f`
2. **Falls Initial-Ausrichtung zu weit dreht:** `SERVO360_CLICK_EXTRA_LOOPS=7` ist Nachlauf nach 3. Hit — ggf. separaten `SERVO360_ALIGN_EXTRA_LOOPS=0` einbauen.
3. **Letzte Schmallinie (Bug 2):** Nach ROT/GELB an Schmallinie #3 geht Roboter fälschlich in FINAL_HALT — Exit-Logik in STATE_ROT_GELB_PAUSE prüfen (`m_small_crossings_left==1`-Zweig).
4. **Doppelimpuls-Timing tunen:** 6+3+6 Loops sind Erstwert, ggf. Pause zwischen den zwei Gegenpulsen verlängern wenn Päckchen nicht sauber rutscht.

## Offene Fragen
- **Kalibrierungs-Drift:** Code nutzt `calibratePulseMinMax(0.050, 0.1050)`, echte Kalibrierung ist aber `(0.0303, 0.1223)` — soll umgestellt werden, oder reicht Click-Counting?
- **Coast-Phase bei Initial-Ausrichtung:** `SERVO360_CLICK_EXTRA_LOOPS=7` (140 ms Nachlauf) — wirkt sich das bei 3×Hit schädlich aus? Ggf. separaten `SERVO360_ALIGN_EXTRA_LOOPS=0` einbauen.
- **D2-Höhe +0.03 Puls** entspricht nur geschätzt ~4 mm — Hebelarm-Annahme ~10 cm. Nach erstem Flash physikalisch verifizieren.
- **Letzter breiter Balken mit 5-Click** — reicht `SERVO360_ALIGN_LOOPS` als Timeout, damit alle 5 Clicks erreicht werden? Falls 5. Hit nach Timeout liegt → ALIGN_LOOPS erhöhen.
- **Letzte Schmallinie (4.):** Exit-Bedingung in ROT_GELB_PAUSE bei `m_rot_gelb_is_small=true` und `m_small_crossings_left==1` — geht fälschlicherweise in FINAL_HALT?
- **Angle Clamp 0.15 rad/Loop** — noch nicht validiert.

## Session-Routine
Am Ende jeder Session:
1. Neue Architektur-Entscheidungen → Abschnitt "Aktive Entscheidungen" ergänzen
2. Neue Erkenntnisse / Bugs → Auto Memory speichern
3. Offene Punkte → Abschnitt "Offene Fragen" aktualisieren
