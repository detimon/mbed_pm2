# CargoSweep — mbed_pm2

Autonomer Roboter auf Basis des **Nucleo F446RE** mit PES Board.  
Der Roboter folgt einer Linie, erkennt farbige Karten und betätigt Servos zum Sortieren von Paketen.

---

## Inhaltsverzeichnis

- [Hardware](#hardware)
- [Projektstruktur](#projektstruktur)
- [Aktiver Test auswählen](#aktiver-test-auswählen)
- [Ablauf-Übersicht](#ablauf-übersicht)
- [State Machine](#state-machine)
- [Hardware-Pinbelegung](#hardware-pinbelegung)
- [Tuning-Parameter](#tuning-parameter)
- [Farb-Logik](#farb-logik)
- [Servo-Logik](#servo-logik)
- [Build & Flash](#build--flash)
- [Serial Debug-Ausgabe](#serial-debug-ausgabe)

---

## Hardware

| Komponente | Details |
|---|---|
| Mikrocontroller | STM32 Nucleo F446RE |
| Board | PES Board |
| DC-Motoren | 2× DC-Motor, Getriebe 100:1, KN = 140/12 |
| Liniensensor | 8-Kanal IR LineFollower (Schwellwert: 0.40) |
| Farbsensor | TCS3200/TCS230 (6-Pin) |
| Servo 360° | D3 (PB_12) — Tablett-Rotation |
| Servo 180° D1 | D6 (PC_8) — Arm ausfahren |
| Servo 180° D2 | D0 (PB_2) — Arm absenken |
| NeoPixel | PC_5 (A2) — WS2812B Farb-Debug-LED |
| Endschalter | A0 (PC_2), PullUp, LOW-aktiv |
| User Button | PC_13 (BUTTON1) — Start/Stop |
| User LED | PA_5 (LED1) — Blink-Statusanzeige |

---

## Projektstruktur

```
mbed_pm2/
├── src/
│   ├── main.cpp                  # Hauptprogramm — Loop, Button-ISR, Test-Dispatch
│   ├── test_config.h             # HIER aktiven Test/Prototyp auswählen
│   ├── io_spec.md                # Pin-Belegung aller Tests
│   ├── prototype02_v20.cpp/.h    # Neueste Roboter-Version (aktuell aktiv)
│   ├── prototype02_v9–v19.cpp    # Ältere Versionen (Archiv)
│   ├── prototype01_v1–v8.cpp     # Erste Prototyp-Reihe (Archiv)
│   └── test_files/               # Einzeltests (Servo, Farbsensor, LineFollower, …)
├── lib/                          # Bibliotheken (DCMotor, LineFollower, ColorSensor, …)
├── include/                      # Shared Headers
├── platformio.ini                # PlatformIO Build-Konfiguration
└── mbed_app.json                 # Mbed-OS Konfiguration
```

---

## Aktiver Test auswählen

In `src/test_config.h` genau **ein** `#define` einkommentieren:

```cpp
// --- Einzeltests ---
// #define TEST_SERVO
// #define TEST_COLOR_SENSOR
// #define TEST_COLOR_SENSOR_CALIB
// #define TEST_LINE_FOLLOWER
// #define TEST_DC_MOTOR
// #define TEST_IR
// #define TEST_NEOPIXEL
// #define TEST_ENDSCHALTER

// --- Prototypen (Roboter-Vollprogramm) ---
// #define PROTOTYPE_02_V19
#define PROTOTYPE_02_V20          // ← aktuell aktiv
```

Danach: **Rebuild & Flash**.

---

## Ablauf-Übersicht

```
  [Start / Button drücken]
          │
          ▼
  ┌──────────────┐
  │  STATE_BLIND  │  Geradeaus fahren bis alle 8 Sensoren die Startlinie sehen
  └──────┬───────┘
         │ alle 8 Sensoren aktiv
         ▼
  ┌─────────────────┐
  │  STATE_STRAIGHT  │  1.4 s geradeaus über die Linie hinaus
  └──────┬──────────┘
         │ Timer abgelaufen
         ▼
  ┌──────────────────┐
  │  STATE_TURN_SPOT  │  Auf der Stelle drehen bis Mittelsensoren Linie sehen
  └──────┬───────────┘
         │ Sensoren 3 oder 4 aktiv
         ▼
  ┌──────────────┐
  │  STATE_FOLLOW │  Linie folgen bis Querbalken (alle 8 Sensoren aktiv)
  └──────┬───────┘
         │ Querbalken erkannt
         ▼
  ┌────────────────┐
  │ STATE_FOLLOW_1S │  Noch 1 s weiterfolgen nach Balkenerkennung
  └──────┬─────────┘
         │
         ▼
  ┌─────────────┐
  │ STATE_BRAKE  │  Sanft abbremsen (0.24 s Rampe)
  └──────┬──────┘
         │
         ▼
  ┌─────────────┐
  │ STATE_PAUSE  │  0.4 s Stillstand
  └──────┬──────┘
         │
         ▼
  ┌───────────────┐
  │ STATE_BACKWARD │  4.44 s rückwärts fahren (mit Accel/Decel-Rampe)
  └──────┬────────┘
         │
         ▼
  ┌────────────────────────┐
  │ STATE_REAL_START_PAUSE  │  0.4 s Pause vor Hauptprogramm
  └──────┬─────────────────┘
         │
         ▼
  ┌──────────────────────┐
  │ STATE_REAL_APPROACH   │  Geradeaus zur ersten Karte / erstem Balken
  └──────┬───────────────┘
         │ Querbalken erkannt
         ▼
  ╔═══════════════════════════════════════╗
  ║    HAUPTPROGRAMM (4× Querbalken)      ║
  ║                                       ║
  ║  STATE_REAL_FOLLOW                    ║
  ║    │ Linie folgen, Farbe erkennen,    ║
  ║    │ Geschwindigkeit bei Farbe        ║
  ║    │ auf 20% reduzieren               ║
  ║    ↓ Querbalken erkannt               ║
  ║  STATE_CROSSING_STOP                  ║
  ║    │ Stopp → Farbe lesen (0.5 s)     ║
  ║    │ → Servo-Sequenz (je nach Farbe) ║
  ║    ↓                                  ║
  ║  ROT/GELB → STATE_ROT_GELB_DRIVE     ║
  ║    │ 120 mm Linienfolge               ║
  ║    ↓                                  ║
  ║  STATE_ROT_GELB_PAUSE (Arm-Sequenz)  ║
  ╚═════════╤═════════════════════════════╝
            │ 4. Querbalken abgeschlossen
            ▼
  ╔═════════════════════════════════════╗
  ║     KLEINE LINIEN (4× wiederholt)   ║
  ║                                     ║
  ║  STATE_SMALL_FOLLOW                 ║
  ║    ↓ kleine Linie erkannt           ║
  ║  STATE_SMALL_CROSSING_STOP          ║
  ║    │ Stopp → Farbe → Servo          ║
  ║    ↓ ROT/GELB → ROT_GELB_DRIVE     ║
  ╚═════════╤═══════════════════════════╝
            │ 4. kleine Linie abgeschlossen
            ▼
  ┌──────────────────┐
  │  STATE_FINAL_HALT │  Dauerhafter Stopp — Programm beendet
  └──────────────────┘
```

---

## State Machine

| State | Enum | Beschreibung |
|---|---|---|
| `STATE_BLIND` | 1 | Geradeaus bis Startlinie erkannt (alle 8 Sensoren) |
| `STATE_STRAIGHT` | 2 | 1.4 s geradeaus über Linie fahren |
| `STATE_TURN_SPOT` | 3 | Auf der Stelle drehen bis Linie gefunden |
| `STATE_FOLLOW` | 4 | Linie folgen bis Querbalken |
| `STATE_FOLLOW_1S` | 5 | 1 s weiterfolgen nach Balken |
| `STATE_BRAKE` | 6 | Sanfte Bremsrampe (0.24 s) |
| `STATE_PAUSE` | 7 | 0.4 s Stillstand |
| `STATE_BACKWARD` | 8 | 4.44 s Rückwärtsfahrt mit Accel/Decel-Rampen |
| `STATE_REAL_START_PAUSE` | 14 | 0.4 s Pause vor Hauptprogramm |
| `STATE_REAL_APPROACH` | 15 | Geradeaus zur ersten Station (langsam, mit Rampe) |
| `STATE_REAL_FOLLOW` | 9 | Hauptlinienfolger — stoppt an jedem der 4 Querbalken |
| `STATE_CROSSING_STOP` | 10 | Stopp + Farbe lesen + Servo-Sequenz bei Querbalken |
| `STATE_SMALL_FOLLOW` | 12 | Linienfolger zwischen den 4 kleinen Linien |
| `STATE_SMALL_CROSSING_STOP` | 13 | Stopp + Farbe + Servo bei kleiner Linie |
| `STATE_ROT_GELB_DRIVE` | 16 | 120 mm Linienfolge nach ROT/GELB-Erkennung |
| `STATE_ROT_GELB_PAUSE` | 17 | Arm-Sequenz nach der 120 mm Fahrt |
| `STATE_FINAL_HALT` | 11 | Dauerhafter Stopp nach 4. kleiner Linie |

---

## Hardware-Pinbelegung

### Shared (alle Modi)

| Signal | Mbed Pin | Board Pin | Richtung | Beschreibung |
|---|---|---|---|---|
| User Button | BUTTON1 | PC_13 | Input | Blauer Taster — togglet Start/Stop |
| User LED | LED1 | PA_5 | Output | Grüne LED — blinkt jede Periode |

### Motoren & Sensoren (Prototype v20)

| Komponente | Pin(s) | Beschreibung |
|---|---|---|
| Motor M1 PWM | PB_PWM_M1 | Linker DC-Motor |
| Motor M2 PWM | PB_PWM_M2 | Rechter DC-Motor |
| Motor Enable | PB_ENABLE_DCMOTORS | Motorstufe freigeben (active high) |
| LineFollower | PB_9, PB_8 | 8-Kanal IR-Sensor |
| ColorSensor | PB_3, PC_3, PA_4, PB_0, PC_1, PC_0 | TCS3200 Farbsensor |
| Servo 360° | PB_12 (D3) | Tablett-Rotation |
| Servo D1 180° | PC_8 (D6) | Arm ausfahren |
| Servo D2 180° | PB_2 (D0) | Arm absenken |
| NeoPixel | PC_5 (A2) | WS2812B Debug-LED |
| Endschalter | PC_2 (A0) | PullUp, LOW-aktiv, Interrupt auf fallende Flanke |

---

## Tuning-Parameter

Alle Parameter stehen am Anfang von `prototype02_v20.cpp` als `static const`:

| Parameter | Wert | Beschreibung |
|---|---|---|
| `KP` | 2.8 | P-Gain Linienfolger (normal) |
| `KP_NL` | 4.55 | Nichtlinearer P-Gain (normal) |
| `KP_FOLLOW` | 1.1 | P-Gain beim Einfahren in Linienfolge (niedrig → kein Durchdrehen) |
| `MAX_SPEED` | 1.6 | Maximale Radgeschwindigkeit [m/s] |
| `SENSOR_THRESHOLD` | 0.40 | Schwellwert IR-Sensor (0–1) |
| `GEAR_RATIO` | 100.0 | Motorgetriebe-Übersetzung |
| `D_WHEEL` | 0.0291 m | Raddurchmesser |
| `B_WHEEL` | 0.1493 m | Radabstand (Spurweite) |
| `BAR_DIST` | 0.182 m | Abstand Sensor zu Radachse |
| `STRAIGHT_LOOPS` | 70 | 1.4 s Geradeausfahrt nach Startlinie (@ 50 Hz) |
| `BACKWARD_LOOPS` | 222 | 4.44 s Rückwärtsfahrt |
| `BRAKE_LOOPS` | 12 | 0.24 s Bremsrampe |
| `COLOR_STABLE_CNT` | 3 | Konsekutive gleiche Lesungen zum Auslösen (60 ms) |
| `SLOW_FACTOR` | 0.2 | Zielgeschwindigkeit bei Farbannäherung (20% von aktuell) |

> **SPEED_SCALE-Hinweis:** Wenn `MAX_SPEED` erhöht wird, skalieren alle zeitbasierten
> Guards (markiert mit `// [SPEED_SCALE]`) automatisch mit, sodass die zurückgelegte
> Strecke konstant bleibt. Servo-Zeiten und Standstill-Pausen **nicht** skalieren.

---

## Farb-Logik

Der TCS3200-Farbsensor gibt einen Integer-Farbcode zurück:

| Code | Farbe | Aktion bei Balken/Linie |
|---|---|---|
| 0 | UNKNOWN | Keine Aktion (Fallback auf Fahrt-Farbe) |
| 1 | BLACK | Keine Aktion |
| 2 | WHITE | Keine Aktion |
| 3 | ROT | 120 mm Linienfolge danach (`STATE_ROT_GELB_DRIVE`) |
| 4 | GELB | 120 mm Linienfolge danach (`STATE_ROT_GELB_DRIVE`) |
| 5 | GRÜN | Servo D1 ausfahren + D2 auf `SERVO_D2_GRUEN_DOWN` (0.13) |
| 7 | BLAU | Servo D1 ausfahren + D2 auf `SERVO_D2_BLAU_DOWN` (0.34) |

**Erkennungs-Mechanismus (während der Fahrt):**
1. Farbsensor liest jeden Loop (50 Hz)
2. Gleiche Aktionsfarbe muss `COLOR_STABLE_CNT` (3) mal in Folge gelesen werden
3. Dann beginnt Geschwindigkeitsreduktion auf 20% (`SLOW_FACTOR`) über 0.26 s (`SLOWDOWN_LOOPS`)
4. Bei Balkenerkennung: Stopp → Farbe im Stillstand nochmals über `COLOR_READ_PHASE` (0.5 s) bestätigen

**NeoPixel-LED** zeigt aktuelle Farbe gedimmt (60/255) an:
- ROT → rot, GELB → orange, GRÜN → grün, BLAU → blau

**User-LED Blinkcode** (2-Sekunden-Periode @ 50 Hz):
- Kein Signal → schnelles Dauerblinken (alive)
- ROT → 1 Blink, GELB → 2, GRÜN → 3, BLAU → 4 Blinks pro 2 s

---

## Servo-Logik

### 360°-Servo (Tablett-Rotation)

```
Kick-Phase (SERVO360_KICK_LOOPS = 8, ~150 ms):
  Servo startet mit SERVO360_KICK_SPEED (0.55) → überwindet Haftreibung

Alignment-Phase (bis zu SERVO360_ALIGN_LOOPS = 250, max 5 s):
  Servo dreht mit SERVO360_ALIGN_SPEED (0.55) bis Endschalter auslöst

Bei Endschalter-Hit:
  → ISR setzt s_endstop_hit = true (fallende Flanke)
  → Servo stoppt nach SERVO360_BRAKE_LOOPS (1 Loop Verzögerung)
  → Servo disabled

5-Click-Modus (Balken 1/2/3):
  Tablett dreht exakt 5 Positionen weiter für das nächste Paket
  Nach letztem Click: SERVO360_CLICK_EXTRA_LOOPS (7) Weiterdrehung, dann Stopp
  Danach: Korrektur zurück zu 45° mit SERVO360_CORRECT_SPEED (0.42)
```

### 180°-Servo D1 (Arm ausfahren)

| Position | Wert | Bedeutung |
|---|---|---|
| Eingefahren | `0.0` | Ruheposition |
| Ausgefahren | `0.95` | Paket greifen |
| Kalibrierung | 0.050–0.1050 | Pulsbreitenbereich |

### 180°-Servo D2 (Arm absenken)

| Position | Wert | Bedeutung |
|---|---|---|
| Oben | `1.0` | Ruheposition |
| GRÜN-Down | `0.13` | Ablagetiefe für GRÜN |
| BLAU-Down | `0.34` | Ablagetiefe für BLAU |
| Partial-Down | `0.85` | Zwischenposition vor 5-Click-Korrektur |
| Kalibrierung | 0.0200–0.1310 | Pulsbreitenbereich |

---

## Build & Flash

### PlatformIO (empfohlen)

```bash
# Bauen
pio run

# Flashen
pio run --target upload

# Serial Monitor (115200 Baud)
pio device monitor
```

### Mbed CLI

```bash
mbed compile -m NUCLEO_F446RE -t GCC_ARM
```

---

## Haupt-Loop (`main.cpp`)

Der `main()`-Loop läuft mit **50 Hz** (20 ms Periode) und dispatcht per Präprozessor-Makros
an den aktiven Test/Prototyp:

```
main()
 │
 ├── user_button.fall(ISR) ← Button-ISR registrieren
 ├── TEST_INIT(loops_per_second) ← Hardware init
 │
 └── while(true)  [20 ms = 50 Hz]
      │
      ├── if (do_execute_main_task) → TEST_TASK(led)
      ├── else if (do_reset_all_once) → TEST_RESET(led)  [einmalig]
      │
      ├── user_led togglen
      ├── TEST_PRINT() ← Serial-Debug-Ausgabe
      └── thread_sleep_for(rest) ← Restzeit der 20 ms Periode abwarten
```

> **Button-ISR:** `toggle_do_execute_main_fcn()` togglet `do_execute_main_task`.
> Bei Aktivierung wird `do_reset_all_once = true` gesetzt → sauberer Reset vor dem Start.

---

## Serial Debug-Ausgabe

Beispielausgabe im Serial Monitor:

```
State: REAL_FOLLOW  | wide=3 small=4 | a=0.012 | M1=+1.58 M2=+1.61 | all8=0 s2345=0
Color: GRUEN     hue= 92.3 delta=0.41 r=0.18 g=0.59 b=0.21 | action=UNKNOWN
```

| Feld | Bedeutung |
|---|---|
| `State` | Aktueller Zustand der State Machine |
| `wide` | Verbleibende Querbalken |
| `small` | Verbleibende kleine Linien |
| `a` | Lenkwinkel des LineFollowers [rad] |
| `M1` / `M2` | Ist-Geschwindigkeit der Motoren [m/s] |
| `all8` | 1 wenn alle 8 IR-Sensoren aktiv (= Querbalken) |
| `s2345` | 1 wenn kleine Linie erkannt (Sensoren 2–4 oder 3–5) |
| `hue` | Farbton [0–360°] zur Sensor-Kalibrierung |
| `delta` | RGB-Spreizung — Sättigungsindikator |
| `action` | Farbe, die an der aktuellen Station ausgeführt wird |
