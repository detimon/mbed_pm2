# Software Export — CargoSweep Team 5
**Projekt:** ZHAW PM2, 2. Semester 2026 — Nucleo F446RE + PES Board  
**Hauptprogramm:** `src/cargosweep_team5_20260512.cpp`  
**Exportiert:** 2026-05-12

---

## 1. INBETRIEBNAHMEPROTOKOLLE

### test_motor_drive.cpp
| Feld | Inhalt |
|---|---|
| Testname | test_motor_drive.cpp |
| Komponente + Pin(s) | M10 (links): PWM PB_13, ENC PA_6/PC_7; M11 (rechts): PWM PA_9, ENC PB_6/PB_7; EN PB_15 |
| Testziel | Beide DC-Motoren durchlaufen 5-Phasen-Testsequenz. |
| Erwartetes Verhalten | Phasenwechsel alle 2 s; Istwert ≤ 1.5 RPS; Soll/Ist-Anzeige auf Serial Monitor |
| Ergebnis | Offen |
| Besonderheiten | GEAR_RATIO=78.125 (Testdatei), KN=180; TIM1->BDTR MOE-Bit pflicht |

### test_linefollower.cpp
| Feld | Inhalt |
|---|---|
| Testname | test_linefollower.cpp |
| Komponente + Pin(s) | LineFollower SEN-13582; I2C1: SDA=PB_9, SCL=PB_8 |
| Testziel | Linienerkennung bei Standard-Geschwindigkeit. |
| Erwartetes Verhalten | Roboter folgt schwarzer Linie und korrigiert leicht in Kurven |
| Ergebnis | Offen |
| Besonderheiten | KP=3.0, KP_NL=10.0, MAX_SPEED=0.5 RPS; VEL_SIGN=-1.0 |

### test_linefollower_fast.cpp
| Feld | Inhalt |
|---|---|
| Testname | test_linefollower_fast.cpp |
| Komponente + Pin(s) | LineFollower SEN-13582; I2C1: SDA=PB_9, SCL=PB_8 |
| Testziel | Linienerkennung bei voller Fahrgeschwindigkeit (Wettkampf-Setting). |
| Erwartetes Verhalten | Roboter folgt Linie ohne Ausbruch; STABIL — Parameter NICHT veraendern |
| Ergebnis | Bestanden (Commit 183603b) |
| Besonderheiten | KP=2.8, KP_NL=4.55, MAX_SPEED=1.0 RPS; VEL_SIGN=-1.0; GEAR_RATIO=100 |

### test_linefollower_slow.cpp
| Feld | Inhalt |
|---|---|
| Testname | test_linefollower_slow.cpp |
| Komponente + Pin(s) | LineFollower SEN-13582; I2C1: SDA=PB_9, SCL=PB_8 |
| Testziel | Linienerkennung mit reduzierter Geschwindigkeit fuer Justage. |
| Erwartetes Verhalten | Roboter folgt Linie langsam; ideal zum Pruefen der Sensor-Klassifikation |
| Ergebnis | Offen |
| Besonderheiten | KP=3.0, KP_NL=10.0, MAX_SPEED=0.5 RPS; VEL_SIGN=-1.0 |

### test_linefollower_bwd.cpp
| Feld | Inhalt |
|---|---|
| Testname | test_linefollower_bwd.cpp |
| Komponente + Pin(s) | LineFollower SEN-13582; I2C1: SDA=PB_9, SCL=PB_8 |
| Testziel | Linienfolge bei Rueckwaertsfahrt. |
| Erwartetes Verhalten | Roboter folgt Linie kontrolliert rueckwaerts ohne abrupten Lenkausschlag |
| Ergebnis | Offen |
| Besonderheiten | KP=2.8, KP_NL=4.55, MAX_SPEED=1.0 RPS; ANGLE_KP=0.3; VEL_SIGN=-1.0 |

### test_servo_180.cpp
| Feld | Inhalt |
|---|---|
| Testname | test_servo_180.cpp |
| Komponente + Pin(s) | 180° Servo A (M20/D1); PB_D1 = PC_8 |
| Testziel | Servo faehrt langsam von 0 bis 180 Grad und stoppt am oberen Anschlag. |
| Erwartetes Verhalten | Pulse-Wert steigt um 0.005 pro Sekunde; keine Stockungen, kein Brummen |
| Ergebnis | Offen |
| Besonderheiten | calibratePulseMinMax(0.050, 0.1050); setMaxAcceleration(0.3) |

### test_servo_180_angle.cpp
| Feld | Inhalt |
|---|---|
| Testname | test_servo_180_angle.cpp |
| Komponente + Pin(s) | 360°-Servo (M22); PB_D3 = PB_12 |
| Testziel | Winkelposition gegen mechanische Markierung verifizieren — 4x90°=360° Kalibrierung. |
| Erwartetes Verhalten | Servo faehrt 4 x 90° und landet exakt wieder auf Startposition |
| Ergebnis | Offen |
| Besonderheiten | PULSE_360_MIN=0.0303, PULSE_360_MAX=0.1223; TIME_FOR_90_MS=140; SERVO_CW=0.25, SERVO_STOP=0.50 |

### test_servo_combined.cpp
| Feld | Inhalt |
|---|---|
| Testname | test_servo_combined.cpp |
| Komponente + Pin(s) | M20 (PB_D1=PC_8), M21 (PB_D2=PC_6), M22 (PB_D3=PB_12) |
| Testziel | Alle drei Servos zusammen ansteuern und Cross-Talk ausschliessen. |
| Erwartetes Verhalten | Jeder Servo erreicht Zielposition unabhaengig, keine PWM-Interferenz |
| Ergebnis | Offen |
| Besonderheiten | A: MIN=0.0303/MAX=0.1204; B: MIN=0.0314/MAX=0.1232; 360°: MIN=0.0303/MAX=0.1223/Stopp=0.0763 |

### test_arm_logic.cpp
| Feld | Inhalt |
|---|---|
| Testname | test_arm_logic.cpp |
| Komponente + Pin(s) | D1 (PC_8), D2 (PB_2), Tray 360° (PB_12, Feedback PC_2) |
| Testziel | Logische Greif- und Ablage-Sequenz des Arms durchspielen. |
| Erwartetes Verhalten | Arm faehrt Heim → greifen → ablegen → Heim ohne Kollision |
| Ergebnis | Offen |
| Besonderheiten | SF360_OFFSET=70°; alterniert zwischen BLAU-Tiefe (0.28) und GRUEN-Tiefe (0.07) |

### test_arm_sequence.cpp
| Feld | Inhalt |
|---|---|
| Testname | test_arm_sequence.cpp |
| Komponente + Pin(s) | M20 (PC_8), M21 (PB_2), M22 Parallax 360° (PB_12 PWM, PC_2 Feedback) |
| Testziel | Vollstaendige Pick-and-Place-Sequenz inkl. Karussel-Drehung verifizieren. |
| Erwartetes Verhalten | 8 Schritte in unter 30 s sequentiell abgearbeitet; Button 2 stoppt jederzeit |
| Ergebnis | Offen |
| Besonderheiten | SF360_OFFSET=110°; kein setMaxAcceleration; Single-File ohne .h |

### test_color_detect.cpp
| Feld | Inhalt |
|---|---|
| Testname | test_color_detect.cpp |
| Komponente + Pin(s) | TCS3200 Farbsensor; OUT=PB_3, LED=PC_3, S0=PA_4, S1=PB_0, S2=PC_1, S3=PC_0 |
| Testziel | Farbsensor klassifiziert ROT/GELB/GRUEN/BLAU korrekt. |
| Erwartetes Verhalten | Bei Vorhalten einer Farbkarte erscheint der korrekte Farbname auf Serial; Reaktion sichtbar |
| Ergebnis | Offen |
| Besonderheiten | ANSI-Farbausgabe; zeigt Raw Hz, Avg Hz, Calibr, Norm, Hue; PRINT_EVERY_N=25 |

### test_ir_sensor.cpp
| Feld | Inhalt |
|---|---|
| Testname | test_ir_sensor.cpp |
| Komponente + Pin(s) | IR-Sensor analog; PB_1 (A3) |
| Testziel | Spannungsausgabe des IR-Sensors als gefilterter Mittelwert ueberpruefen. |
| Erwartetes Verhalten | Wert in mV laeuft monoton mit Annaeherung eines Hindernisses (z.B. Hand) |
| Ergebnis | Offen |
| Besonderheiten | read() liefert gefilterten Durchschnitt in mV (unkalibriert) |

### test_neopixel_basic.cpp
| Feld | Inhalt |
|---|---|
| Testname | test_neopixel_basic.cpp |
| Komponente + Pin(s) | WS2812B NeoPixel; DIN = PB_2 (D0) |
| Testziel | NeoPixel-Treiber zyklisch RGB-Farben anzeigen lassen. |
| Erwartetes Verhalten | LED leuchtet abwechselnd rot, gelb, gruen, blau, aus im Sekundentakt |
| Ergebnis | Offen |
| Besonderheiten | Farbzyklus ROT→GELB→GRUEN→BLAU→AUS, je 1 s |

### test_neopixel_color.cpp
| Feld | Inhalt |
|---|---|
| Testname | test_neopixel_color.cpp |
| Komponente + Pin(s) | NeoPixel PC_5; Farbsensor PB_3/PC_3/PA_4/PB_0/PC_1/PC_0 |
| Testziel | Farbsensor-Klassifikation wird live auf NeoPixel gespiegelt. |
| Erwartetes Verhalten | Erkannte Farbe leuchtet sofort auf NeoPixel; Reaktion < 100 ms |
| Ergebnis | Offen |
| Besonderheiten | Dimmed-Ausgabe (r/g/b maximal 60); nur die 4 Aktionsfarben treiben NeoPixel |

### test_system_all.cpp
| Feld | Inhalt |
|---|---|
| Testname | test_system_all.cpp |
| Komponente + Pin(s) | Alle Module: Motoren, LineFollower, 3x Servo, ColorSensor |
| Testziel | Vollstaendige LineXpress-Mission durchspielen (Integrationstest). |
| Erwartetes Verhalten | Roboter loest komplette Aufgabe selbstaendig: Start - Linie - Stops - Ablage - Ende |
| Ergebnis | Offen |
| Besonderheiten | KP=4.0, KP_NL=15.0, MAX_SPEED=3.5 RPS; VEL_SIGN=-1.0; Voraussetzung: alle Einzeltests bestanden, Akku >50% |

---

## 2. KALIBRIERUNGSWERTE

### Servo 180° D1 (horizontal, PC_8)
| Parameter | Wert |
|---|---|
| pulse_min | 0.0500 |
| pulse_max | 0.1050 |
| setMaxAcceleration | 0.3f (Hauptprogramm), 0.3f (Testsuite) |
| 0.0 = | eingefahren (D1_RETRACTED = 0.1f im Hauptprogramm) |
| 1.0 = | vollständig ausgefahren |

### Servo 180° D2 (vertikal, PB_2)
| Parameter | Wert |
|---|---|
| pulse_min | 0.0200 |
| pulse_max | 0.1310 |
| setMaxAcceleration | D2_SMALL_DESCENT_ACCEL = 3.5f (Absenken), D2_RAISE_ACCEL = 3.0f (Hochfahren) |
| 1.0 = | oben (D2_UP, Transportposition) |
| 0.25 = | D2_DOWN_BLAU (breite Balken BLAU/GELB) |
| 0.10 = | D2_DOWN_GRUEN (breite Balken GRUEN/ROT) |
| 0.27 = | D2_DOWN_BLAU_SMALL (Schmallinien BLAU/GELB) |
| 0.12 = | D2_DOWN_GRUEN_SMALL (Schmallinien GRUEN/ROT) |

### D1 Ausfahrwege pro Farbe
| Farbe | Puls-Wert |
|---|---|
| ROT | D1_EXTENDED_ROT = 0.87f |
| GELB | D1_EXTENDED_GELB = 0.87f |
| GRUEN | D1_EXTENDED_GRUEN = 0.87f |
| BLAU | D1_EXTENDED_BLAU = 0.87f |
| Fallback | D1_EXTENDED = 0.82f |

### Servo 360° Tray (Parallax Feedback, PB_12 PWM / PC_2 Feedback)
| Parameter | Wert |
|---|---|
| Klasse | ServoFeedback360 |
| SF360_KP | 0.005f |
| SF360_TOL_DEG | 2.1° |
| SF360_MIN_SPEED | 0.18f |
| SF360_OFFSET | 110.0° (mechanischer Nullpunkt) |
| PULSE_CALIB_MIN | 0.064f (1280 µs = volle CW) |
| PULSE_CALIB_MAX | 0.086f (1720 µs = volle CCW) |
| Stopp-Puls | 0.5 → 1500 µs |
| DUTY_MIN (Feedback) | 0.029 = 0° |
| DUTY_MAX (Feedback) | 0.971 = 359° |

### Tray-Zielwinkel pro Farbe
| Farbe | Zielwinkel | Carry-Vorwinkel (−30°) |
|---|---|---|
| ROT | 0.0° | 330.0° |
| GRUEN | 90.0° | 60.0° |
| BLAU | 180.0° | 150.0° |
| GELB | 270.0° | 240.0° |

### Servo 360° (Kalibrierung aus test_servo_combined.cpp / test_servo_180_angle.cpp)
| Parameter | Wert |
|---|---|
| PULSE_360_MIN | 0.0303f |
| PULSE_360_MAX | 0.1223f |
| Stopp (Mitte) | 0.0763f = (MIN+MAX)/2 |

### Farbsensor TCS3200 — Kalibrierungsreferenzen (Hz)
| Kanal | Schwarz-Referenz | Weiss-Referenz |
|---|---|---|
| ROT | 333.22 | 598.44 |
| GRUEN | 345.66 | 643.92 |
| BLAU | 539.37 | 993.05 |
| CLEAR (W) | 1297.02 | 2298.85 |

### Farbsensor — Hue-Grenzen (nach Kalibrierung 2026-04-29)
| Farbe | Code | Hue MIN | Hue MAX | Anmerkung |
|---|---|---|---|---|
| ROT | 3 | 0.0° | 15.0° | Kein Wrap noetig |
| GELB | 4 | 25.0° | 50.0° | |
| GRUEN | 5 | 65.0° | 125.0° | |
| BLAU | 7 | 210.0° | 255.0° | |

### Farbsensor — Schwellwerte
| Parameter | Wert | Bedeutung |
|---|---|---|
| C0_BLACK_MAX | 80.0 Hz | Unterhalb = BLACK |
| C0_WHITE_MIN | 650.0 Hz | Oberhalb + neutral = WHITE |
| SATP_GRAY_MAX | 0.03 | Sättigung unterhalb = neutral |
| NORM_DELTA_MIN | 0.03 | Zu stark verdünnt = UNKNOWN |
| STABLE_COUNT | 1 | Hysterese-Fenster (intern) |

### Motor DC
| Parameter | Wert |
|---|---|
| GEAR_RATIO | 100.0 |
| KN | 140.0 / 12.0 ≈ 11.667 rpm/V |
| VOLTAGE_MAX | 12.0 V |
| VEL_SIGN | 1.0f (nach physischem Motortausch) |
| D_WHEEL | 0.0291 m |
| B_WHEEL | 0.1493 m |
| BAR_DIST | 0.182 m |

---

## 3. STATE MACHINE

### State-Tabelle (cargosweep_team5_20260512.cpp)
| State-Name | Enum-Wert | Beschreibung |
|---|---|---|
| STATE_ALIGN_START | 1 | Blindfahrt vorwaerts bis Startlinie (≥7 Sensoren aktiv) |
| STATE_ALIGN_STRAIGHT | 2 | 1.4 s geradeaus von Startlinie wegfahren |
| STATE_ALIGN_TURN | 3 | Auf der Stelle drehen (M1 rueckwaerts, M2 vorwaerts) bis Mittelsensor aktiv |
| STATE_ALIGN_FOLLOW | 4 | Linie folgen mit START_GUARD + FOLLOW_1S_LOOPS Countdown |
| STATE_ALIGN_BRAKE | 5 | Bremsrampe (BRAKE_LOOPS) + Standstill-Pause (PAUSE_LOOPS) |
| STATE_ALIGN_REVERSE | 6 | Trapez-Rueckwaertsfahrt zur Startposition (BACKWARD_LOOPS) |
| STATE_START_PAUSE | 14 | 0.4 s Pause vor erstem Querbalken; liest Fallback-Farbe |
| STATE_APPROACH | 15 | Langsame Annaeherung (0.5 RPS) an ersten breiten Balken |
| STATE_REAL_FOLLOW | 9 | Linienfolger breite Balken: MAX_SPEED + Farberkennung + Bremsrampe |
| STATE_SMALL_FOLLOW | 12 | Linienfolger schmale Querlinien: identisch zu REAL_FOLLOW |
| STATE_TUNNEL_FOLLOW | 24 | 3-Phasen-Tunnelfahrt (A: Linie 2 s, B: Gerade 1.5 s, C: Linie) |
| STATE_STOP_AND_READ | 10 | Stillstand + Farblesen + Dispatch zu Farb-State |
| STATE_RED | 20 | Ablage ROT: Vorwärtsfahrt 1.02 s + runDeliverPhase(); Tray 0° |
| STATE_GREEN | 21 | Ablage GRUEN: kein Fahren + runDeliverPhase(); Tray 90° |
| STATE_BLUE | 22 | Ablage BLAU: kein Fahren + runDeliverPhase(); Tray 180° |
| STATE_YELLOW | 23 | Ablage GELB: Vorwärtsfahrt 1.02 s + runDeliverPhase(); Tray 270° |
| STATE_FINISH | 11 | Endposition: Motoren + Tray gestoppt, NeoPixel laeuft weiter |

### State-Uebergangstabelle
| Von | Nach | Bedingung |
|---|---|---|
| STATE_ALIGN_START | STATE_ALIGN_STRAIGHT | all_sensors_active() ≥7 aktiv AND startup_delay==0 |
| STATE_ALIGN_STRAIGHT | STATE_ALIGN_TURN | m_straight_ctr (70 Loops) abgelaufen |
| STATE_ALIGN_TURN | STATE_ALIGN_FOLLOW | center_sensors_active() == true |
| STATE_ALIGN_FOLLOW | STATE_ALIGN_BRAKE | m_follow_ctr (FOLLOW_1S_LOOPS) auf 0 |
| STATE_ALIGN_BRAKE | STATE_ALIGN_REVERSE | m_pause_ctr (PAUSE_LOOPS) auf 0 |
| STATE_ALIGN_REVERSE | STATE_START_PAUSE | m_backward_ctr (222 Loops) auf 0 |
| STATE_START_PAUSE | STATE_APPROACH | m_pause_ctr (PAUSE_LOOPS) auf 0 |
| STATE_APPROACH | STATE_STOP_AND_READ | wide_bar_active() AND POST_BAR_DRIVE_LOOPS abgelaufen |
| STATE_REAL_FOLLOW | STATE_STOP_AND_READ | wide_bar_active() AND m_guard_ctr==0 |
| STATE_SMALL_FOLLOW | STATE_STOP_AND_READ | small_line_active() AND m_guard_ctr==0 |
| STATE_TUNNEL_FOLLOW | STATE_STOP_AND_READ | small_line_active() AND m_guard_ctr==0 |
| STATE_STOP_AND_READ | STATE_RED | m_action_color == 3 (ROT) |
| STATE_STOP_AND_READ | STATE_GREEN | m_action_color == 5 (GRUEN) |
| STATE_STOP_AND_READ | STATE_BLUE | m_action_color == 7 (BLAU) |
| STATE_STOP_AND_READ | STATE_YELLOW | m_action_color == 4 (GELB) |
| STATE_STOP_AND_READ | exitWideBar()/exitSmallLine() | Timeout ODER unbekannte Farbe |
| STATE_RED/YELLOW (Arm fertig, nicht letzter Balken) | STATE_REAL_FOLLOW oder STATE_SMALL_FOLLOW | runDeliverPhase() == true, exitWideBar()/exitSmallLine() |
| STATE_RED/YELLOW (letzter Balken) | (Rueckfahrt) dann exitWideBar()/exitSmallLine() | m_reversing nach runDeliverPhase() |
| STATE_GREEN/BLUE (Arm fertig) | STATE_REAL_FOLLOW oder STATE_SMALL_FOLLOW | runDeliverPhase() == true |
| exitWideBar() wenn line_count < 4 | STATE_REAL_FOLLOW | — |
| exitWideBar() wenn line_count >= 4 | STATE_TUNNEL_FOLLOW | — |
| exitSmallLine() wenn line_count < 8 | STATE_SMALL_FOLLOW | — |
| exitSmallLine() wenn line_count >= 8 | STATE_FINISH | — |

### Timing-Konstanten
| Name | Loops | Sekunden | Beschreibung |
|---|---|---|---|
| STRAIGHT_LOOPS | 70 | 1.4 s | Geradeausfahrt nach Ausrichtung |
| FOLLOW_1S_LOOPS | ~31 | ~0.625 s | Linienfolgen bis Bremsen (50/MAX_SPEED) |
| BRAKE_LOOPS | 12 | 0.24 s | Bremsrampe von Vollgas auf Null |
| PAUSE_LOOPS | 20 | 0.4 s | Standstill nach Bremsen |
| BACKWARD_LOOPS | 222 | 4.44 s | Rueckwaertsfahrt zur Startposition |
| ACCEL_LOOPS | 12 | 0.24 s | Anlauframpe in STATE_APPROACH |
| RESTART_ACCEL_LOOPS | 50 | 1.0 s | Anlauframpe nach jedem Stopp |
| REAL_APPROACH_GUARD | 20 | 0.4 s | Sperre vor Balkenerkennung in STATE_APPROACH |
| STOP_GUARD | ~47 | ~0.94 s | Sperre nach Querbalken (75/MAX_SPEED) |
| START_GUARD | ~188 | ~3.75 s | Sperre beim ersten Einlauf (300/MAX_SPEED) |
| SLOWDOWN_LOOPS | 13 | 0.26 s | Bremsrampe bei Farberkennung |
| COLOR_READ_DELAY | 50 | 1.0 s | Sperrzeit nach Stopp gegen Phantomfarben |
| CROSSING_STOP_LOOPS | 250 | 5.0 s | Lesefenster breite Balken |
| SMALL_CROSSING_STOP_LOOPS | 100 | 2.0 s | Lesefenster Schmallinien |
| SMALL_FOLLOW_START_GUARD | ~488 | ~9.77 s | Sperre nach 4. Balken (Tunnel; 781/MAX_SPEED) |
| SMALL_REENTRY_GUARD | ~63 | ~1.25 s | Sperre gegen Doppel-Trigger Schmallinie (100/MAX_SPEED) |
| POST_BAR_DRIVE_LOOPS | 9 | 0.18 s | Extra-Loops nach erster Balkenerkennung |
| SF360_TIMEOUT_LOOPS | 225 | 4.5 s | Tray-Warte-Timeout |
| ROT_GELB_DRIVE_LOOPS | 51 | 1.02 s | Vorwaertsfahrt ROT/GELB zum Slot |
| ROT_GELB_ACCEL_LOOPS | 10 | 0.2 s | Anlauframpe ROT/GELB Fahrt |
| ROT_GELB_BRAKE_LOOPS | 12 | 0.24 s | Bremsrampe ROT/GELB Fahrt |
| TUNNEL_FOLLOW_LOOPS | 100 | 2.0 s | Phase A Tunnel: Linienfolger |
| TUNNEL_STRAIGHT_LOOPS | 75 | 1.5 s | Phase B Tunnel: Gerade ohne Lenken |
| m_startup_delay | 150 | 3.0 s | Wartezeit nach Button-Druck |
| m_home_timer | 150 | 3.0 s | Wartezeit fuer Tray-Ausrichtung nach Start |

### runDeliverPhase() — 5 Phasen
| Phase | Dauer | Aktion |
|---|---|---|
| 0 | max SF360_TIMEOUT_LOOPS (4.5 s) | Warten bis Tray Zielwinkel ±35° erreicht (CCW-Fenster oder CW-Fenster 185–360°); verhindert Ablage bei falschem Winkel |
| 1 | D2_DOWN_ONLY_LOOPS=12 (0.24 s) breit / =10 (0.20 s) schmal | D2 absenken auf depthForColor(); D1 eingefahren; D2_SMALL_DESCENT_ACCEL=3.5 |
| 2 | PICKUP_PAUSE_LOOPS=50 (1.0 s) | D1 ausfahren auf extensionForColor(); Tray-Zielkorrektur auf angleForColor() |
| 3 | JIGGLE_LOOPS=100 (2.0 s) / JIGGLE_LOOPS_GRUEN=150 (3.0 s) / JIGGLE_LOOPS_SMALL=75 (1.5 s) | D1 Jiggle ±D1_JIGGLE_OFFSET mit Amplituden-Envelope; Tray synchroner Jiggle (serviceTray) |
| 4 | WIDE_BAR_SETTLE_LOOPS=5 (0.1 s) + RAISE_LOOPS=60 (1.2 s) | 0.1 s Settle-Pause (breite Balken), dann D2 hochfahren (D2_RAISE_ACCEL=3.0), D1 einziehen (D1_RETRACT_ACCEL=5.0); returns true |

---

## 4. SOFTWAREARCHITEKTUR

| Klasse | Library-Pfad | Funktion | Pins |
|---|---|---|---|
| DCMotor | lib/DCMotor/ | PID-geregelter DC-Motor (Encoder-Feedback, RPS) | M1: PWM=PB_13, ENC=PA_6/PC_7; M2: PWM=PA_9, ENC=PB_6/PB_7 |
| DigitalOut (enable) | mbed OS | Motorfreigabe H-Bruecke | PB_ENABLE_DCMOTORS = PB_15 |
| LineFollower | lib/LineFollower/ | Differentialantrieb-Linienfolger (P+nichtlinear, Eigen) | I2C1 SDA=PB_9, SCL=PB_8 |
| ColorSensor | lib/ColorSensor/ | TCS3200 Farbsensor-Klassifikation (R/G/B/Hue) | OUT=PB_3, LED=PC_3, S0=PA_4, S1=PB_0, S2=PC_1, S3=PC_0 |
| Servo (D1) | lib/Servo/ | 180° Servo horizontal (Bit-Bang PWM) | PC_8 (PB_D1) |
| Servo (D2) | lib/Servo/ | 180° Servo vertikal (Bit-Bang PWM) | PB_2 (PB_D0) |
| ServoFeedback360 | lib/ServoFeedback360/ | Parallax 360° Servo mit P-Regler (PwmIn-Feedback) | PWM=PB_12 (PB_D3), Feedback=PC_2 (A0) |
| NeoPixel | lib/NeoPixel/ | WS2812B Bit-Bang-Treiber (DWT-Cycle-Counter, GRB-Order) | PC_5 |

---

## 5. JIGGLE-PARAMETER

### D1 Jiggle-Amplituden
| Balken-Typ | Konstante | Wert | Grad-Aequivalent |
|---|---|---|---|
| Alle breiten Balken | D1_JIGGLE_OFFSET | 0.098f | ~12° |
| Schmallinien | D1_JIGGLE_OFFSET_SMALL | 0.057f | ~7° |

### Tray Jiggle-Amplituden (Grad, positions-basiert via P-Regler)
| Richtung | Balken-Typ | Bedingung | Konstante | Wert |
|---|---|---|---|---|
| Gegen Karussel (CW, positiv) | Breite Balken | immer | TRAY_JIG_CW_WIDE | 17.5° |
| Gegen Karussel (CW, positiv) | Schmallinien | immer | TRAY_JIG_CW_SMALL | 15.0° |
| In Karussel (CCW, negativ) | Breite Balken, Balken 1 (m_line_count==1) | symmetrisch | TRAY_JIG_SYM | 10.0° |
| In Karussel (CCW, negativ) | Breite Balken, Balken 2–4 | asymmetrisch | TRAY_JIG_CCW_WIDE | 5.0° |
| In Karussel (CCW, negativ) | Schmallinien 1–3 | asymmetrisch | TRAY_JIG_CCW_SMALL | 5.0° |
| In Karussel (CCW, negativ) | Schmallinie 4 (m_line_count==8) | symmetrisch | TRAY_JIG_SYM | 10.0° |

### Jiggle-Timing
| Parameter | Wert | Beschreibung |
|---|---|---|
| JIGGLE_LOOPS | 100 (2.0 s) | Alle breiten Balken ausser GRUEN |
| JIGGLE_LOOPS_GRUEN | 150 (3.0 s) | GRUEN breite Balken (Magnet loest schlechter) |
| JIGGLE_LOOPS_SMALL | 75 (1.5 s) | Schmallinien |
| JIGGLE_RAMP_LOOPS | 18 (0.36 s) | Auf-/Abrampe der Amplitude (Envelope 0→max→0) |
| D1_JIGGLE_PERIOD | 42 Loops (0.84 s) | Ein vollstaendiger D1-Zyklus |
| TRAY_JIGGLE_PERIOD | 42 Loops (0.84 s) | Ein vollstaendiger Tray-Zyklus (synchron zu D1) |
| WIDE_BAR_SETTLE_LOOPS | 5 (0.1 s) | Settle-Pause nach Jiggle vor Hochfahren (breite Balken) |

### Amplituden-Envelope Formel
- Jiggle-Phase ctr 0..jig_end  
- ramp = min(JIGGLE_RAMP_LOOPS, jig_end/3)  
- t = ctr/ramp wenn ctr < ramp; t = (jig_end-ctr)/ramp wenn ctr >= jig_end-ramp; sonst t = 1.0  
- eff_off = jig_off * t

### Tray-Kick-Parameter (serviceTray)
| Parameter | Wert | Beschreibung |
|---|---|---|
| Kick-Geschwindigkeit | -0.40f | Impulsdrehzahl wenn neues Ziel (25 Loops = 0.5 s) |
| Cruise-Geschwindigkeit | -0.30f | Langsame CCW-Drehung in Navigations-States |
| Kick-Fenster | 25 Loops (0.5 s) | Dauer des Kick-Impulses nach neuem Zielwinkel |
| In-Window-Toleranz | ±35° (CCW ≤35° oder ≥185°) | Bereich gilt als "nah am Ziel" fuer Phase-0-Freigabe |
