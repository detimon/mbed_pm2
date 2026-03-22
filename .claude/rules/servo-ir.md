---
glob: src/test_servo*.{cpp,h}
---

# Regeln für Servo- und IR-Sensor-Tests

## Servo-Pins (aus Pin-Layout)
- **180° Servo A** → `PC_8`  (= `PB_D1`) — D1
- **180° Servo B** → `PC_6`  (= `PB_D2`) — D2
- **360° Servo**   → `PB_12` (= `PB_D3`) — D3

## 180° Servo — Kalibrierung
Immer diese Pulse-Werte verwenden (getestet mit FS90-Servo):
```cpp
servo.calibratePulseMinMax(0.050f, 0.1050f);
```
- `setPulseWidth(0.0f)` = 0° (eine Endlage)
- `setPulseWidth(0.5f)` = 90° (Mitte)
- `setPulseWidth(1.0f)` = 180° (andere Endlage)

## 360° Kontinuierlich-Servo — Verhalten
Ein 360°-Servo dreht sich statt zu positionieren.
```cpp
servo.calibratePulseMinMax(0.050f, 0.1050f); // gleiche Kalibrierung
```
- `setPulseWidth(0.5f)` = Stopp (Mittelpunkt)
- `setPulseWidth(0.0f)` = volle Drehung eine Richtung
- `setPulseWidth(1.0f)` = volle Drehung andere Richtung
- Werte um 0.5f herum = langsame Drehung (je weiter von 0.5f, desto schneller)
- Achtung: genaue Stopp-Position (0.5f) muss nach erstem Test feinabgestimmt werden

## Servo — allgemeine Regeln
- `setMaxAcceleration(0.3f)` für sanfte Bewegung bei 180°-Servos
- Servo in `_init()` anlegen, in `_task()` erst enablen wenn Task aktiv
- In `_reset()`: `servo.disable()` aufrufen und Position auf 0.0f zurücksetzen
- Servo-Objekt als `static` in `_init()` anlegen, kein `new`

## IR-Sensor
- Pin: `PB_1` (= `A3`)
- Klasse: `IRSensor` aus `lib/IRSensor/IRSensor.h`

```cpp
// Initialisierung (unkalibriert — gibt Millivolt zurück):
static IRSensor ir_sensor(PB_1);

// Wert lesen:
float wert_mv  = ir_sensor.readmV();  // Rohwert in Millivolt
float wert_avg = ir_sensor.read();    // gefilterter Durchschnitt (verwende diesen)
```
- `read()` liefert den geglätteten Wert — immer diesen verwenden, nicht `readmV()` direkt
- Unkalibriert: Wert in mV (ca. 0–3300 mV je nach Abstand)
- Kalibriert: `setCalibration(a, b)` — lineare Umrechnung in cm
- Kalibrierung erst nach erstem Messtest eintragen (Werte aufnehmen, dann Formel bestimmen)
