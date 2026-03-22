---
glob: lib/**/*.{cpp,h}
---

# Regeln für Hardware-Treiber (lib/**)

## Allgemein
- Library-Code ist geteilter, stabiler Code — minimale, rückwärtskompatible Änderungen
- Jede Klasse hat Header-Guard `#ifndef FOO_H_ / #define FOO_H_`
- Doxygen-Style Kommentare für alle public-Methoden (@brief, @param, @return)
- Member-Variablen: `m_`-Präfix, z.B. `m_velocity`, `m_Kp`

## Threading (DCMotor, LineFollower)
- Eigene Threads via `Thread` + `Ticker` + `ThreadFlag` — dieses Pattern nicht ändern
- Thread-Funktion heisst `threadTask()` oder `followLine()`
- `sendThreadFlag()` wird vom Ticker aufgerufen und setzt das Flag

## DCMotor
- Geschwindigkeit immer in RPS (nicht rad/s, nicht m/s)
- Default-PID: KP=4.2, KI=140.0, KD=0.0192 (für Gear Ratio 78:1 optimiert)
- `PERFORM_GPA_MEAS` und `PERFORM_CHIRP_MEAS` niemals beide `true` gleichzeitig
- PWM-Limits: PWM_MIN=0.01, PWM_MAX=0.99 — nicht unterschreiten

## LineFollower
- Benötigt Eigen (`#include <Eigen/Dense>`) für Wheel-Koordinaten-Transformation
- `m_Cwheel2robot` ist eine 2x2-Matrix (Differentialantrieb-Kinematik)
- `getAngleRadians()` gibt den Winkel zur Linie in Radians zurück (0 = gerade)
- `isLedActive()` = true, wenn mindestens eine IR-LED der SensorBar die Linie sieht

## Eigen-Lib
- Build-Flags `EIGEN_NO_DEBUG` und `EIGEN_DONT_VECTORIZE` sind Pflicht (platformio.ini)
- Keine dynamischen Eigen-Objekte (kein `MatrixXf`) — nur fixe Grössen (`Matrix2f`, `Vector2f`)
