# CargoSweep — State Machine Flowchart

> Gerendert automatisch auf GitHub. Zum Exportieren: [mermaid.live](https://mermaid.live) → Code einfügen → PNG/SVG herunterladen.

```mermaid
stateDiagram-v2
    [*] --> STATE_BLIND : Button drücken

    STATE_BLIND --> STATE_STRAIGHT : Alle 8 IR-Sensoren aktiv\n(Startlinie erkannt)
    STATE_STRAIGHT --> STATE_TURN_SPOT : Timer abgelaufen\n(1.4 s = 70 Loops)
    STATE_TURN_SPOT --> STATE_FOLLOW : Mittelsensor 3 oder 4 aktiv

    STATE_FOLLOW --> STATE_FOLLOW_1S : Querbalken erkannt\n(≥4 Sensoren aktiv)
    STATE_FOLLOW_1S --> STATE_BRAKE : Timer abgelaufen\n(1 s = 50 Loops)
    STATE_BRAKE --> STATE_PAUSE : Bremsrampe fertig\n(0.24 s = 12 Loops)
    STATE_PAUSE --> STATE_BACKWARD : Pause fertig\n(0.4 s = 20 Loops)
    STATE_BACKWARD --> STATE_REAL_START_PAUSE : Rückwärtsfahrt fertig\n(4.44 s = 222 Loops)
    STATE_REAL_START_PAUSE --> STATE_REAL_APPROACH : Pause fertig\n(0.4 s)
    STATE_REAL_APPROACH --> STATE_REAL_FOLLOW : Querbalken erkannt

    note right of STATE_REAL_FOLLOW
        Hauptprogramm
        4× Querbalken
    end note

    STATE_REAL_FOLLOW --> STATE_CROSSING_STOP : Querbalken erkannt\n+ Farbe bestätigt

    STATE_CROSSING_STOP --> STATE_REAL_FOLLOW : GRÜN/BLAU Servo-Sequenz fertig\nNoch Querbalken übrig
    STATE_CROSSING_STOP --> STATE_ROT_GELB_DRIVE : Farbe = ROT oder GELB
    STATE_CROSSING_STOP --> STATE_SMALL_FOLLOW : 4. Querbalken abgeschlossen

    STATE_ROT_GELB_DRIVE --> STATE_ROT_GELB_PAUSE : 120 mm Linienfolge fertig
    STATE_ROT_GELB_PAUSE --> STATE_REAL_FOLLOW : Arm-Sequenz fertig\n(kam von Querbalken)
    STATE_ROT_GELB_PAUSE --> STATE_SMALL_FOLLOW : Arm-Sequenz fertig\n(kam von kleiner Linie)

    note right of STATE_SMALL_FOLLOW
        4× kleine Linien
    end note

    STATE_SMALL_FOLLOW --> STATE_SMALL_CROSSING_STOP : Kleine Linie erkannt\n(Sensoren 2-4 oder 3-5)
    STATE_SMALL_CROSSING_STOP --> STATE_SMALL_FOLLOW : Servo-Sequenz fertig\nNoch Linien übrig
    STATE_SMALL_CROSSING_STOP --> STATE_ROT_GELB_DRIVE : Farbe = ROT oder GELB
    STATE_SMALL_CROSSING_STOP --> STATE_FINAL_HALT : 4. kleine Linie abgeschlossen

    STATE_FINAL_HALT --> [*]
```
