---
glob: src/test_*.{cpp,h}
---

# Regeln für Test-Module (src/test_*.cpp / src/test_*.h)

## Pflicht-Struktur
Jedes Test-Modul implementiert exakt diese vier Funktionen — nicht mehr, nicht weniger:
- `<name>_init(int loops_per_second)` — Objekte anlegen, Hardware initialisieren
- `<name>_task(DigitalOut& led)` — wird 50x/s aufgerufen, erstes Statement immer `led = !led`
- `<name>_reset(DigitalOut& led)` — Motoren stoppen, led = 0, Kommandos auf 0
- `<name>_print()` — `printf()`-Ausgabe, eine Zeile, kein `\n` am Anfang

## Hardware-Objekte
- Immer als `static`-lokale Variable in `_init()` anlegen, niemals `new` verwenden
- Globale Zeiger mit `g_`-Präfix und `= nullptr` initialisieren
- `enable_motors` (PB_ENABLE_DCMOTORS) immer erst in `_task()` auf 1 setzen, in `_reset()` auf 0

## LINE_FOLLOWER-Tests spezifisch
- `TIM1->BDTR |= TIM_BDTR_MOE` muss in `_init()` stehen (STM32-MOE-Pflicht)
- `VEL_SIGN = -1.0f` nicht ändern ohne Hardware-Verifikation
- SensorBar belegt PB_9/PB_8 als I2C — kein DigitalOut auf PB_9 in diesen Tests

## test_config.h
- Immer genau ein `#define` aktiv lassen
- Vor Commit sicherstellen, dass das gewünschte Modul aktiv ist
