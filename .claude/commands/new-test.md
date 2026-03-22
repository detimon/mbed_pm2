# Neues Test-Modul erstellen

Der Benutzer möchte ein neues Test-Modul anlegen.
Argument: `$ARGUMENTS` = Name des Tests in snake_case (z.B. `servo_ir` oder `ir_sensor`)

Führe diese Schritte aus:

## Schritt 1: Namen ableiten

Aus `$ARGUMENTS` (z.B. `servo_ir`) ableiten:
- Dateiname:     `test_servo_ir`
- Define-Name:   `TEST_SERVO_IR` (Grossbuchstaben, Unterstriche)
- Funktions-Präfix: `servo_ir` (snake_case)

## Schritt 2: Header-Datei erstellen

Erstelle `src/test_<name>.h` mit exakt dieser Struktur:

```cpp
#ifndef TEST_<NAME>_H_
#define TEST_<NAME>_H_

#include "mbed.h"

void <präfix>_init(int loops_per_second);
void <präfix>_task(DigitalOut& led);
void <präfix>_reset(DigitalOut& led);
void <präfix>_print();

#endif /* TEST_<NAME>_H_ */
```

## Schritt 3: Implementierungs-Datei erstellen

Erstelle `src/test_<name>.cpp` mit exakt dieser Struktur:

```cpp
#include "test_<name>.h"

// --- Hier die benötigten Library-Includes einfügen ---
// #include "Servo.h"
// #include "IRSensor.h"
// #include "PESBoardPinMap.h"

// Globale Zeiger auf Hardware-Objekte (werden in _init() gesetzt)
// static Servo*     g_servo_a = nullptr;
// static IRSensor*  g_ir      = nullptr;

void <präfix>_init(int loops_per_second)
{
    // Hardware-Objekte als static anlegen (bleiben dauerhaft im Speicher)
    // static Servo servo_a(PC_8);   // 180° Servo A
    // g_servo_a = &servo_a;
}

void <präfix>_task(DigitalOut& led)
{
    led = !led;  // LED blinkt — zeigt dass der Task läuft

    // Hauptlogik hier
}

void <präfix>_reset(DigitalOut& led)
{
    led = 0;

    // Hardware deaktivieren / Werte zurücksetzen
}

void <präfix>_print()
{
    printf("<präfix>: Wert=...\n");
}
```

## Schritt 4: Anleitung anzeigen

Zeige dem Benutzer danach:

```
Dateien erstellt:
  src/test_<name>.h
  src/test_<name>.cpp

Jetzt noch 2 Stellen anpassen:

1. src/test_config.h — Zeile hinzufügen:
   // #define TEST_<NAME>

2. src/main.cpp — Block hinzufügen:
   #elif defined(TEST_<NAME>)
       #include "test_<name>.h"
       #define TEST_INIT(lps)  <präfix>_init(lps)
       #define TEST_TASK(led)  <präfix>_task(led)
       #define TEST_RESET(led) <präfix>_reset(led)
       #define TEST_PRINT()    <präfix>_print()
```
