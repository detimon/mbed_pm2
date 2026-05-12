// +==============================================================+
// |        INBETRIEBNAHMEPROTOKOLL - LineXpress FS26             |
// +==============================================================+
// | Datei      : test_ir_sensor.cpp
// | Komponente : IR-Sensor (Analog)
// | BMK        : B3
// | Pin(s)     : PB_1 (A3)
// +--------------------------------------------------------------+
// | TESTZIEL   : Spannungsausgabe des IR-Sensors als gefilterter Mittelwert ueberpruefen.
// | VORAUSSETZ.: Sensor verkabelt, freie Sicht ohne Hindernis vor Sensor.
// +--------------------------------------------------------------+
// | ERWARTETES VERHALTEN:
// |   Wert in mV laeuft monoton mit Annaeherung eines Hindernisses (z.B. Hand).
// +--------------------------------------------------------------+
// | TESTDATUM  : __.__.____ | TESTER: _______________
// | ERGEBNIS   : [ ] Bestanden   [ ] Nicht bestanden
// | BEMERKUNGEN: ______________________________________________
// +==============================================================+

#include "test_config.h"

#ifdef TEST_IR_SENSOR

#include "test_ir_sensor.h"

// IR sensor on PB_1 (= A3) — unkalibriert, Rohwert in Millivolt
static IRSensor* mp_ir_sensor = nullptr;
static float     m_ir_value   = 0.0f; // gefilterter Durchschnittswert in mV

void ir_sensor_init(int loops_per_second)
{
    (void)loops_per_second;

    static IRSensor ir_sensor(PB_1);
    mp_ir_sensor = &ir_sensor;
    mp_ir_sensor->reset(); // Filter auf aktuellen Rohwert setzen
}

void ir_sensor_task(DigitalOut& led1)
{
    led1 = 1;

    m_ir_value = mp_ir_sensor->read(); // gefilterter Wert (mV, da unkalibriert)
}

void ir_sensor_reset(DigitalOut& led1)
{
    led1 = 0;
    m_ir_value = 0.0f;
}

void ir_sensor_print()
{
    printf("IR sensor: %.1f mV\n", m_ir_value);
}

#endif // TEST_IR_SENSOR
