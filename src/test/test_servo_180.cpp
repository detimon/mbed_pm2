// +==============================================================+
// |        INBETRIEBNAHMEPROTOKOLL - LineXpress FS26             |
// +==============================================================+
// | Datei      : test_servo_180.cpp
// | Komponente : 180-Grad-Servo A (horizontal)
// | BMK        : M20
// | Pin(s)     : PB_D1 = PC_8
// +--------------------------------------------------------------+
// | TESTZIEL   : Servo faehrt langsam von 0 bis 180 Grad und stoppt am oberen Anschlag.
// | VORAUSSETZ.: calib_servo_180 abgeschlossen, Pulsweiten in calibration_values.h aktuell.
// +--------------------------------------------------------------+
// | ERWARTETES VERHALTEN:
// |   Pulse-Wert steigt um 0.005 pro Sekunde. Keine Stockungen, kein Brummen.
// +--------------------------------------------------------------+
// | TESTDATUM  : __.__.____ | TESTER: _______________
// | ERGEBNIS   : [ ] Bestanden   [ ] Nicht bestanden
// | BEMERKUNGEN: ______________________________________________
// +==============================================================+

#include "test_config.h"

#ifdef TEST_SERVO_180

#include "test_servo_180.h"

static Servo*  mp_servo        = nullptr;
static int     m_lps           = 50;
static float   m_servo_input   = 0.0f;
static int     m_servo_counter = 0;

void servo_180_init(int loops_per_second)
{
    m_lps = loops_per_second;

    static Servo servo(PB_D1);
    mp_servo = &servo;
    mp_servo->calibratePulseMinMax(0.050f, 0.1050f);
    mp_servo->setMaxAcceleration(0.3f);
}

void servo_180_task(DigitalOut& led1)
{
    led1 = 1;

    if (!mp_servo->isEnabled())
        mp_servo->enable();

    mp_servo->setPulseWidth(m_servo_input);

    if ((m_servo_input < 1.0f) &&
        (m_servo_counter % m_lps == 0) &&
        (m_servo_counter != 0))
        m_servo_input += 0.005f;

    m_servo_counter++;
}

void servo_180_reset(DigitalOut& led1)
{
    led1 = 0;
    mp_servo->disable();
    m_servo_input   = 0.0f;
    m_servo_counter = 0;
}

void servo_180_print()
{
    printf("Servo input: %.4f\n", m_servo_input);
}

#endif // TEST_SERVO_180
