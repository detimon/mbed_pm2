#include "test_config.h"

#ifdef TEST_ENDSCHALTER

#include "test_endschalter.h"

// Endschalter: PC_5 (PB_A2 = A2)
static DigitalIn* g_sw   = nullptr;

static int m_state       = 1; // aktueller Zustand: 1=offen, 0=gedrueckt
static int m_prev        = 1; // vorheriger Zustand fuer Flanken-Erkennung
static int m_count       = 0; // Anzahl Betaetigungen (fallende Flanke)

void endschalter_init(int loops_per_second)
{
    (void)loops_per_second;

    static DigitalIn sw(PC_5, PullUp); // A2
    g_sw = &sw;

    m_prev = g_sw->read();
}

void endschalter_task(DigitalOut& led)
{
    led = !led;

    m_state = g_sw->read();

    // Fallende Flanke = Schalter wurde gerade gedrueckt (PullUp: 1->0)
    if (m_prev == 1 && m_state == 0)
        m_count++;

    m_prev = m_state;
}

void endschalter_reset(DigitalOut& led)
{
    led     = 0;
    m_state = 1;
    m_prev  = 1;
    m_count = 0;
}

void endschalter_print()
{
    printf("Endschalter(A2): %s  cnt=%d\n",
           m_state == 0 ? "GEDRUECKT" : "offen    ",
           m_count);
}

#endif // TEST_ENDSCHALTER
