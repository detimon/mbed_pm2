#include "test_config.h"

#ifdef TEST_ENDSCHALTER

#include "test_endschalter.h"

// Endschalter: PC_2 (A0 auf PES-Board)
static DigitalIn* g_sw   = nullptr;

static int m_state       = 1; // aktueller Zustand: 1=offen, 0=gedrueckt
static int m_prev        = 1; // vorheriger Zustand fuer Flanken-Erkennung
static int m_count       = 0; // Anzahl Betaetigungen (fallende Flanke)

void endschalter_init(int loops_per_second)
{
    (void)loops_per_second;

    static DigitalIn sw(PC_2, PullUp); // A0
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
    // Pin direkt lesen, unabhängig davon ob Task aktiv ist
    int raw = g_sw ? g_sw->read() : -1;
    printf("Endschalter(A0/PC_2): raw=%d  state=%s  cnt=%d\n",
           raw,
           raw == 0 ? "GEDRUECKT" : "offen    ",
           m_count);
}

#endif // TEST_ENDSCHALTER
