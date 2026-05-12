// +==============================================================+
// |        INBETRIEBNAHMEPROTOKOLL - LineXpress FS26             |
// +==============================================================+
// | Datei      : test_neopixel_basic.cpp
// | Komponente : WS2812B NeoPixel Selbsttest
// | BMK        : P1
// | Pin(s)     : PB_A2 = PC_5
// +--------------------------------------------------------------+
// | TESTZIEL   : NeoPixel-Treiber zyklisch RGB-Farben anzeigen lassen.
// | VORAUSSETZ.: 5V-Versorgung der NeoPixel-Linie.
// +--------------------------------------------------------------+
// | ERWARTETES VERHALTEN:
// |   LED leuchtet abwechselnd rot, gruen, blau im Sekundentakt.
// +--------------------------------------------------------------+
// | TESTDATUM  : __.__.____ | TESTER: _______________
// | ERGEBNIS   : [ ] Bestanden   [ ] Nicht bestanden
// | BEMERKUNGEN: ______________________________________________
// +==============================================================+

#include "test_config.h"

#ifdef TEST_NEOPIXEL_BASIC

#include "test_neopixel_basic.h"

// NeoPixel DIN = PB_2 (D0) per drawing list
static NeoPixel* g_np = nullptr;

static int m_lps       = 50;
static int m_step_ctr  = 0;
static int m_step_idx  = 0;

// cycle through: RED, YELLOW, GREEN, BLUE, OFF (1 s each)
static const uint8_t COLORS[][3] = {
    {255,   0,   0}, // ROT
    {255, 100,   0}, // GELB
    {  0, 255,   0}, // GRÜN
    {  0,   0, 255}, // BLAU
    {  0,   0,   0}, // AUS
};
static const int COLOR_COUNT = sizeof(COLORS) / sizeof(COLORS[0]);

void neopixel_basic_init(int loops_per_second)
{
    m_lps = loops_per_second;

    static NeoPixel np(PB_2);
    g_np = &np;

    g_np->clear();
}

void neopixel_basic_task(DigitalOut& led1)
{
    led1 = !led1;

    const uint8_t* c = COLORS[m_step_idx];
    g_np->setRGB(c[0], c[1], c[2]);
    g_np->show();

    m_step_ctr++;
    if (m_step_ctr >= m_lps) { // 1 s pro Schritt
        m_step_ctr = 0;
        m_step_idx = (m_step_idx + 1) % COLOR_COUNT;
    }
}

void neopixel_basic_reset(DigitalOut& led1)
{
    led1 = 0;
    g_np->clear();
    m_step_ctr = 0;
    m_step_idx = 0;
}

void neopixel_basic_print()
{
    const uint8_t* c = COLORS[m_step_idx];
    printf("NeoPixel step %d  RGB=(%3u,%3u,%3u)\n",
           m_step_idx, c[0], c[1], c[2]);
}

#endif // TEST_NEOPIXEL_BASIC
