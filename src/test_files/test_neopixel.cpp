#include "test_config.h"

#ifdef TEST_NEOPIXEL

#include "test_neopixel.h"

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

void neopixel_init(int loops_per_second)
{
    m_lps = loops_per_second;

    static NeoPixel np(PB_2);
    g_np = &np;

    g_np->clear();
}

void neopixel_task(DigitalOut& led1)
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

void neopixel_reset(DigitalOut& led1)
{
    led1 = 0;
    g_np->clear();
    m_step_ctr = 0;
    m_step_idx = 0;
}

void neopixel_print()
{
    const uint8_t* c = COLORS[m_step_idx];
    printf("NeoPixel step %d  RGB=(%3u,%3u,%3u)\n",
           m_step_idx, c[0], c[1], c[2]);
}

#endif // TEST_NEOPIXEL
