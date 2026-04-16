#include "test_config.h"

#ifdef TEST_COLOR_NEOPIXEL

#include "test_color_neopixel.h"

static ColorSensor* g_cs = nullptr;
static NeoPixel*    g_np = nullptr;

static int         m_color_num    = -1;
static const char* m_color_string = nullptr;
static float       m_color_norm[3] = {0.0f, 0.0f, 0.0f};
static float       m_hue          = 0.0f;

void color_neopixel_init(int loops_per_second)
{
    (void)loops_per_second;

    // pin order: freq_out, led_enable, S0, S1, S2, S3
    static ColorSensor sensor(PB_3, PC_3, PA_4, PB_0, PC_1, PC_0);
    g_cs = &sensor;
    g_cs->switchLed(ON);

    static NeoPixel np(PB_2);
    g_np = &np;
    g_np->clear();
}

void color_neopixel_task(DigitalOut& led1)
{
    led1 = !led1;

    m_color_num    = g_cs->getColor();
    m_color_string = g_cs->getColorString(m_color_num);

    const float* norm = g_cs->readColorNorm();
    for (int i = 0; i < 3; i++) m_color_norm[i] = norm[i];
    float r = norm[0], g = norm[1], b = norm[2];
    float cmax = fmaxf(r, fmaxf(g, b));
    float cmin = fminf(r, fminf(g, b));
    float delta = cmax - cmin;
    float hue = 0.0f;
    if (delta > 1e-6f) {
        if      (cmax == r) hue = 60.0f * fmodf((g - b) / delta, 6.0f);
        else if (cmax == g) hue = 60.0f * ((b - r) / delta + 2.0f);
        else                hue = 60.0f * ((r - g) / delta + 4.0f);
        if (hue < 0.0f) hue += 360.0f;
    }
    m_hue = hue;

    // Dimmed mapping — only the 4 action colors drive the NeoPixel
    uint8_t pr = 0, pg = 0, pb = 0;
    switch (m_color_num) {
        case 3: pr = 60; pg =  0; pb =  0; break; // ROT
        case 4: pr = 60; pg = 25; pb =  0; break; // GELB (leicht orange-verschoben)
        case 5: pr =  0; pg = 60; pb =  0; break; // GRÜN
        case 7: pr =  0; pg =  0; pb = 60; break; // BLAU
        default: break;                            // AUS
    }
    g_np->setRGB(pr, pg, pb);
    g_np->show();
}

void color_neopixel_reset(DigitalOut& led1)
{
    led1 = 0;
    if (g_np) g_np->clear();
    m_color_num    = -1;
    m_color_string = nullptr;
    m_hue          = 0.0f;
    for (int i = 0; i < 3; i++) m_color_norm[i] = 0.0f;
}

void color_neopixel_print()
{
    const char* name = m_color_string ? m_color_string : "---";
    printf("Color=%-7s Hue=%6.1f deg  RGBn=(%.2f, %.2f, %.2f)\n",
           name, m_hue, m_color_norm[0], m_color_norm[1], m_color_norm[2]);
}

#endif // TEST_COLOR_NEOPIXEL
