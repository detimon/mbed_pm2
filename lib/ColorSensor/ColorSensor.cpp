#include "ColorSensor.h"
#include <cmath>

// Implementation notes:
// - The sensor produces a square-wave output whose frequency is proportional to light intensity.
// - S2/S3 select the photodiode filter (R/G/B/Clear) and S0/S1 select output scaling.
// - A Ticker periodically wakes a worker thread that sequences through the filters and updates
//   m_color_* arrays.

/**
 * @brief Construct using default pin mapping for LED/S0..S3.
 *
 * Sets a default filter and scaling, loads calibration references, then starts the worker thread.
 */
ColorSensor::ColorSensor(PinName pin) : m_PwmIn(pin),
                                        m_S0(COLOR_SENSOR_S0, 0),
                                        m_S1(COLOR_SENSOR_S1, 0),
                                        m_S2(COLOR_SENSOR_S2, 0),
                                        m_S3(COLOR_SENSOR_S3, 0),
                                        m_Led(COLOR_SENSOR_LED, 0),
                                        m_Thread(osPriorityLow),
                                        m_AvgFilter{AvgFilter(N_FILTER), AvgFilter(N_FILTER), AvgFilter(N_FILTER), AvgFilter(N_FILTER)}
{
    setColor(WHITE);
    setFrequency(FREQ_002);
    setCalibration();

    // start thread
    m_Thread.start(callback(this, &ColorSensor::threadTask));

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    m_Ticker.attach(callback(this, &ColorSensor::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});
}

/**
 * @brief Construct with explicit wiring.
 *
 * Uses a shorter averaging window by default (faster response, less smoothing).
 */
ColorSensor::ColorSensor(PinName pin, PinName led, PinName s0, PinName s1, PinName s2, PinName s3) : m_PwmIn(pin),
                                                                                        m_S0(s0, 0),
                                                                                        m_S1(s1, 0),
                                                                                        m_S2(s2, 0),
                                                                                        m_S3(s3, 0),
                                                                                        m_Led(led, 0),
                                                                                        m_Thread(osPriorityLow),
                                                                                        m_AvgFilter{AvgFilter(N_FILTER), AvgFilter(N_FILTER), AvgFilter(N_FILTER), AvgFilter(N_FILTER)}
{
    setColor(WHITE);
    setFrequency(FREQ_002);
    setCalibration();

    // start thread
    m_Thread.start(callback(this, &ColorSensor::threadTask));

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    m_Ticker.attach(callback(this, &ColorSensor::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});
}

/**
 * @brief Destructor.
 *
 * Important ordering:
 * 1) Detach the ticker first so no further wakeups occur.
 * 2) Terminate the worker thread.
 *
 * Note: terminate() is an abrupt stop. It's acceptable here because the thread only touches
 * this object, but if you later add shared resources (I2C, heap allocations, mutexes), consider
 * a graceful shutdown.
 */
ColorSensor::~ColorSensor()
{
    m_Ticker.detach();
    m_Thread.terminate();
}

/**
 * @brief Reset the moving-average filters to the current raw readings.
 *
 * This forces the filtered output to match the latest raw sample immediately.
 */
void ColorSensor::reset()
{
    for (int i = 0; i < 4; i++) {
        m_AvgFilter[i].reset(m_color_Hz[i]);
        m_is_first_run[i] = true;
    }
}

/**
 * @brief Load hard-coded calibration references.
 *
 * The reference values below are frequency measurements (Hz) for:
 * - a "black" surface (offset / dark level)
 * - a "white" surface (normalization / white balance)
 *
 * These numbers depend on:
 * - sensor module tolerance
 * - selected scaling (S0/S1)
 * - LED state and brightness
 * - distance/angle and ambient light
 *
 * If you change any of those, remeasure and update the constants.
 */
void ColorSensor::setCalibration()
{
    m_reference_black.red = 333.22f;
    m_reference_black.green = 345.66f;
    m_reference_black.blue = 539.37f;
    m_reference_black.white = 1297.02f;

    m_reference_white.red = 598.44f;
    m_reference_white.green = 643.92f;
    m_reference_white.blue = 993.05f;
    m_reference_white.white = 2298.85f;

    m_calib_black[0] = m_reference_black.red;
    m_calib_black[1] = m_reference_black.green;
    m_calib_black[2] = m_reference_black.blue;
    m_calib_black[3] = m_reference_black.white;

    m_calib_white[0] = m_reference_white.red;
    m_calib_white[1] = m_reference_white.green;
    m_calib_white[2] = m_reference_white.blue;
    m_calib_white[3] = m_reference_white.white;

    const float eps = 1e-3f;
    float denom = (m_calib_white[3] - m_calib_black[3]);
    if (denom < eps) {
        // invalid calibration
        return;
    }

    for (int i = 0; i < 3; i++) {
        m_ratio[i] = (m_calib_white[i] - m_calib_black[i]) / denom;
        if (m_ratio[i] < eps) m_ratio[i] = eps; // avoid divide by zero later
    }
    
}

/**
 * @brief Select the photodiode filter via S2/S3.
 *
 * colorfilter_t values encode the required bit pattern (see header).
 */
void ColorSensor::setColor(colorfilter_t color)
{
    m_S3 = (color >> 0) & 1; // LSB
    m_S2 = (color >> 1) & 1; // MSB
}

/**
 * @brief Select output scaling via S0/S1.
 *
 * frequency_t values encode the required bit pattern (see header).
 */
void ColorSensor::setFrequency(frequency_t frequency)
{
    m_S1 = (frequency >> 0) & 1; // LSB
    m_S0 = (frequency >> 1) & 1; // MSB
}


/**
 * @brief Turn illumination LED on/off.
 */
void ColorSensor::switchLed(ledstate_t led_state)
{
    m_Led = !led_state;
}

/**
 * @brief Worker thread: periodically samples all 4 channels.
 *
 * Woken by a Ticker via sendThreadFlag(). For each channel, we select the filter,
 * wait briefly for the sensor output to settle, then measure the output period.
 */
void ColorSensor::threadTask()
{
    while (true) {
        // Block until the ticker signals us.
        ThisThread::flags_wait_any(m_ThreadFlag);

        // Sequence through channels in the order defined by m_color[].
        for (int i = 0; i < 4; i++)
        {
            setColor(m_color[i]);
            // Allow filter switching and output frequency to stabilize.
            ThisThread::sleep_for(25ms);

            // PwmIn::period() returns the measured signal period (typically in seconds).
            float period = m_PwmIn.period();

            // catch division by zero
            if (period > 1e-6f) {
                m_color_Hz[i] = 1.0f/period;

                // average filtered color
                if (m_is_first_run[i]) {
                    m_is_first_run[i] = false;
                    m_color_Avg_Hz[i] = m_AvgFilter[i].reset(m_color_Hz[i]);
                } else {
                    m_color_Avg_Hz[i] = m_AvgFilter[i].apply(m_color_Hz[i]);
                }
            }
        }    
        applyCalibration(m_color_Avg_Hz);    
    }
}

/**
 * @brief Apply black/white calibration and compute normalized RGB.
 *
 * Steps:
 * 1) Subtract black reference (dark offset).
 * 2) Convert RGB to chromatic ratios by dividing by Clear.
 * 3) White-balance by dividing by precomputed m_ratio[] (derived from white reference).
 * 4) Normalize RGB so max(R,G,B)=1 (useful for classification/thresholding).
 */
void ColorSensor::applyCalibration(const float *color)
{
    float color0[4];

    // Subtract black reference and clamp
    for (int i = 0; i < 4; i++) {
        color0[i] = color[i] - m_calib_black[i];
        color0[i] = fmax(color0[i], 0.0f);
    }

    const float eps = 1e-3f;
    
    // Clear channel after black subtraction is the denominator
    if (color0[3] < eps) {
        return; // too dark / invalid
    }

    m_color_cal[3] = color0[3];
    
    // Chromatic ratios (relative to clear)
    float ratio[3];
    for (int i = 0; i < 3; i++) {
        ratio[i] = color0[i] / color0[3];
    }

    // White-balance
    for (int i = 0; i < 3; i++) {
        if (m_ratio[i] > eps) {
            m_color_cal[i] = ratio[i] / m_ratio[i];
        }
    }

    // Normalize RGB
    const float max_rgb = fmax(m_color_cal[0], fmax(m_color_cal[1], m_color_cal[2]));
    if (max_rgb < eps) {
        return;
    }

    for (int i = 0; i < 3; i++) {
        m_color_norm[i] = m_color_cal[i] / max_rgb;
    }

}

int ColorSensor::getColor()
{
    // Brightness after black subtraction (Hz)
    const float c0 = std::max(0.0f, m_color_cal[3]);

    // ---- Tunables ----
    const float C0_BLACK_MAX    = 80.0f;  // below => BLACK
    const float C0_WHITE_MIN    = 650.0f; // above + neutral => WHITE
    // The chromatic-ratio calibration compresses colour signals heavily when
    // mixed with white floor light.  At ~35% floor contamination the satp in
    // calibrated space already drops below 0.08 for green/blue cards.
    // Setting SATP_GRAY_MAX=0.01 keeps pure white floor (satp≈0) as WHITE
    // while letting diluted colours through to the hue classifier.
    const float SATP_GRAY_MAX   = 0.03f;  // below => neutral (for WHITE check)
    // Similarly, norm(max-min) drops quickly with dilution.  0.03 allows
    // detection up to ~83% white floor contamination.
    const float NORM_DELTA_MIN  = 0.03f;  // norm(max-min) below => too diluted to classify

    // Hue-based colour thresholds (degrees, 0-360).
    // Messung 2026-04-19 (neue Sensorhalterung):
    //   BLAU:  hue ≈ 364°–3°  (wrap — sehr niedrig, knapp unter ROT)
    //   ROT:   hue ≈ 11°–14°
    //   WHITE: hue ≈ 30°       (UNKNOWN-Schutzzone)
    //   GELB:  hue ≈ 37°–40°
    //   GRÜN:  hue ≈ 62°–66°
    const float HUE_BLUE_MAX_LOW  =   7.0f;  // 0°–7°     = BLAU (wrap-Bereich)
    const float HUE_RED_MAX       =  20.0f;  // 7°–20°    = ROT
    const float HUE_WHITE_MAX     =  28.0f;  // 20°–28°   = UNKNOWN (WHITE ≈30° fällt jetzt in GELB, Sättigungsfilter schützt)
    const float HUE_YELLOW_MAX    =  52.0f;  // 28°–52°   = GELB (9° Puffer unter gemessenem Min ≈37°)
    const float HUE_GREEN_MAX     =  80.0f;  // 50°–80°   = GRÜN
    const float HUE_BLUE_MIN_HIGH = 348.0f;  // 348°–360° = BLAU (wrap-Bereich, erweitert)

    const int STABLE_COUNT = 1;

    // ---- Kalibrierte Kanalwerte (für BLACK/WHITE-Check) ----
    float r0 = std::max(0.0f, m_color_cal[0]);
    float g0 = std::max(0.0f, m_color_cal[1]);
    float b0 = std::max(0.0f, m_color_cal[2]);

    const float eps = 1e-6f;
    const float sum = r0 + g0 + b0;
    const float mx0 = std::max(r0, std::max(g0, b0));
    const float mn0 = std::min(r0, std::min(g0, b0));
    const float satp = (mx0 - mn0) / std::max(sum, eps);

    int candidate = 0;

    if (c0 < C0_BLACK_MAX) {
        candidate = 1; // BLACK
    } else {
        // ---- Hue-Berechnung (immer, auch bei niedriger Sättigung) ----
        float rn = m_color_norm[0];
        float gn = m_color_norm[1];
        float bn = m_color_norm[2];
        float cmax = std::max(rn, std::max(gn, bn));
        float cmin = std::min(rn, std::min(gn, bn));
        float delta = cmax - cmin;

        float hue = 0.0f;
        if (delta > 1e-6f) {
            if      (cmax == rn) hue = 60.0f * fmodf((gn - bn) / delta, 6.0f);
            else if (cmax == gn) hue = 60.0f * ((bn - rn) / delta + 2.0f);
            else                 hue = 60.0f * ((rn - gn) / delta + 4.0f);
            if (hue < 0.0f) hue += 360.0f;
        }

        // BLAU-Zone hat Priorität vor WHITE-Check, damit niedrige Sättigung
        // in der BLAU-Zone nicht als UNKNOWN klassifiziert wird.
        const bool in_blue_zone = (hue < HUE_BLUE_MAX_LOW || hue > HUE_BLUE_MIN_HIGH);

        if (in_blue_zone) {
            candidate = 7; // BLAU
        } else if (satp < SATP_GRAY_MAX && c0 > C0_WHITE_MIN) {
            candidate = 0; // WHITE → UNKNOWN (keine Action-Farbe)
        } else if (delta < NORM_DELTA_MIN) {
            candidate = 0; // zu stark verdünnt → UNKNOWN
        } else if (hue < HUE_RED_MAX)     candidate = 3; // ROT
        else if  (hue < HUE_WHITE_MAX)    candidate = 0; // UNKNOWN (WHITE ≈30°)
        else if  (hue < HUE_YELLOW_MAX)   candidate = 4; // GELB
        else if  (hue < HUE_GREEN_MAX)    candidate = 5; // GRÜN
        else                               candidate = 0; // UNKNOWN (80°–348°)

#if COLOR_DEBUG
        printf("c0=%.1f  rn=%.3f gn=%.3f bn=%.3f  delta=%.3f  hue=%.1f  out=%d\n",
               c0, rn, gn, bn, delta, hue, candidate);
#endif
    }

    // ---- Hysterese ----
    static int last = 0;
    static int cnt  = 0;
    if (candidate == last) cnt = std::min(cnt + 1, STABLE_COUNT);
    else { last = candidate; cnt = 1; }
    return (cnt >= STABLE_COUNT) ? candidate : last;
}

const char* ColorSensor::getColorString(int color)
{
    switch (color) {
        case 0: return "UNKNOWN";
        case 1: return "BLACK";
        case 2: return "WHITE";
        case 3: return "RED";
        case 4: return "YELLOW";
        case 5: return "GREEN";
        case 6: return "CYAN";
        case 7: return "BLUE";
        case 8: return "MAGENTA";
        default: return "INVALID";
    }
}

/**
 * @brief Wake the worker thread.
 *
 * Called by the ticker at PERIOD_MUS cadence.
 */
void ColorSensor::sendThreadFlag()
{
    m_Thread.flags_set(m_ThreadFlag);
}