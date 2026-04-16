#ifndef NEOPIXEL_H_
#define NEOPIXEL_H_

#include "mbed.h"
#include <stdint.h>

/**
 * @brief Single-pixel WS2812/WS2812B (NeoPixel) bit-bang driver.
 *
 * Drives one NeoPixel on any GPIO pin. Uses GPIO BSRR direct-register writes
 * and the DWT cycle counter for the timing-critical 800 kHz protocol.
 * Interrupts are briefly disabled during transmission (~30 µs per show()).
 *
 * Tested for STM32F446RE @ 180 MHz / WS2812B at 5 V.
 */
class NeoPixel {
public:
    /**
     * @brief Construct driver for a single NeoPixel on @p pin.
     * @param pin  Any STM32 GPIO pin wired to the NeoPixel's DIN line.
     */
    explicit NeoPixel(PinName pin);

    /**
     * @brief Buffer a new colour (not yet transmitted to the LED).
     * @param r  Red channel   (0..255)
     * @param g  Green channel (0..255)
     * @param b  Blue channel  (0..255)
     */
    void setRGB(uint8_t r, uint8_t g, uint8_t b);

    /** Immediately clear the LED (0,0,0) and transmit. */
    void clear();

    /** Transmit the buffered colour to the LED. */
    void show();

private:
    GPIO_TypeDef* m_port;
    uint32_t      m_set_bsrr;
    uint32_t      m_reset_bsrr;
    uint8_t       m_r;
    uint8_t       m_g;
    uint8_t       m_b;
};

#endif // NEOPIXEL_H_
