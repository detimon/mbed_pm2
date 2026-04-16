#include "NeoPixel.h"

// WS2812B timing targets @ 180 MHz (5.56 ns per cycle)
//   Full bit period : 1.25 µs = 225 cycles
//   T1H (1-bit high): ~700 ns = 126 cycles
//   T0H (0-bit high): ~350 ns =  63 cycles
//   Reset latch     : > 50 µs low
//
// Values are slightly trimmed to account for the BSRR store and
// `DWT->CYCCNT` read overhead inside the busy-wait.
static const uint32_t BIT_PERIOD_CY = 220;
static const uint32_t T1H_CY        = 122;
static const uint32_t T0H_CY        = 58;

NeoPixel::NeoPixel(PinName pin)
    : m_port(nullptr), m_set_bsrr(0), m_reset_bsrr(0), m_r(0), m_g(0), m_b(0)
{
    const uint32_t port_idx = (static_cast<uint32_t>(pin) >> 4) & 0xF;
    const uint32_t pin_idx  =  static_cast<uint32_t>(pin)       & 0xF;

    GPIO_TypeDef* const gpios[] = {
        GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH
    };
    m_port       = gpios[port_idx];
    m_set_bsrr   = (1UL << pin_idx);
    m_reset_bsrr = (1UL << (pin_idx + 16));

    RCC->AHB1ENR |= (1UL << port_idx);

    uint32_t moder = m_port->MODER;
    moder &= ~(0x3UL << (pin_idx * 2));
    moder |=  (0x1UL << (pin_idx * 2));   // general-purpose output
    m_port->MODER = moder;

    uint32_t otyper = m_port->OTYPER;
    otyper &= ~(0x1UL << pin_idx);        // push-pull
    m_port->OTYPER = otyper;

    uint32_t ospeedr = m_port->OSPEEDR;
    ospeedr |= (0x3UL << (pin_idx * 2));  // very high speed
    m_port->OSPEEDR = ospeedr;

    m_port->BSRR = m_reset_bsrr;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    wait_us(80);
}

void NeoPixel::setRGB(uint8_t r, uint8_t g, uint8_t b)
{
    m_r = r;
    m_g = g;
    m_b = b;
}

void NeoPixel::clear()
{
    setRGB(0, 0, 0);
    show();
}

void NeoPixel::show()
{
    const uint8_t bytes[3] = {m_g, m_r, m_b}; // WS2812 expects GRB order
    GPIO_TypeDef* const port      = m_port;
    const uint32_t      set_mask  = m_set_bsrr;
    const uint32_t      rst_mask  = m_reset_bsrr;

    core_util_critical_section_enter();

    for (int byte_idx = 0; byte_idx < 3; byte_idx++) {
        uint8_t byte = bytes[byte_idx];
        for (int bit = 7; bit >= 0; bit--) {
            const uint32_t start = DWT->CYCCNT;
            port->BSRR = set_mask;
            if (byte & (1u << bit)) {
                while ((DWT->CYCCNT - start) < T1H_CY) { }
                port->BSRR = rst_mask;
            } else {
                while ((DWT->CYCCNT - start) < T0H_CY) { }
                port->BSRR = rst_mask;
            }
            while ((DWT->CYCCNT - start) < BIT_PERIOD_CY) { }
        }
    }

    core_util_critical_section_exit();

    wait_us(80); // latch
}
