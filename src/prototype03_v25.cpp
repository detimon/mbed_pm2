// CargoSweep — PROTOTYPE_03_V25
#include "test_config.h"

#ifdef PROTOTYPE_03_V25

#include "prototype03_v25.h"

void roboter_v25_init(int loops_per_second)
{
}

void roboter_v25_task(DigitalOut& led)
{
    led = !led;
}

void roboter_v25_reset(DigitalOut& led)
{
    led = 0;
}

void roboter_v25_print()
{
    printf("v25\n");
}

#endif // PROTOTYPE_03_V25
