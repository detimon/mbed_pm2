#ifndef TEST_LINE_FOLLOWER_SLOW_H_
#define TEST_LINE_FOLLOWER_SLOW_H_

#include "mbed.h"

void linefollower_slow_init(int loops_per_second);
void linefollower_slow_task(DigitalOut& led);
void linefollower_slow_reset(DigitalOut& led);
void linefollower_slow_print();

#endif /* TEST_LINE_FOLLOWER_SLOW_H_ */
