#ifndef TEST_LINE_FOLLOWER_H_
#define TEST_LINE_FOLLOWER_H_

#include "mbed.h"

void line_follower_init(int loops_per_second);
void line_follower_task(DigitalOut& led);
void line_follower_reset(DigitalOut& led);
void line_follower_print();

#endif /* TEST_LINE_FOLLOWER_H_ */
