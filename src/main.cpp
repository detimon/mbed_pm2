// CargoSweep — Hauptprogramm (Nucleo F446RE + PES Board)
#include "mbed.h"
#include "PESBoardPinMap.h"
#include "DebounceIn.h"

// === SELECT ACTIVE TEST (edit test_config.h) ===

#include "test_config.h"

#if defined(TEST_SERVO)
    #include "test_files/test_servo.h"
    #define TEST_INIT(lps)    servo_init(lps)
    #define TEST_TASK(led)    servo_task(led)
    #define TEST_RESET(led)   servo_reset(led)
    #define TEST_PRINT()      servo_print()

#elif defined(TEST_COLOR_SENSOR)
    #include "test_files/test_color_sensor.h"
    #define TEST_INIT(lps)    color_sensor_init(lps)
    #define TEST_TASK(led)    color_sensor_task(led)
    #define TEST_RESET(led)   color_sensor_reset(led)
    #define TEST_PRINT()      color_sensor_print()

#elif defined(TEST_COLOR_SENSOR_CALIB)
    #include "test_files/test_color_sensor_calib.h"
    #define TEST_INIT(lps)    color_sensor_calib_init(lps)
    #define TEST_TASK(led)    color_sensor_calib_task(led)
    #define TEST_RESET(led)   color_sensor_calib_reset(led)
    #define TEST_PRINT()      color_sensor_calib_print()

#elif defined(TEST_LINE_FOLLOWER)
    #include "test_files/test_line_follower.h"
    #define TEST_INIT(lps)  line_follower_init(lps)
    #define TEST_TASK(led)  line_follower_task(led)
    #define TEST_RESET(led) line_follower_reset(led)
    #define TEST_PRINT()    line_follower_print()

#elif defined(TEST_LINE_FOLLOWER_SLOW)
    #include "test_files/test_line_follower_SLOW.h"
    #define TEST_INIT(lps)  line_follower_slow_init(lps)
    #define TEST_TASK(led)  line_follower_slow_task(led)
    #define TEST_RESET(led) line_follower_slow_reset(led)
    #define TEST_PRINT()    line_follower_slow_print()

#elif defined(TEST_LINE_FOLLOWER_FAST)
    #include "test_files/test_line_follower_FAST.h"
    #define TEST_INIT(lps)  line_follower_fast_init(lps)
    #define TEST_TASK(led)  line_follower_fast_task(led)
    #define TEST_RESET(led) line_follower_fast_reset(led)
    #define TEST_PRINT()    line_follower_fast_print()

#elif defined(TEST_SERVO_ALL)
    #include "test_files/test_servo_all.h"
    #define TEST_INIT(lps)    servo_all_init(lps)
    #define TEST_TASK(led)    servo_all_task(led)
    #define TEST_RESET(led)   servo_all_reset(led)
    #define TEST_PRINT()      servo_all_print()

#elif defined(TEST_SERVO_CALIB)
    #include "test_files/test_servo_calib.h"
    #define TEST_INIT(lps)    servo_calib_init(lps)
    #define TEST_TASK(led)    servo_calib_task(led)
    #define TEST_RESET(led)   servo_calib_reset(led)
    #define TEST_PRINT()      servo_calib_print()

#elif defined(TEST_IR)
    #include "test_files/test_ir.h"
    #define TEST_INIT(lps)    ir_init(lps)
    #define TEST_TASK(led)    ir_task(led)
    #define TEST_RESET(led)   ir_reset(led)
    #define TEST_PRINT()      ir_print()

#elif defined(TEST_DC_MOTOR)
    #include "test_files/test_dc_motor.h"
    #define TEST_INIT(lps)    dc_motor_init(lps)
    #define TEST_TASK(led)    dc_motor_task(led)
    #define TEST_RESET(led)   dc_motor_reset(led)
    #define TEST_PRINT()      dc_motor_print()

#elif defined(TEST_ALL)
    #include "test_files/test_all.h"
    #define TEST_INIT(lps)    all_init(lps)
    #define TEST_TASK(led)    all_task(led)
    #define TEST_RESET(led)   all_reset(led)
    #define TEST_PRINT()      all_print()

#elif defined(TEST_LINE_FOLLOWER_BACKWARD)
    #include "test_files/test_line_follower_BACKWARD.h"
    #define TEST_INIT(lps)    line_follower_backward_init(lps)
    #define TEST_TASK(led)    line_follower_backward_task(led)
    #define TEST_RESET(led)   line_follower_backward_reset(led)
    #define TEST_PRINT()      line_follower_backward_print()

#elif defined(PROTOTYPE_01_V1)
    #include "prototype01_v1.h"
    #define TEST_INIT(lps)    roboter_v1_init(lps)
    #define TEST_TASK(led)    roboter_v1_task(led)
    #define TEST_RESET(led)   roboter_v1_reset(led)
    #define TEST_PRINT()      roboter_v1_print()

#elif defined(PROTOTYPE_01_V2)
    #include "prototype01_v2.h"
    #define TEST_INIT(lps)    roboter_v2_init(lps)
    #define TEST_TASK(led)    roboter_v2_task(led)
    #define TEST_RESET(led)   roboter_v2_reset(led)
    #define TEST_PRINT()      roboter_v2_print()

#elif defined(PROTOTYPE_01_V3)
    #include "prototype01_v3.h"
    #define TEST_INIT(lps)    roboter_v3_init(lps)
    #define TEST_TASK(led)    roboter_v3_task(led)
    #define TEST_RESET(led)   roboter_v3_reset(led)
    #define TEST_PRINT()      roboter_v3_print()

#elif defined(PROTOTYPE_01_V4)
    #include "prototype01_v4.h"
    #define TEST_INIT(lps)    roboter_v4_init(lps)
    #define TEST_TASK(led)    roboter_v4_task(led)
    #define TEST_RESET(led)   roboter_v4_reset(led)
    #define TEST_PRINT()      roboter_v4_print()

#elif defined(PROTOTYPE_01_V5)
    #include "prototype01_v5.h"
    #define TEST_INIT(lps)    roboter_v5_init(lps)
    #define TEST_TASK(led)    roboter_v5_task(led)
    #define TEST_RESET(led)   roboter_v5_reset(led)
    #define TEST_PRINT()      roboter_v5_print()

#elif defined(PROTOTYPE_01_V6)
    #include "prototype01_v6.h"
    #define TEST_INIT(lps)    roboter_v6_init(lps)
    #define TEST_TASK(led)    roboter_v6_task(led)
    #define TEST_RESET(led)   roboter_v6_reset(led)
    #define TEST_PRINT()      roboter_v6_print()

#elif defined(PROTOTYPE_01_V7)
    #include "prototype01_v7.h"
    #define TEST_INIT(lps)    roboter_v7_init(lps)
    #define TEST_TASK(led)    roboter_v7_task(led)
    #define TEST_RESET(led)   roboter_v7_reset(led)
    #define TEST_PRINT()      roboter_v7_print()

#elif defined(PROTOTYPE_01_V8)
    #include "prototype01_v8.h"
    #define TEST_INIT(lps)    roboter_v8_init(lps)
    #define TEST_TASK(led)    roboter_v8_task(led)
    #define TEST_RESET(led)   roboter_v8_reset(led)
    #define TEST_PRINT()      roboter_v8_print()

#elif defined(PROTOTYPE_02_V9)
    #include "prototype02_v9.h"
    #define TEST_INIT(lps)    roboter_v9_init(lps)
    #define TEST_TASK(led)    roboter_v9_task(led)
    #define TEST_RESET(led)   roboter_v9_reset(led)
    #define TEST_PRINT()      roboter_v9_print()

#elif defined(PROTOTYPE_02_V10)
    #include "prototype02_v10.h"
    #define TEST_INIT(lps)    roboter_v10_init(lps)
    #define TEST_TASK(led)    roboter_v10_task(led)
    #define TEST_RESET(led)   roboter_v10_reset(led)
    #define TEST_PRINT()      roboter_v10_print()

#elif defined(PROTOTYPE_02_V11)
    #include "prototype02_v11.h"
    #define TEST_INIT(lps)    roboter_v11_init(lps)
    #define TEST_TASK(led)    roboter_v11_task(led)
    #define TEST_RESET(led)   roboter_v11_reset(led)
    #define TEST_PRINT()      roboter_v11_print()

#elif defined(PROTOTYPE_02_V12)
    #include "prototype02_v12.h"
    #define TEST_INIT(lps)    roboter_v12_init(lps)
    #define TEST_TASK(led)    roboter_v12_task(led)
    #define TEST_RESET(led)   roboter_v12_reset(led)
    #define TEST_PRINT()      roboter_v12_print()

#elif defined(PROTOTYPE_02_V13)
    #include "prototype02_v13.h"
    #define TEST_INIT(lps)    roboter_v13_init(lps)
    #define TEST_TASK(led)    roboter_v13_task(led)
    #define TEST_RESET(led)   roboter_v13_reset(led)
    #define TEST_PRINT()      roboter_v13_print()

#elif defined(PROTOTYPE_02_V14)
    #include "prototype02_v14.h"
    #define TEST_INIT(lps)    roboter_v14_init(lps)
    #define TEST_TASK(led)    roboter_v14_task(led)
    #define TEST_RESET(led)   roboter_v14_reset(led)
    #define TEST_PRINT()      roboter_v14_print()

#elif defined(PROTOTYPE_02_V15)
    #include "prototype02_v15.h"
    #define TEST_INIT(lps)    roboter_v15_init(lps)
    #define TEST_TASK(led)    roboter_v15_task(led)
    #define TEST_RESET(led)   roboter_v15_reset(led)
    #define TEST_PRINT()      roboter_v15_print()

#elif defined(PROTOTYPE_02_V16)
    #include "prototype02_v16.h"
    #define TEST_INIT(lps)    roboter_v16_init(lps)
    #define TEST_TASK(led)    roboter_v16_task(led)
    #define TEST_RESET(led)   roboter_v16_reset(led)
    #define TEST_PRINT()      roboter_v16_print()

#elif defined(PROTOTYPE_02_V17)
    #include "prototype02_v17.h"
    #define TEST_INIT(lps)    roboter_v17_init(lps)
    #define TEST_TASK(led)    roboter_v17_task(led)
    #define TEST_RESET(led)   roboter_v17_reset(led)
    #define TEST_PRINT()      roboter_v17_print()

#elif defined(PROTOTYPE_02_V18)
    #include "prototype02_v18.h"
    #define TEST_INIT(lps)    roboter_v18_init(lps)
    #define TEST_TASK(led)    roboter_v18_task(led)
    #define TEST_RESET(led)   roboter_v18_reset(led)
    #define TEST_PRINT()      roboter_v18_print()

#elif defined(PROTOTYPE_02_V19)
    #include "prototype02_v19.h"
    #define TEST_INIT(lps)    roboter_v19_init(lps)
    #define TEST_TASK(led)    roboter_v19_task(led)
    #define TEST_RESET(led)   roboter_v19_reset(led)
    #define TEST_PRINT()      roboter_v19_print()

#elif defined(PROTOTYPE_02_V20)
    #include "prototype02_v20.h"
    #define TEST_INIT(lps)    roboter_v20_init(lps)
    #define TEST_TASK(led)    roboter_v20_task(led)
    #define TEST_RESET(led)   roboter_v20_reset(led)
    #define TEST_PRINT()      roboter_v20_print()

#elif defined(PROTOTYPE_02_V21)
    #include "prototype02_v21.h"
    #define TEST_INIT(lps)    roboter_v21_init(lps)
    #define TEST_TASK(led)    roboter_v21_task(led)
    #define TEST_RESET(led)   roboter_v21_reset(led)
    #define TEST_PRINT()      roboter_v21_print()

#elif defined(PROTOTYPE_02_V22)
    #include "prototype02_v22.h"
    #define TEST_INIT(lps)    roboter_v22_init(lps)
    #define TEST_TASK(led)    roboter_v22_task(led)
    #define TEST_RESET(led)   roboter_v22_reset(led)
    #define TEST_PRINT()      roboter_v22_print()

#elif defined(PROTOTYPE_02_V23)
    #include "prototype02_v23.h"
    #define TEST_INIT(lps)    roboter_v23_init(lps)
    #define TEST_TASK(led)    roboter_v23_task(led)
    #define TEST_RESET(led)   roboter_v23_reset(led)
    #define TEST_PRINT()      roboter_v23_print()

#elif defined(PROTOTYPE_03_V24)
    #include "prototype03_v24.h"
    #define TEST_INIT(lps)    roboter_v24_init(lps)
    #define TEST_TASK(led)    roboter_v24_task(led)
    #define TEST_RESET(led)   roboter_v24_reset(led)
    #define TEST_PRINT()      roboter_v24_print()

#elif defined(PROTOTYPE_03_V25)
    #include "prototype03_v25.h"
    #define TEST_INIT(lps)    roboter_v25_init(lps)
    #define TEST_TASK(led)    roboter_v25_task(led)
    #define TEST_RESET(led)   roboter_v25_reset(led)
    #define TEST_PRINT()      roboter_v25_print()

#elif defined(PROTOTYPE_03_V26)
    #include "prototype03_v26.h"
    #define TEST_INIT(lps)    roboter_v26_init(lps)
    #define TEST_TASK(led)    roboter_v26_task(led)
    #define TEST_RESET(led)   roboter_v26_reset(led)
    #define TEST_PRINT()      roboter_v26_print()

#elif defined(PROTOTYPE_03_V23_2)
    #include "prototype03_v23_2.h"
    #define TEST_INIT(lps)    roboter_v23_2_init(lps)
    #define TEST_TASK(led)    roboter_v23_2_task(led)
    #define TEST_RESET(led)   roboter_v23_2_reset(led)
    #define TEST_PRINT()      roboter_v23_2_print()

#elif defined(PROTOTYPE_03_V27)
    #include "prototype03_v27.h"
    #define TEST_INIT(lps)    roboter_v27_init(lps)
    #define TEST_TASK(led)    roboter_v27_task(led)
    #define TEST_RESET(led)   roboter_v27_reset(led)
    #define TEST_PRINT()      roboter_v27_print()

#elif defined(PROTOTYPE_03_V28)
    #include "prototype03_v28.h"
    #define TEST_INIT(lps)    roboter_v28_init(lps)
    #define TEST_TASK(led)    roboter_v28_task(led)
    #define TEST_RESET(led)   roboter_v28_reset(led)
    #define TEST_PRINT()      roboter_v28_print()

#elif defined(TEST_LOGIC_ARM_STANDARD)
    #include "test_files/test_logic_arm_standard.h"
    #define TEST_INIT(lps)    logic_arm_standard_init(lps)
    #define TEST_TASK(led)    logic_arm_standard_task(led)
    #define TEST_RESET(led)   logic_arm_standard_reset(led)
    #define TEST_PRINT()      logic_arm_standard_print()

#elif defined(TEST_ARM_SEQUENCE)
    // Single-File-Test: Forward-Deklarationen (kein .h)
    void test_arm_sequence_init(int);
    void test_arm_sequence_task(DigitalOut&);
    void test_arm_sequence_reset(DigitalOut&);
    void test_arm_sequence_print();
    #define TEST_INIT(lps)    test_arm_sequence_init(lps)
    #define TEST_TASK(led)    test_arm_sequence_task(led)
    #define TEST_RESET(led)   test_arm_sequence_reset(led)
    #define TEST_PRINT()      test_arm_sequence_print()

#elif defined(TEST_DREHSERVO_90)
    #include "test_files/test_drehservo_90.h"
    #define TEST_INIT(lps)    drehservo_90_init(lps)
    #define TEST_TASK(led)    drehservo_90_task(led)
    #define TEST_RESET(led)   drehservo_90_reset(led)
    #define TEST_PRINT()      drehservo_90_print()

#elif defined(TEST_ENDSCHALTER)
    #include "test_files/test_endschalter.h"
    #define TEST_INIT(lps)    endschalter_init(lps)
    #define TEST_TASK(led)    endschalter_task(led)
    #define TEST_RESET(led)   endschalter_reset(led)
    #define TEST_PRINT()      endschalter_print()

#elif defined(TEST_PARALLAX_360)
    #include "test_files/test_parallax_360.h"
    #define TEST_INIT(lps)    parallax_360_init(lps)
    #define TEST_TASK(led)    parallax_360_task(led)
    #define TEST_RESET(led)   parallax_360_reset(led)
    #define TEST_PRINT()      parallax_360_print()

#elif defined(TEST_NEOPIXEL)
    #include "test_files/test_neopixel.h"
    #define TEST_INIT(lps)    neopixel_init(lps)
    #define TEST_TASK(led)    neopixel_task(led)
    #define TEST_RESET(led)   neopixel_reset(led)
    #define TEST_PRINT()      neopixel_print()

#elif defined(TEST_COLOR_NEOPIXEL)
    #include "test_files/test_color_neopixel.h"
    #define TEST_INIT(lps)    color_neopixel_init(lps)
    #define TEST_TASK(led)    color_neopixel_task(led)
    #define TEST_RESET(led)   color_neopixel_reset(led)
    #define TEST_PRINT()      color_neopixel_print()

#else
    #error "No test selected. Open test_config.h and uncomment one #define."
#endif
// ================================================

#ifdef TEST_COLOR_NEOPIXEL
bool do_execute_main_task = true;   // Sensor-Tests laufen direkt ohne Button
#else
bool do_execute_main_task = false;
#endif
bool do_reset_all_once    = false;

DebounceIn user_button(BUTTON1);
void toggle_do_execute_main_fcn();

int main()
{
    user_button.fall(&toggle_do_execute_main_fcn);

    const int main_task_period_ms = 20;
    Timer main_task_timer;

    DigitalOut user_led(LED1);

    const int loops_per_second = static_cast<int>(
        ceilf(1.0f / (0.001f * static_cast<float>(main_task_period_ms))));

    TEST_INIT(loops_per_second);

    main_task_timer.start();

    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {
            TEST_TASK(user_led);
        } else {
            if (do_reset_all_once) {
                do_reset_all_once = false;
                TEST_RESET(user_led);
            }
            user_led = !user_led;   // Blinken nur im Idle (Task verwaltet LED selbst)
        }

        TEST_PRINT();

        int elapsed_ms = duration_cast<milliseconds>(
            main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - elapsed_ms < 0)
            printf("Warning: Main task took longer than main_task_period_ms\n");
        else
            thread_sleep_for(main_task_period_ms - elapsed_ms);
    }
}

void toggle_do_execute_main_fcn()
{
    do_execute_main_task = !do_execute_main_task;
    do_reset_all_once = true;
}
