// CargoSweep — Hauptprogramm (Nucleo F446RE + PES Board)
#include "mbed.h"
#include "PESBoardPinMap.h"
#include "DebounceIn.h"

// === SELECT ACTIVE TEST (edit test_config.h) ===
// eierläcke
#include "test_config.h"

#if defined(TEST_SERVO_180)
    #include "../test/test_servo_180.h"
    #define TEST_INIT(lps)    servo_180_init(lps)
    #define TEST_TASK(led)    servo_180_task(led)
    #define TEST_RESET(led)   servo_180_reset(led)
    #define TEST_PRINT()      servo_180_print()

#elif defined(TEST_COLOR_DETECT)
    #include "../test/test_color_detect.h"
    #define TEST_INIT(lps)    color_detect_init(lps)
    #define TEST_TASK(led)    color_detect_task(led)
    #define TEST_RESET(led)   color_detect_reset(led)
    #define TEST_PRINT()      color_detect_print()

#elif defined(CALIB_COLOR_HUE)
    #include "../calibration/calib_color_hue.h"
    #define TEST_INIT(lps)    calib_color_hue_init(lps)
    #define TEST_TASK(led)    calib_color_hue_task(led)
    #define TEST_RESET(led)   calib_color_hue_reset(led)
    #define TEST_PRINT()      calib_color_hue_print()

#elif defined(TEST_LINEFOLLOWER)
    #include "../test/test_linefollower.h"
    #define TEST_INIT(lps)  linefollower_init(lps)
    #define TEST_TASK(led)  linefollower_task(led)
    #define TEST_RESET(led) linefollower_reset(led)
    #define TEST_PRINT()    linefollower_print()

#elif defined(TEST_LINEFOLLOWER_SLOW)
    #include "../test/test_linefollower_slow.h"
    #define TEST_INIT(lps)  linefollower_slow_init(lps)
    #define TEST_TASK(led)  linefollower_slow_task(led)
    #define TEST_RESET(led) linefollower_slow_reset(led)
    #define TEST_PRINT()    linefollower_slow_print()

#elif defined(TEST_LINEFOLLOWER_FAST)
    #include "../test/test_linefollower_fast.h"
    #define TEST_INIT(lps)  linefollower_fast_init(lps)
    #define TEST_TASK(led)  linefollower_fast_task(led)
    #define TEST_RESET(led) linefollower_fast_reset(led)
    #define TEST_PRINT()    linefollower_fast_print()

#elif defined(TEST_SERVO_COMBINED)
    #include "../test/test_servo_combined.h"
    #define TEST_INIT(lps)    servo_combined_init(lps)
    #define TEST_TASK(led)    servo_combined_task(led)
    #define TEST_RESET(led)   servo_combined_reset(led)
    #define TEST_PRINT()      servo_combined_print()

#elif defined(CALIB_SERVO_180)
    #include "../calibration/calib_servo_180.h"
    #define TEST_INIT(lps)    calib_servo_180_init(lps)
    #define TEST_TASK(led)    calib_servo_180_task(led)
    #define TEST_RESET(led)   calib_servo_180_reset(led)
    #define TEST_PRINT()      calib_servo_180_print()

#elif defined(TEST_IR_SENSOR)
    #include "../test/test_ir_sensor.h"
    #define TEST_INIT(lps)    ir_sensor_init(lps)
    #define TEST_TASK(led)    ir_sensor_task(led)
    #define TEST_RESET(led)   ir_sensor_reset(led)
    #define TEST_PRINT()      ir_sensor_print()

#elif defined(TEST_MOTOR_DRIVE)
    #include "../test/test_motor_drive.h"
    #define TEST_INIT(lps)    motor_drive_init(lps)
    #define TEST_TASK(led)    motor_drive_task(led)
    #define TEST_RESET(led)   motor_drive_reset(led)
    #define TEST_PRINT()      motor_drive_print()

#elif defined(TEST_SYSTEM_ALL)
    #include "../test/test_system_all.h"
    #define TEST_INIT(lps)    system_all_init(lps)
    #define TEST_TASK(led)    system_all_task(led)
    #define TEST_RESET(led)   system_all_reset(led)
    #define TEST_PRINT()      system_all_print()

#elif defined(TEST_LINEFOLLOWER_BWD)
    #include "../test/test_linefollower_bwd.h"
    #define TEST_INIT(lps)    linefollower_bwd_init(lps)
    #define TEST_TASK(led)    linefollower_bwd_task(led)
    #define TEST_RESET(led)   linefollower_bwd_reset(led)
    #define TEST_PRINT()      linefollower_bwd_print()

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

#elif defined(PROTOTYPE_03_V29)
    #include "prototype03_v29.h"
    #define TEST_INIT(lps)    roboter_v29_init(lps)
    #define TEST_TASK(led)    roboter_v29_task(led)
    #define TEST_RESET(led)   roboter_v29_reset(led)
    #define TEST_PRINT()      roboter_v29_print()

#elif defined(PROTOTYPE_03_V30_1)
    #include "prototype03_v30_1.h"
    #define TEST_INIT(lps)    roboter_v30_1_init(lps)
    #define TEST_TASK(led)    roboter_v30_1_task(led)
    #define TEST_RESET(led)   roboter_v30_1_reset(led)
    #define TEST_PRINT()      roboter_v30_1_print()
#elif defined(PROTOTYPE_03_V31_1)
    #include "prototype03_v31_1.h"
    #define TEST_INIT(lps)    roboter_v31_1_init(lps)
    #define TEST_TASK(led)    roboter_v31_1_task(led)
    #define TEST_RESET(led)   roboter_v31_1_reset(led)
    #define TEST_PRINT()      roboter_v31_1_print()

#elif defined(PROTOTYPE_03_V32_02)
    #include "prototype03_v32_02.h"
    #define TEST_INIT(lps)    roboter_v32_02_init(lps)
    #define TEST_TASK(led)    roboter_v32_02_task(led)
    #define TEST_RESET(led)   roboter_v32_02_reset(led)
    #define TEST_PRINT()      roboter_v32_02_print()

#elif defined(PROTOTYPE_03_V33_03)
    #include "prototype03_v33_03.h"
    #define TEST_INIT(lps)    roboter_v33_03_init(lps)
    #define TEST_TASK(led)    roboter_v33_03_task(led)
    #define TEST_RESET(led)   roboter_v33_03_reset(led)
    #define TEST_PRINT()      roboter_v33_03_print()

#elif defined(PROTOTYPE_03_V34_04_01)
    #include "prototype03_v34_04_01.h"
    #define TEST_INIT(lps)    roboter_v34_04_01_init(lps)
    #define TEST_TASK(led)    roboter_v34_04_01_task(led)
    #define TEST_RESET(led)   roboter_v34_04_01_reset(led)
    #define TEST_PRINT()      roboter_v34_04_01_print()

#elif defined(PROTOTYPE_03_V35_04_02)
    #include "prototype03_v35_04_02.h"
    #define TEST_INIT(lps)    roboter_v35_04_02_init(lps)
    #define TEST_TASK(led)    roboter_v35_04_02_task(led)
    #define TEST_RESET(led)   roboter_v35_04_02_reset(led)
    #define TEST_PRINT()      roboter_v35_04_02_print()

#elif defined(CARGOSWEEP_FINAL_VERSION_01)
    #include "cargosweep_final_version_01.h"
    #define TEST_INIT(lps)    cargosweep_final_v01_init(lps)
    #define TEST_TASK(led)    cargosweep_final_v01_task(led)
    #define TEST_RESET(led)   cargosweep_final_v01_reset(led)
    #define TEST_PRINT()      cargosweep_final_v01_print()

#elif defined(CARGOSWEEP_FINAL_VERSION_02)
    #include "cargosweep_final_version_02.h"
    #define TEST_INIT(lps)    cargosweep_final_v02_init(lps)
    #define TEST_TASK(led)    cargosweep_final_v02_task(led)
    #define TEST_RESET(led)   cargosweep_final_v02_reset(led)
    #define TEST_PRINT()      cargosweep_final_v02_print()

#elif defined(CARGOSWEEP_FINAL_VERSION_03)
    #include "cargosweep_final_version_03.h"
    #define TEST_INIT(lps)    cargosweep_final_v03_init(lps)
    #define TEST_TASK(led)    cargosweep_final_v03_task(led)
    #define TEST_RESET(led)   cargosweep_final_v03_reset(led)
    #define TEST_PRINT()      cargosweep_final_v03_print()

#elif defined(CARGOSWEEP_FINAL_VERSION_04)
    #include "cargosweep_final_version_04.h"
    #define TEST_INIT(lps)    cargosweep_final_v04_init(lps)
    #define TEST_TASK(led)    cargosweep_final_v04_task(led)
    #define TEST_RESET(led)   cargosweep_final_v04_reset(led)
    #define TEST_PRINT()      cargosweep_final_v04_print()

#elif defined(CARGOSWEEP_FINAL_VERSION_05)
    #include "cargosweep_final_version_05.h"
    #define TEST_INIT(lps)    cargosweep_final_v05_init(lps)
    #define TEST_TASK(led)    cargosweep_final_v05_task(led)
    #define TEST_RESET(led)   cargosweep_final_v05_reset(led)
    #define TEST_PRINT()      cargosweep_final_v05_print()

#elif defined(CARGOSWEEP_TEAM5_20260511)
    #include "cargosweep_team5_20260511.h"
    #define TEST_INIT(lps)    cargosweep_team5_20260511_init(lps)
    #define TEST_TASK(led)    cargosweep_team5_20260511_task(led)
    #define TEST_RESET(led)   cargosweep_team5_20260511_reset(led)
    #define TEST_PRINT()      cargosweep_team5_20260511_print()

#elif defined(CARGOSWEEP_TEAM5_20260512)
    #include "cargosweep_team5_20260512.h"
    #define TEST_INIT(lps)    cargosweep_team5_20260512_init(lps)
    #define TEST_TASK(led)    cargosweep_team5_20260512_task(led)
    #define TEST_RESET(led)   cargosweep_team5_20260512_reset(led)
    #define TEST_PRINT()      cargosweep_team5_20260512_print()

#elif defined(TEST_ARM_LOGIC)
    #include "../test/test_arm_logic.h"
    #define TEST_INIT(lps)    arm_logic_init(lps)
    #define TEST_TASK(led)    arm_logic_task(led)
    #define TEST_RESET(led)   arm_logic_reset(led)
    #define TEST_PRINT()      arm_logic_print()

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

#elif defined(TEST_SERVO_180_ANGLE)
    #include "../test/test_servo_180_angle.h"
    #define TEST_INIT(lps)    servo_180_angle_init(lps)
    #define TEST_TASK(led)    servo_180_angle_task(led)
    #define TEST_RESET(led)   servo_180_angle_reset(led)
    #define TEST_PRINT()      servo_180_angle_print()

#elif defined(TEST_ENDSCHALTER)
    #include "test_files/test_endschalter.h"
    #define TEST_INIT(lps)    endschalter_init(lps)
    #define TEST_TASK(led)    endschalter_task(led)
    #define TEST_RESET(led)   endschalter_reset(led)
    #define TEST_PRINT()      endschalter_print()

#elif defined(CALIB_SERVO_360)
    #include "../calibration/calib_servo_360.h"
    #define TEST_INIT(lps)    calib_servo_360_init(lps)
    #define TEST_TASK(led)    calib_servo_360_task(led)
    #define TEST_RESET(led)   calib_servo_360_reset(led)
    #define TEST_PRINT()      calib_servo_360_print()

#elif defined(TEST_NEOPIXEL_BASIC)
    #include "../test/test_neopixel_basic.h"
    #define TEST_INIT(lps)    neopixel_basic_init(lps)
    #define TEST_TASK(led)    neopixel_basic_task(led)
    #define TEST_RESET(led)   neopixel_basic_reset(led)
    #define TEST_PRINT()      neopixel_basic_print()

#elif defined(TEST_NEOPIXEL_COLOR)
    #include "../test/test_neopixel_color.h"
    #define TEST_INIT(lps)    neopixel_color_init(lps)
    #define TEST_TASK(led)    neopixel_color_task(led)
    #define TEST_RESET(led)   neopixel_color_reset(led)
    #define TEST_PRINT()      neopixel_color_print()

#else
    #error "No test selected. Open test_config.h and uncomment one #define."
#endif
// ================================================

#ifdef TEST_NEOPIXEL_COLOR
bool do_execute_main_task = true;   // Sensor-Tests laufen direkt ohne Button
#else
bool do_execute_main_task = false;
#endif
bool do_reset_all_once    = false;

DebounceIn user_button(BUTTON1);
Timer      button_guard_timer;
void toggle_do_execute_main_fcn();

int main()
{
    button_guard_timer.start();
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
    // Zusätzlicher Software-Guard gegen Bouncing beim Loslassen:
    // DebounceIn filtert nur 20ms beim Drücken; beim Release entstehen
    // weitere falling-edges, die sonst ein zweites Toggle auslösen würden.
    
    if (button_guard_timer.elapsed_time() < 200ms) return;
    button_guard_timer.reset();

    do_execute_main_task = !do_execute_main_task;
    do_reset_all_once = true;
}
