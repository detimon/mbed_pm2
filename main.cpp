#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "Servo.h"

bool do_execute_main_task = false;
bool do_reset_all_once = false;

DebounceIn user_button(BUTTON1);
void toggle_do_execute_main_fcn();

int main()
{
    user_button.fall(&toggle_do_execute_main_fcn);

    const int main_task_period_ms = 20;
    Timer main_task_timer;

    DigitalOut user_led(LED1);
    DigitalOut led1(PB_9);

    // --- adding variables and objects and applying functions starts here ---

    // servo an Pin D1
    Servo servo_D1(PB_D1);

    // Kalibrierungswerte für FS90 (Startwerte - müssen noch angepasst werden!)
    float servo_ang_min = 0.050f;
    float servo_ang_max = 0.1050f;
    servo_D1.calibratePulseMinMax(servo_ang_min, servo_ang_max);

    // sanfte Bewegung
    servo_D1.setMaxAcceleration(0.3f);

    // Variablen für die Servo-Bewegung
    float servo_input = 0.0f;
    int servo_counter = 0;
    const int loops_per_seconds = static_cast<int>(
        ceilf(1.0f / (0.001f * static_cast<float>(main_task_period_ms))));

    // start timer
    main_task_timer.start();

    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {

            led1 = 1;

            // Servo einschalten
            if (!servo_D1.isEnabled())
                servo_D1.enable();

            // Servo bewegen
            servo_D1.setPulseWidth(servo_input);

            // jede Sekunde um 0.005 erhöhen (langsam von 0.0 bis 1.0)
            if ((servo_input < 1.0f) &&
                (servo_counter % loops_per_seconds == 0) &&
                (servo_counter != 0))
                servo_input += 0.005f;
            servo_counter++;

        } else {
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // reset
                led1 = 0;
                servo_D1.disable();
                servo_input = 0.0f;
                servo_counter = 0;
            }
        }

        user_led = !user_led;

        // Pulsewidth ausgeben im Serial Monitor
        printf("Servo input: %.4f\n", servo_input);

        int main_task_elapsed_time_ms = duration_cast<milliseconds>(
            main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - main_task_elapsed_time_ms < 0)
            printf("Warning: Main task took longer than main_task_period_ms\n");
        else
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void toggle_do_execute_main_fcn()
{
    do_execute_main_task = !do_execute_main_task;
    if (do_execute_main_task)
        do_reset_all_once = true;
}
// asdf
// blablabla

