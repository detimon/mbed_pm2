#ifndef SERVO_FEEDBACK_360_H_
#define SERVO_FEEDBACK_360_H_

#include "mbed.h"
#include "PwmIn.h"
#include "Servo.h"

/**
 * @brief Driver for the Parallax Feedback 360° High Speed Servo (#900-00360).
 *
 * Control (white wire): uses existing Servo class on pwm_pin (bit-bang, pin-flexible).
 *   speed -1.0 = full CW (1280 µs), 0.0 = stop (1500 µs), +1.0 = full CCW (1720 µs).
 * Feedback (yellow wire): ~910 Hz PWM signal on feedback_pin via PwmIn.
 *   Duty 2.9 % = 0°, duty 97.1 % = 359°.
 *
 * Call update() every 20 ms. The P-controller drives the servo to the target angle
 * and stops within TOLERANCE_DEG (2°). Minimum output speed is enforced so the
 * servo keeps moving through the mechanical dead band right up to the tolerance.
 */
class ServoFeedback360 {
public:
    /**
     * @brief Construct a ServoFeedback360.
     * @param pwm_pin      Servo control pin (white wire) — any digital pin.
     * @param feedback_pin InterruptIn-capable pin for feedback (yellow wire).
     * @param kp           P-gain (start 0.005; useful range 0.002–0.01).
     */
    ServoFeedback360(PinName pwm_pin, PinName feedback_pin,
                     float kp            = 0.002f,
                     float tolerance_deg = 5.0f,
                     float min_speed     = 0.03f,
                     float angle_offset  = 0.0f);

    /**
     * @brief Run one P-controller iteration. Call every 20 ms.
     *        Reads current angle from feedback, computes error, commands speed.
     */
    void update();

    /**
     * @brief Command absolute target angle.
     * @param target_deg Target in degrees; normalised to [0, 360).
     */
    void moveToAngle(float target_deg);

    /**
     * @brief Rotate relative to the current *target* angle.
     * @param delta_deg Positive = CCW, negative = CW.
     */
    void moveRelative(float delta_deg);

    /** @brief Current measured angle in degrees [0, 360). */
    float getCurrentAngle() const;

    /** @brief true when |error| < 2°. */
    bool isAtTarget() const;

    /** @brief Current target angle in degrees [0, 360). */
    float getTargetAngle() const;

    /** @brief Signed error in degrees (target − current, shortest path). */
    float getError() const;

    /** @brief Last commanded speed in [-1.0, +1.0]. */
    float getSpeed() const;

    /**
     * @brief true once at least one full feedback cycle has been measured.
     *        Wait for this before trusting getCurrentAngle().
     */
    bool isFeedbackValid();

    /** @brief Command speed 0 immediately without changing target. */
    void stop();

private:
    void setServoSpeed(float speed);

    Servo  m_servo;
    PwmIn  m_pwm_in;

    float m_kp;
    float m_target_deg;
    float m_current_deg;
    float m_error;
    float m_speed;

    // Parallax 360° servo calibrated pulse range (fraction of 20 ms period):
    //   1280 µs / 20000 µs = 0.064  (full CW)
    //   1720 µs / 20000 µs = 0.086  (full CCW)
    //   setPulseWidth(0.5) → 0.075 → 1500 µs = stop
    static constexpr float PULSE_CALIB_MIN = 0.064f;
    static constexpr float PULSE_CALIB_MAX = 0.086f;

    // Feedback duty-cycle range for 0°–359°
    static constexpr float DUTY_MIN = 0.029f;
    static constexpr float DUTY_MAX = 0.971f;

    // Controller tuning (set via constructor)
    float m_tolerance_deg;
    float m_min_speed;
    float m_angle_offset;
};

#endif // SERVO_FEEDBACK_360_H_
