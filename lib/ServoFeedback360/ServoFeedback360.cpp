#include "ServoFeedback360.h"
#include <cmath>

ServoFeedback360::ServoFeedback360(PinName pwm_pin, PinName feedback_pin,
                                   float kp, float tolerance_deg, float min_speed,
                                   float angle_offset)
    : m_servo(pwm_pin),
      m_pwm_in(feedback_pin),
      m_kp(kp),
      m_target_deg(0.0f),
      m_current_deg(0.0f),
      m_error(0.0f),
      m_speed(0.0f),
      m_tolerance_deg(tolerance_deg),
      m_min_speed(min_speed),
      m_angle_offset(angle_offset),
      m_direction_lock(0)
{
    m_servo.calibratePulseMinMax(PULSE_CALIB_MIN, PULSE_CALIB_MAX);
    // Enable immediately with stop pulse so the servo gets a defined signal.
    m_servo.enable(0.5f);
}

void ServoFeedback360::update()
{
    // Update angle from feedback only after at least one full cycle is measured.
    if (isFeedbackValid()) {
        float duty  = m_pwm_in.dutycycle();
        float angle = 360.0f - ((duty - DUTY_MIN) / (DUTY_MAX - DUTY_MIN) * 360.0f) - m_angle_offset;
        // Normalize to [0, 360)
        while (angle <    0.0f) angle += 360.0f;
        while (angle >= 360.0f) angle -= 360.0f;
        m_current_deg = angle;
    }

    // Error calculation — direction-locked or shortest-path.
    if (m_direction_lock == 1) {
        // CW only: use CW distance, stop within tolerance on either side
        float cw_dist = fmodf(m_target_deg - m_current_deg + 360.0f, 360.0f);
        if (cw_dist < m_tolerance_deg || cw_dist > 360.0f - m_tolerance_deg)
            m_error = 0.0f;
        else
            m_error = cw_dist;  // positive = CW
    } else if (m_direction_lock == -1) {
        // CCW only: use CCW distance, stop within tolerance on either side
        float ccw_dist = fmodf(m_current_deg - m_target_deg + 360.0f, 360.0f);
        if (ccw_dist < m_tolerance_deg || ccw_dist > 360.0f - m_tolerance_deg)
            m_error = 0.0f;
        else
            m_error = -ccw_dist;  // negative = CCW
    } else {
        // Shortest path (default)
        m_error = m_target_deg - m_current_deg;
        if (m_error >  180.0f) m_error -= 360.0f;
        if (m_error < -180.0f) m_error += 360.0f;
    }

    if (fabsf(m_error) < m_tolerance_deg) {
        m_speed = 0.0f;
    } else {
        m_speed = m_kp * m_error;
        if (m_speed >  1.0f) m_speed =  1.0f;
        if (m_speed < -1.0f) m_speed = -1.0f;
        // Clamp to minimum speed so the servo keeps moving right up to the tolerance
        // instead of stalling inside the mechanical dead band.
        if (m_speed > 0.0f && m_speed <  m_min_speed) m_speed =  m_min_speed;
        if (m_speed < 0.0f && m_speed > -m_min_speed) m_speed = -m_min_speed;
    }

    setServoSpeed(m_speed);
}

void ServoFeedback360::moveToAngle(float target_deg)
{
    target_deg = fmodf(target_deg, 360.0f);
    if (target_deg < 0.0f) target_deg += 360.0f;
    m_target_deg = target_deg;
}

void ServoFeedback360::moveRelative(float delta_deg)
{
    moveToAngle(m_target_deg + delta_deg);
}

float ServoFeedback360::getCurrentAngle() const { return m_current_deg; }
bool  ServoFeedback360::isAtTarget()      const { return fabsf(m_error) < m_tolerance_deg; }
float ServoFeedback360::getTargetAngle()  const { return m_target_deg; }
float ServoFeedback360::getError()        const { return m_error; }
float ServoFeedback360::getSpeed()        const { return m_speed; }

bool ServoFeedback360::isFeedbackValid()
{
    // period() returns 0 until the first rise-edge is seen; 910 Hz → ~1.1 ms.
    return m_pwm_in.period() > 0.0005f;
}

void ServoFeedback360::stop()
{
    m_speed = 0.0f;
    setServoSpeed(0.0f);
}

void ServoFeedback360::addSpeed(float extra)
{
    float s = m_speed + extra;
    if (s >  1.0f) s =  1.0f;
    if (s < -1.0f) s = -1.0f;
    setServoSpeed(s);
}

void ServoFeedback360::setDirectionLock(int dir)
{
    m_direction_lock = dir;
}

void ServoFeedback360::disable()
{
    m_speed = 0.0f;
    m_servo.disable();
}

void ServoFeedback360::enable(float pulse)
{
    m_servo.enable(pulse);
}

void ServoFeedback360::setServoSpeed(float speed)
{
    // Map speed [-1, +1] to normalized pulse [0, 1]:
    //   0.5 + 0.0*0.5 = 0.5  → 1500 µs (stop)
    //   0.5 - 1.0*0.5 = 0.0  → 1280 µs (full CW)
    //   0.5 + 1.0*0.5 = 1.0  → 1720 µs (full CCW)
    float pulse = 0.5f - speed * 0.5f;
    if (pulse < 0.0f) pulse = 0.0f;
    if (pulse > 1.0f) pulse = 1.0f;
    m_servo.setPulseWidth(pulse);
}
