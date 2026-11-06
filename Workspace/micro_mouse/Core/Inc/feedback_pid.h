#ifndef FEEDBACK_PID_H
#define FEEDBACK_PID_H

typedef struct {
    float kp, ki, kd, kff;
    float integral;
    float prev_measurement;
    float out_limit;
    float int_limit;
} PID_Controller_t;

static inline float FeedbackPID_Clamp(float value, float limit)
{
    return value > limit ? limit : (value < -limit ? -limit : value);
}

/* D on measured velocity avoids a derivative kick when the angle target
 * changes. For yaw, measured_rate is the fused angular velocity in rad/s.
 * Conditional integration prevents windup at the controller output limit. */
static inline float FeedbackPID_Step(PID_Controller_t *pid, float error,
                                     float measured_rate, float feedforward,
                                     float dt)
{
    float candidate = FeedbackPID_Clamp(pid->integral + error * dt, pid->int_limit);
    float pd_ff = pid->kp * error - pid->kd * measured_rate + feedforward;
    float proposed = pd_ff + pid->ki * candidate;
    if (!((proposed > pid->out_limit && error > 0.0f) ||
          (proposed < -pid->out_limit && error < 0.0f))) {
        pid->integral = candidate;
    }
    return FeedbackPID_Clamp(pd_ff + pid->ki * pid->integral, pid->out_limit);
}

#endif
