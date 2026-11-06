#ifndef YAW_FUSION_H
#define YAW_FUSION_H

#include <math.h>
#include <stdint.h>

/* Pure math, independent of MCU registers; CCW positive, rad/s. */
typedef struct {
    float encoder_rate;
    float agreement_sec;
    float encoder_weight;
} YawFusion_t;

static inline int32_t YawFusion_Delta16(uint16_t now, uint16_t previous)
{
    uint32_t delta = (uint16_t)(now - previous);
    return delta < 32768U ? (int32_t)delta : (int32_t)delta - 65536;
}

static inline int32_t YawFusion_Delta32(uint32_t now, uint32_t previous)
{
    uint32_t delta = now - previous;
    return delta <= INT32_MAX ? (int32_t)delta :
           -1 - (int32_t)(UINT32_MAX - delta);
}

static inline float YawFusion_Update(YawFusion_t *state, float gyro_rate,
                                     float encoder_rate, float dt)
{
    state->encoder_rate += (dt / (0.030f + dt)) *
                           (encoder_rate - state->encoder_rate);
    float error = fabsf(state->encoder_rate - gyro_rate);
    float limit = 0.08f + 0.20f * fabsf(gyro_rate);
    /* Wheels cannot establish body rotation when the gyro sees no turn.
     * Also reject opposite signs and large innovations immediately. */
    int agrees = fabsf(gyro_rate) >= 0.02f &&
                 state->encoder_rate * gyro_rate > 0.0f && error < limit;
    state->encoder_weight = 0.0f;
    if (agrees) {
        state->agreement_sec += dt;
        if (state->agreement_sec > 0.10f) {
            state->agreement_sec = 0.10f;
            state->encoder_weight = 0.15f * (1.0f - error / limit);
        }
    } else {
        state->agreement_sec = 0.0f;
    }
    return gyro_rate + state->encoder_weight * (state->encoder_rate - gyro_rate);
}

#endif
