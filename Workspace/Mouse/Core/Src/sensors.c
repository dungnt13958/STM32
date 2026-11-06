#include "sensors.h"
#include <math.h>
#include <stdio.h>
/* Provide these ADC adapters in the STM32 application; they return raw readings.
   Do not include the C++ Arduino adc.h here. */
extern int adc_get_L(void);
extern int adc_get_FL(void);
extern int adc_get_FR(void);
extern int adc_get_R(void);
extern void HAL_Delay(uint32_t ms);
extern void blinkLedsFor(uint32_t ms);

float L_RAW_CENTER = 500.0f;
float R_RAW_CENTER = 500.0f;
const uint8_t NO_START = 0;
const uint8_t LEFT_START = 1;
const uint8_t RIGHT_START = 2;
Sensors sensors;

void sensors_init(Sensors *self) {
    *self = (Sensors){0};
    self->m_active = true;
    self->g_steering_mode = STEER_NORMAL;
    self->dyn_left_scale = LEFT_SCALE;
    self->dyn_right_scale = RIGHT_SCALE;
}
static float sensors_constrain(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

int get_front_sum(Sensors *self) {
        return (int)(self->m_front_sum);
    }

int get_front_diff(Sensors *self) {
        return (int)(self->m_front_diff);
    }

float get_steering_feedback(Sensors *self) {
        return self->m_steering_adjustment;
    }

float get_cross_track_error(Sensors *self) {
        return self->m_cross_track_error;
    }

void enable(Sensors *self) {
        self->m_active = true;
    }

void disable(Sensors *self) {
        self->m_active = false;
        reset_control_state(self);
    }

void set_steering_mode(Sensors *self, uint8_t mode) {
        self->g_steering_mode = mode;
        reset_control_state(self);
    }

void reset_navigation_state(Sensors *self) {
        self->see_left_wall =
            self->lss.value >= LEFT_THRESHOLD;

        self->see_right_wall =
            self->rss.value >= RIGHT_THRESHOLD;

        self->see_front_wall =
            self->m_front_sum >= FRONT_THRESHOLD;

        self->m_left_wall_transition_count = 0;
        self->m_right_wall_transition_count = 0;
        self->m_front_wall_transition_count = 0;

        reset_control_state(self);
    }

void calibrate_scales(Sensors *self) {
        long sum_l = 0;
        long sum_r = 0;
        const int NUM_SAMPLES = 100;

        printf("Calib sensors .....\n");

        for (int i = 0; i < NUM_SAMPLES; ++i) {
            sum_l += adc_get_L();
            sum_r += adc_get_R();
            HAL_Delay(5);
        }

        const int avg_l =
            (int)(sum_l / NUM_SAMPLES);

        const int avg_r =
            (int)(sum_r / NUM_SAMPLES);

        if (avg_l > 10) {
            self->dyn_left_scale =
                (float)(SIDE_NOMINAL) /
                (float)(avg_l);
        }

        if (avg_r > 10) {
            self->dyn_right_scale =
                (float)(SIDE_NOMINAL) /
                (float)(avg_r);
        }

        printf("Calib L_Scale: %.4f | R_Scale: %.4f\n", (double)self->dyn_left_scale, (double)self->dyn_right_scale);

        reset_navigation_state(self);
    }

void update(Sensors *self) {
        if (!self->m_active) {
            reset_control_state(self);
            return;
        }

        self->lss.raw = adc_get_L();
        self->lfs.raw = adc_get_FL();
        self->rfs.raw = adc_get_FR();
        self->rss.raw = adc_get_R();

        self->lss.value =
            (int)(
                self->lss.raw * self->dyn_left_scale
            );

        self->lfs.value =
            (int)(
                self->lfs.raw * FRONT_LEFT_SCALE
            );

        self->rfs.value =
            (int)(
                self->rfs.raw * FRONT_RIGHT_SCALE
            );

        self->rss.value =
            (int)(
                self->rss.raw * self->dyn_right_scale
            );

        self->m_front_sum =
            self->lfs.value +
            self->rfs.value;

        self->m_front_diff =
            self->lfs.value -
            self->rfs.value;

        update_wall_latch(self, 
            self->lss.value,
            LEFT_THRESHOLD,
            SIDE_WALL_HYSTERESIS,
            &self->see_left_wall,
            &self->m_left_wall_transition_count
        );

        update_wall_latch(self, 
            self->rss.value,
            RIGHT_THRESHOLD,
            SIDE_WALL_HYSTERESIS,
            &self->see_right_wall,
            &self->m_right_wall_transition_count
        );

        update_wall_latch(self, 
            self->m_front_sum,
            FRONT_THRESHOLD,
            FRONT_WALL_HYSTERESIS,
            &self->see_front_wall,
            &self->m_front_wall_transition_count
        );

        if (self->g_steering_mode == STEERING_OFF) {
            reset_control_state(self);
            return;
        }

        const float left_error =
            SIDE_NOMINAL -
            (float)(self->lss.value);

        const float right_error =
            SIDE_NOMINAL -
            (float)(self->rss.value);

        float raw_error = 0.0f;

        if (self->g_steering_mode == STEER_NORMAL) {
            if (self->see_left_wall &&
                self->see_right_wall) {
                raw_error =
                    (float)(
                        self->rss.value -
                        self->lss.value
                    );
            } else if (self->see_left_wall) {
                raw_error =
                    STEERING_SINGLE_WALL_GAIN *
                    left_error;
            } else if (self->see_right_wall) {
                raw_error =
                    -STEERING_SINGLE_WALL_GAIN *
                    right_error;
            }
        } else if (
            self->g_steering_mode ==
            STEER_LEFT_WALL) {
            raw_error =
                STEERING_SINGLE_WALL_GAIN *
                left_error;
        } else if (
            self->g_steering_mode ==
            STEER_RIGHT_WALL) {
            raw_error =
                -STEERING_SINGLE_WALL_GAIN *
                right_error;
        }

        // In a front-wall corner the side values are not reliable. Disable the
        // correction immediately and clear all residual filter/D history.
        if (self->m_front_sum >=
            STEERING_FRONT_SUPPRESS_THRESHOLD) {
            reset_control_state(self);
            return;
        }

        // Requested safety rule:
        // a very large error is considered an invalid optical measurement,
        // not a request for a very large steering correction.
        if (fabsf(raw_error) >
            STEERING_MAX_VALID_ABS_ERROR) {
            reset_control_state(self);
            return;
        }

        // No valid side wall means no side correction.
        const bool has_valid_wall_reference =
            self->see_left_wall ||
            self->see_right_wall ||
            self->g_steering_mode ==
                STEER_LEFT_WALL ||
            self->g_steering_mode ==
                STEER_RIGHT_WALL;

        if (!has_valid_wall_reference) {
            reset_control_state(self);
            return;
        }

        self->m_filtered_steering_error +=
            STEERING_ERROR_FILTER_ALPHA *
            (
                raw_error -
                self->m_filtered_steering_error
            );

        self->m_cross_track_error =
            self->m_filtered_steering_error;

        calculate_steering_adjustment(self);
    }

bool occluded_left(Sensors *self) {
        return
            self->lfs.raw > 200 &&
            self->rfs.raw < 200;
    }

bool occluded_right(Sensors *self) {
        return
            self->lfs.raw < 200 &&
            self->rfs.raw > 200;
    }

uint16_t wait_for_user_start(Sensors *self) {
        int state = 0;
        enable(self);

        uint16_t choice = NO_START;

        while (choice == NO_START) {
            int count = 0;

            while (occluded_left(self)) {
                ++count;
                HAL_Delay(20);
            }

            if (count > 5) {
                choice = LEFT_START;
                break;
            }

            count = 0;

            while (occluded_right(self)) {
                ++count;
                HAL_Delay(20);
            }

            if (count > 5) {
                choice = RIGHT_START;
                break;
            }

            state = 1 - state;
        }

        (void)state;
        blinkLedsFor(500);
        return choice;
    }

void print_values(Sensors *self) {
    printf("L_val: %d FL_val: %d R_val: %d FR_val: %d | Err: %.1f | Steer: %.2f\n",
        self->lss.value, self->lfs.value, self->rss.value, self->rfs.value,
        (double)self->m_cross_track_error, (double)self->m_steering_adjustment);
}

void reset_control_state(Sensors *self) {
        self->m_cross_track_error = 0.0f;
        self->m_filtered_steering_error = 0.0f;
        self->m_last_steering_error = 0.0f;
        self->m_steering_adjustment = 0.0f;
    }

void update_wall_latch(Sensors *self, int value, int threshold, int hysteresis, volatile bool *state, uint8_t *transition_count) {
    (void)self;
        const int on_threshold =
            threshold +
            hysteresis;

        const int off_threshold =
            threshold -
            hysteresis;

        const bool requested_state =
            (*state)
            ? value > off_threshold
            : value > on_threshold;

        if (requested_state == (*state)) {
            (*transition_count) = 0;
            return;
        }

        if ((*transition_count) <
            WALL_DETECT_CONFIRM_TICKS) {
            ++(*transition_count);
        }

        if ((*transition_count) >=
            WALL_DETECT_CONFIRM_TICKS) {
            (*state) = requested_state;
            (*transition_count) = 0;
        }
    }

float calculate_steering_adjustment(Sensors *self) {
        const float p_term =
            STEERING_KP *
            self->m_cross_track_error;

        const float d_term =
            STEERING_KD *
            (
                self->m_cross_track_error -
                self->m_last_steering_error
            );

        float target_adjustment =
            p_term +
            d_term;

        target_adjustment =
            sensors_constrain(
                target_adjustment,
                -STEERING_ADJUST_LIMIT,
                STEERING_ADJUST_LIMIT
            );

        float change =
            target_adjustment -
            self->m_steering_adjustment;

        change =
            sensors_constrain(
                change,
                -STEERING_OUTPUT_SLEW_PER_TICK,
                STEERING_OUTPUT_SLEW_PER_TICK
            );

        self->m_steering_adjustment += change;

        self->m_last_steering_error =
            self->m_cross_track_error;

        return self->m_steering_adjustment;
    }
