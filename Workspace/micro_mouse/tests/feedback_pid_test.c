#include "feedback_pid.h"
#include <math.h>

#ifdef _WIN32
int _fltused = 0;
__declspec(dllexport)
#endif
int run_pid_tests(void)
{
#define CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)
    PID_Controller_t pid = {.kp = 720.0f, .ki = 45.0f, .kd = 36.0f,
                             .out_limit = 550.0f, .int_limit = 0.5f};
    for (int i = 0; i < 2000; ++i)
        CHECK(FeedbackPID_Step(&pid, 10.0f, 0.0f, 0.0f, 0.005f) == 550.0f);
    CHECK(pid.integral == 0.0f);
    for (int i = 0; i < 2000; ++i)
        CHECK(FeedbackPID_Step(&pid, -10.0f, 0.0f, 0.0f, 0.005f) == -550.0f);
    CHECK(pid.integral == 0.0f);
    CHECK(FeedbackPID_Step(&pid, 0.0f, 1.0f, 0.0f, 0.005f) == -36.0f);
    CHECK(FeedbackPID_Step(&pid, 0.0f, 0.0f, 100.0f, 0.005f) == 100.0f);
    pid.integral = 0.2f;
    (void)FeedbackPID_Step(&pid, -0.1f, 0.0f, 0.0f, 0.005f);
    CHECK(pid.integral < 0.2f);
    /* Synthetic body model: yaw acceleration = 0.02*PWM - 2*yaw_rate.
     * Exercises both feedback signs; not a model of the actual robot. */
    for (int direction = -1; direction <= 1; direction += 2) {
        pid.integral = 0.0f;
        float angle = 0.0f, rate = 0.0f;
        float target = direction * 0.3f;
        for (int i = 0; i < 4000; ++i) {
            float command = FeedbackPID_Step(&pid, target - angle, rate, 0.0f, 0.005f);
            rate += (0.02f * command - 2.0f * rate) * 0.005f;
            angle += rate * 0.005f;
        }
        CHECK(fabsf(target - angle) < 0.03f);
        CHECK(fabsf(rate) < 0.01f);
    }
    return 0;
}

int main(void) { return run_pid_tests(); }
