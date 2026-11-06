#include "yaw_fusion.h"

#ifdef _WIN32
/* Allows a freestanding Windows test DLL without a C runtime dependency. */
int _fltused = 0;
#endif

#define CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

#ifdef _WIN32
__declspec(dllexport)
#endif
int run_tests(void)
{
    CHECK(YawFusion_Delta16(3, 65530) == 9);
    CHECK(YawFusion_Delta16(65530, 3) == -9);
    CHECK(YawFusion_Delta16(32770, 32760) == 10);
    CHECK(YawFusion_Delta32(3, UINT32_MAX - 5) == 9);
    CHECK(YawFusion_Delta32(UINT32_MAX - 5, 3) == -9);
    YawFusion_t state = {0};
    float heading = 0.0f;
    /* 60 seconds of unequal wheel motion with a stationary body. */
    for (int i = 0; i < 12000; ++i) {
        heading += YawFusion_Update(&state, 0.0f, 2.0f, 0.005f) * 0.005f;
        CHECK(state.encoder_weight == 0.0f);
    }
    CHECK(heading == 0.0f);
    /* Correct turn: allow encoder only after sustained agreement. */
    state = (YawFusion_t){0};
    for (int i = 0; i < 400; ++i)
        CHECK(fabsf(YawFusion_Update(&state, 1.0f, 1.0f, 0.005f) - 1.0f) < 0.05f);
    CHECK(state.encoder_weight > 0.14f);
    /* Stopping the body must reject even previously trusted moving wheels. */
    CHECK(YawFusion_Update(&state, 0.0f, 1.0f, 0.005f) == 0.0f);
    CHECK(state.encoder_weight == 0.0f);
    state = (YawFusion_t){0};
    for (int i = 0; i < 400; ++i) {
        CHECK(YawFusion_Update(&state, -1.0f, 1.0f, 0.005f) == -1.0f);
        CHECK(state.encoder_weight == 0.0f);
    }
    /* Gross wheel slip during an actual body turn. */
    state = (YawFusion_t){0};
    for (int i = 0; i < 400; ++i)
        (void)YawFusion_Update(&state, 1.0f, 5.0f, 0.005f);
    CHECK(state.encoder_weight == 0.0f);
    /* Reset a segment without injecting stale encoder rate. */
    state = (YawFusion_t){0};
    CHECK(YawFusion_Update(&state, 0.0f, 0.0f, 0.005f) == 0.0f);
    return 0;
}

int main(void) { return run_tests(); }
