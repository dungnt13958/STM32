
#include "ir_sensor.h"
#include "adc.h"
#include "stm32f4xx_hal.h"
#include <math.h>

/* Cấu hình mặc định; có thể ghi đè trong config.h */
#ifndef STEERING_KP
#define STEERING_KP 1.0f
#endif

#ifndef STEERING_KD
#define STEERING_KD 0.0f
#endif

#ifndef STEERING_ADJUST_LIMIT
#define STEERING_ADJUST_LIMIT 100.0f
#endif

#ifndef WALL_TARGET_L
#define WALL_TARGET_L 45.0f
#endif

#ifndef WALL_TARGET_R
#define WALL_TARGET_R 45.0f
#endif

#ifndef FRONT_REFERENCE
#define FRONT_REFERENCE 80.0f
#endif

#ifndef LEFT_THRESHOLD
#define LEFT_THRESHOLD 80
#endif

#ifndef RIGHT_THRESHOLD
#define RIGHT_THRESHOLD 80
#endif

#ifndef FRONT_THRESHOLD
#define FRONT_THRESHOLD 80
#endif

#ifndef FRONT_WALL_RELIABILITY_LIMIT
#define FRONT_WALL_RELIABILITY_LIMIT 100
#endif

#define CLAMP_FLOAT(x, lo, hi) \
    ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

SensorData sensor =
{
    .lfs_val = 0,
    .lss_val = 0,
    .rss_val = 0,
    .rfs_val = 0,

    .SeeFrontWall = false,
    .SeeLeftWall = false,
    .SeeRightWall = false,

    .LastSteeringError = 0.0f,
    .Active = false,
    .CrossTrackError = 0.0f,
    .SteeringAdjustment = 0.0f,

    .FrontSum = 0,
    .FrontDiff = 0,

    .scale_lss = 2119.2158f,
    .scale_rss = 2145.2858f,
    .scale_lfs = 3668.6240f,
    .scale_rfs = 3452.4658f,

    .SteeringMode = STEER_NORMAL
};

static float Sensors_GetDistance(float adc_value, float k)
{
    if (adc_value < 5.0f)
        return 150.0f;

    return CLAMP_FLOAT(k / sqrtf(adc_value), 10.0f, 150.0f);
}

void Sensors_Init(void)
{
    ADC1_Register_Init();

    sensor.lss_val = 0;
    sensor.rss_val = 0;
    sensor.lfs_val = 0;
    sensor.rfs_val = 0;

    sensor.Active = false;
    sensor.CrossTrackError = 0.0f;
    sensor.LastSteeringError = 0.0f;
    sensor.SteeringAdjustment = 0.0f;
    sensor.SteeringMode = STEER_NORMAL;
}

int Sensors_GetFrontSum(void)
{
    return sensor.FrontSum;
}

int Sensors_GetFrontDiff(void)
{
    return sensor.FrontDiff;
}

float Sensors_GetSteeringFeedback(void)
{
    return sensor.SteeringAdjustment;
}

float Sensors_GetCrossTrackError(void)
{
    return sensor.CrossTrackError;
}

float Sensors_CalculateSteeringAdjustment(void)
{
    float pTerm = STEERING_KP * sensor.CrossTrackError;

    float dTerm = STEERING_KD *
        (sensor.CrossTrackError - sensor.LastSteeringError);

    float adjustment = CLAMP_FLOAT(
        pTerm + dTerm,
        -STEERING_ADJUST_LIMIT,
         STEERING_ADJUST_LIMIT
    );

    sensor.LastSteeringError = sensor.CrossTrackError;
    sensor.SteeringAdjustment = adjustment;

    return adjustment;
}

void Sensors_SetSteeringMode(SteeringMode mode)
{
    sensor.LastSteeringError = sensor.CrossTrackError;
    sensor.SteeringAdjustment = 0.0f;
    sensor.SteeringMode = mode;
}

void Sensors_Enable(void)
{
    sensor.Active = true;
}

void Sensors_Disable(void)
{
    sensor.Active = false;
}

void Sensors_Update(void)
{
    if (!sensor.Active)
    {
        sensor.CrossTrackError = 0.0f;
        sensor.SteeringAdjustment = 0.0f;
        return;
    }

    /* 1. Đọc ADC thô */
    int raw_lss = ADC_Read_L();
    int raw_rss = ADC_Read_R();
    int raw_lfs = ADC_Read_FL();
    int raw_rfs = ADC_Read_FR();

    /* Lưu ADC thô để maze.c nhận diện tường */
    sensor.adc_l  = (uint16_t)raw_lss;
    sensor.adc_r  = (uint16_t)raw_rss;
    sensor.adc_fl = (uint16_t)raw_lfs;
    sensor.adc_fr = (uint16_t)raw_rfs;

    /* 2. Chuyển ADC thành khoảng cách */
    int new_lss = (int)Sensors_GetDistance(
        (float)raw_lss, sensor.scale_lss);

    int new_rss = (int)Sensors_GetDistance(
        (float)raw_rss, sensor.scale_rss);

    int new_lfs = (int)Sensors_GetDistance(
        (float)raw_lfs, sensor.scale_lfs);

    int new_rfs = (int)Sensors_GetDistance(
        (float)raw_rfs, sensor.scale_rfs);

    /* 3. Lọc EMA: 70% cũ + 30% mới */
    sensor.lss_val =
        (sensor.lss_val * 7 + new_lss * 3) / 10;

    sensor.rss_val =
        (sensor.rss_val * 7 + new_rss * 3) / 10;

    sensor.lfs_val =
        (sensor.lfs_val * 7 + new_lfs * 3) / 10;

    sensor.rfs_val =
        (sensor.rfs_val * 7 + new_rfs * 3) / 10;

    /* 4. Nhận diện tường */
    sensor.SeeLeftWall =
        sensor.lss_val < LEFT_THRESHOLD;

    sensor.SeeRightWall =
        sensor.rss_val < RIGHT_THRESHOLD;

    sensor.FrontSum =
        sensor.lfs_val + sensor.rfs_val;

    sensor.FrontDiff =
        sensor.lfs_val - sensor.rfs_val;

    sensor.SeeFrontWall =
        ((float)sensor.FrontSum / 2.0f) < FRONT_THRESHOLD;

    /* 5. Tính sai số bám tường */
    float error = 0.0f;

    float right_error =
        WALL_TARGET_R - (float)sensor.rss_val;

    float left_error =
        WALL_TARGET_L - (float)sensor.lss_val;

    switch (sensor.SteeringMode)
    {
        case STEER_NORMAL:
            if (sensor.SeeLeftWall && sensor.SeeRightWall)
                error = -left_error + right_error;
            else if (sensor.SeeLeftWall)
                error = -2.0f * left_error;
            else if (sensor.SeeRightWall)
                error = 2.0f * right_error;
            break;

        case STEER_LEFT_WALL:
            error = -2.0f * left_error;
            break;

        case STEER_RIGHT_WALL:
            error = 2.0f * right_error;
            break;

        case STEERING_OFF:
        default:
            error = 0.0f;
            break;
    }

    /* Giữ nguyên điều kiện của thuật toán gốc */
    if (sensor.FrontSum < FRONT_WALL_RELIABILITY_LIMIT)
        error = 0.0f;

    sensor.CrossTrackError =
        CLAMP_FLOAT(error, -15.0f, 15.0f);

    Sensors_CalculateSteeringAdjustment();
}

bool Sensors_OccludedLeft(void)
{
    return sensor.lfs_val < 50 &&
           sensor.rfs_val > 100;
}

bool Sensors_OccludedRight(void)
{
    return sensor.rfs_val < 50 &&
           sensor.lfs_val > 100;
}


static void Sensors_CalibrateSample(
    uint32_t duration_ms,
    float *avg_l,
    float *avg_r,
    bool front)
{
    uint32_t start = HAL_GetTick();
    uint32_t count = 0U;
    uint32_t sum_l = 0U;
    uint32_t sum_r = 0U;

    while ((uint32_t)(HAL_GetTick() - start) < duration_ms)
    {
        if (front)
        {
            sum_l += ADC_Read_FL();
            sum_r += ADC_Read_FR();
        }
        else
        {
            sum_l += ADC_Read_L();
            sum_r += ADC_Read_R();
        }

        count++;
        HAL_Delay(5);
    }

    if (count > 0U)
    {
        *avg_l = (float)sum_l / (float)count;
        *avg_r = (float)sum_r / (float)count;
    }
    else
    {
        *avg_l = 0.0f;
        *avg_r = 0.0f;
    }
}

void Sensors_AutoCalibrateLR(void)
{
    float avg_l, avg_r;

    Sensors_CalibrateSample(1000U, &avg_l, &avg_r, false);

    if (avg_l > 50.0f)
        sensor.scale_lss = WALL_TARGET_L * sqrtf(avg_l);

    if (avg_r > 50.0f)
        sensor.scale_rss = WALL_TARGET_R * sqrtf(avg_r);
}

void Sensors_AutoCalibrateFLFR(void)
{
    float avg_fl, avg_fr;

    Sensors_CalibrateSample(1000U, &avg_fl, &avg_fr, true);

    if (avg_fl > 50.0f)
        sensor.scale_lfs = FRONT_REFERENCE * sqrtf(avg_fl);

    if (avg_fr > 50.0f)
        sensor.scale_rfs = FRONT_REFERENCE * sqrtf(avg_fr);
}

uint16_t Sensors_WaitStart(void)
{
    uint16_t choice = NoStart;
    uint32_t count;

    Sensors_Enable();

    while (choice == NoStart)
    {
        Sensors_Update();

        count = 0U;
        while (Sensors_OccludedLeft())
        {
            count++;
            HAL_Delay(20);
            Sensors_Update();
        }

        if (count > 3U)
        {
            choice = LeftStart;
            break;
        }

        count = 0U;
        while (Sensors_OccludedRight())
        {
            count++;
            HAL_Delay(20);
            Sensors_Update();
        }

        if (count > 3U)
        {
            choice = RightStart;
            break;
        }
    }

    Sensors_Disable();
    return choice;
}

void IR_Update(void)
{
    Sensors_Update();
}

float Get_Steering_Feedback(void)
{
    return sensor.SteeringAdjustment;
}

void IR_Init(void)
{
    Sensors_Init();
}

void IR_Enable(void)
{
    Sensors_Enable();
}

void IR_Disable(void)
{
    Sensors_Disable();
}
