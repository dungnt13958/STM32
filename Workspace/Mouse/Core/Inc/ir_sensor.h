#ifndef INC_IR_SENSOR_H_
#define INC_IR_SENSOR_H_

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

typedef enum
{
    STEER_NORMAL = 0,
    STEER_LEFT_WALL,
    STEER_RIGHT_WALL,
    STEERING_OFF
} SteeringMode;

#define NoStart     0U
#define LeftStart   1U
#define RightStart  2U

typedef struct
{
    /* ADC thô: dùng cho nhận diện tường trong maze.c */
    volatile uint16_t adc_l;
    volatile uint16_t adc_r;
    volatile uint16_t adc_fl;
    volatile uint16_t adc_fr;

    /* Các trường khoảng cách đã lọc */
    volatile int lfs_val;
    volatile int lss_val;
    volatile int rss_val;
    volatile int rfs_val;

    volatile bool SeeFrontWall;
    volatile bool SeeLeftWall;
    volatile bool SeeRightWall;

    volatile bool Active;
    volatile float LastSteeringError;
    volatile float CrossTrackError;
    volatile float SteeringAdjustment;

    volatile int FrontSum;
    volatile int FrontDiff;

    float scale_lss;
    float scale_rss;
    float scale_lfs;
    float scale_rfs;

    SteeringMode SteeringMode;
} SensorData;

extern SensorData sensor;

void Sensors_Init(void);
void Sensors_Update(void);

int Sensors_GetFrontSum(void);
int Sensors_GetFrontDiff(void);

float Sensors_GetSteeringFeedback(void);
float Sensors_GetCrossTrackError(void);
float Sensors_CalculateSteeringAdjustment(void);

void Sensors_SetSteeringMode(SteeringMode mode);
void Sensors_Enable(void);
void Sensors_Disable(void);

void Sensors_AutoCalibrateLR(void);
void Sensors_AutoCalibrateFLFR(void);

bool Sensors_OccludedLeft(void);
bool Sensors_OccludedRight(void);

uint16_t Sensors_WaitStart(void);

void IR_Update(void);
float Get_Steering_Feedback(void);

void IR_Init(void);
void IR_Enable(void);
void IR_Disable(void);

#endif
