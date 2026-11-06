/**
 * @file    ir_sensor.h
 * @brief   Bộ xử lý khoảng cách hồng ngoại, phát hiện tường và PD giữ tâm đường
 */

#ifndef INC_IR_SENSOR_H_
#define INC_IR_SENSOR_H_

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>
#include "adc.h"

/* ================= HẰNG SỐ NGƯỠNG KHOẢNG CÁCH & ĐIỀU KHIỂN ================= */
/* ================= HẰNG SỐ NGƯỠNG ADC & ĐIỀU KHIỂN ================= */
#ifndef LEFT_THRESHOLD
#define LEFT_THRESHOLD                  200     // Ngưỡng ADC nhận diện có tường trái
#endif
#ifndef RIGHT_THRESHOLD
#define RIGHT_THRESHOLD                 120     // Ngưỡng ADC nhận diện có tường phải
#endif
#ifndef FRONT_THRESHOLD
#define FRONT_THRESHOLD                 300     // Ngưỡng ADC trung bình ((FL+FR)/2) nhận diện tường trước
#endif

#ifndef WALL_TARGET_L
#define WALL_TARGET_L                   200.0f  // Giá trị ADC mắt trái khi xe đặt chính giữa ô
#endif
#ifndef WALL_TARGET_R
#define WALL_TARGET_R                   120.0f  // Giá trị ADC mắt phải khi xe đặt chính giữa ô
#endif
#ifndef FRONT_REFERENCE
#define FRONT_REFERENCE                 300.0f  // Giá trị ADC chuẩn mũi xe
#endif
#ifndef FRONT_WALL_RELIABILITY_LIMIT
#define FRONT_WALL_RELIABILITY_LIMIT    1200    // Tổng ADC (FL+FR) khi mũi sát tường trước để ngắt bẻ lái ngang
#endif

#ifndef STEERING_KP
#define STEERING_KP                     0.45f   // Hệ số tỉ lệ PD bám vách (tính theo đơn vị ADC)
#endif
#ifndef STEERING_KD
#define STEERING_KD                     1.20f   // Hệ số vi phân PD bám vách
#endif
#ifndef STEERING_ADJUST_LIMIT
#define STEERING_ADJUST_LIMIT           150.0f  // Trần giới hạn lực bẻ lái vách
#endif

/* Sai số bám vách được chuẩn hóa theo mức ADC khi xe nằm giữa hành lang. */
#define SIDE_ERROR_SCALE                40.0f
#define FRONT_YAW_ERROR_SCALE           40.0f
#define FRONT_YAW_ERROR_LIMIT           12.0f

/* ================= CHẾ ĐỘ BÁM VÁCH & KHỞI ĐỘNG ================= */
typedef enum {
    STEER_NORMAL = 0,
    STEER_LEFT_WALL,
    STEER_RIGHT_WALL,
    STEERING_OFF
} SteeringMode;

#define NoStart     0U
#define LeftStart   1U
#define RightStart  2U

typedef struct {
    uint16_t adc_l;
    uint16_t adc_r;
    uint16_t adc_fl;
    uint16_t adc_fr;
    int lfs_val;
    int lss_val;
    int rss_val;
    int rfs_val;
    bool SeeFrontWall;
    bool SeeLeftWall;
    bool SeeRightWall;
    bool AtFront3cm;            // Cờ báo hiệu mũi xe đã chạm mốc 3cm trước vách
    bool Active;
    float LastSteeringError;
    float CrossTrackError;
    float SteeringAdjustment;
    int FrontSum;
    int FrontDiff;
    int target_front_3cm;       // Ngưỡng ADC mục tiêu cho cự ly 3cm (~2800)
    float scale_lss;
    float scale_rss;
    float scale_lfs;
    float scale_rfs;
    SteeringMode SteeringMode;
} SensorData;

extern SensorData sensor;

void Sensors_AutoCalibrateFront3cm(void);
void Sensors_AlignToFrontWall3cm(void);
bool Sensors_IsAtFront3cm(void);

void IR_CalibrateFront3cm(void);
void IR_AlignToFrontWall3cm(void);

/* ================= NGUYÊN MẪU HÀM ================= */
void Sensors_Init(void);
void Sensors_Update(void);

int Sensors_GetFrontSum(void);
int Sensors_GetFrontDiff(void);

float Sensors_GetSteeringFeedback(void);
float Sensors_GetCrossTrackError(void);
float Sensors_CalculateSteeringAdjustment(void);
float Sensors_GetDistance(float sensor_value, float k_constant);

void Sensors_SetSteeringMode(SteeringMode mode);
void Sensors_Enable(void);
void Sensors_Disable(void);

void Sensors_AutoCalibrateLR(void);
void Sensors_AutoCalibrateFLFR(void);

bool Sensors_OccludedLeft(void);
bool Sensors_OccludedRight(void);
uint16_t Sensors_WaitStart(void);

/* Các hàm đồng bộ với motion.c và main.c */
void IR_Init(void);
void IR_Enable(void);
void IR_Disable(void);
void IR_Update(void);
float Get_Steering_Feedback(void);

#endif /* INC_IR_SENSOR_H_ */
