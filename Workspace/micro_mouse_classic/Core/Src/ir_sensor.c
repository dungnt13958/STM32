/**
 * @file    ir_sensor.c
 * @brief   Hiện thực thuật toán lọc EMA trên tín hiệu ADC, nhận diện tường,
 *          giữ tâm ô bằng PD và kiểm soát khoảng cách dừng cách tường trước 3cm.
 */

#include "ir_sensor.h"
#include "motion.h"
#include "motor.h"
#include <math.h>
#include <stdlib.h>
#ifndef FRONT_WALL_TARGET_3CM_SUM
#define FRONT_WALL_TARGET_3CM_SUM   2800   // Giá trị FrontSum (FL + FR) tương ứng cự ly 3cm
#endif

SensorData sensor = {
    .adc_l              = 0U,
    .adc_r              = 0U,
    .adc_fl             = 0U,
    .adc_fr             = 0U,
    .lfs_val            = 0,
    .lss_val            = 0,
    .rss_val            = 0,
    .rfs_val            = 0,
    .SeeFrontWall       = false,
    .SeeLeftWall        = false,
    .SeeRightWall       = false,
    .AtFront3cm         = false,
    .Active             = false,
    .LastSteeringError  = 0.0f,
    .CrossTrackError    = 0.0f,
    .SteeringAdjustment = 0.0f,
    .FrontSum           = 0,
    .FrontDiff          = 0,
    .target_front_3cm   = FRONT_WALL_TARGET_3CM_SUM,
    .scale_lss          = WALL_TARGET_L,
    .scale_rss          = WALL_TARGET_R,
    .scale_lfs          = 1.0f,
    .scale_rfs          = 1.0f,
    .SteeringMode       = STEER_NORMAL
};

/**
 * @brief Kẹp giới hạn giá trị số thực trong đoạn [min_val, max_val]
 */
static inline float Clamp_Float(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

/**
 * @brief Khởi tạo phần cứng ADC và đặt lại trạng thái cảm biến
 */
void Sensors_Init(void)
{
    ADC_Begin();

    sensor.adc_l              = 0U;
    sensor.adc_r              = 0U;
    sensor.adc_fl             = 0U;
    sensor.adc_fr             = 0U;
    sensor.lfs_val            = 0;
    sensor.lss_val            = 0;
    sensor.rss_val            = 0;
    sensor.rfs_val            = 0;
    sensor.SeeFrontWall       = false;
    sensor.SeeLeftWall        = false;
    sensor.SeeRightWall       = false;
    sensor.AtFront3cm         = false;
    sensor.Active             = false;
    sensor.LastSteeringError  = 0.0f;
    sensor.CrossTrackError    = 0.0f;
    sensor.SteeringAdjustment = 0.0f;
    sensor.target_front_3cm   = FRONT_WALL_TARGET_3CM_SUM;
    sensor.SteeringMode       = STEER_NORMAL;
    sensor.scale_lss          = WALL_TARGET_L;
    sensor.scale_rss          = WALL_TARGET_R;
}

float Sensors_GetDistance(float sensor_value, float k_constant)
{
    (void)k_constant;
    return sensor_value;
}

int Sensors_GetFrontSum(void)              { return sensor.FrontSum; }
int Sensors_GetFrontDiff(void)             { return sensor.FrontDiff; }
float Sensors_GetSteeringFeedback(void)    { return sensor.SteeringAdjustment; }
float Sensors_GetCrossTrackError(void)     { return sensor.CrossTrackError; }
bool Sensors_IsAtFront3cm(void)            { return sensor.AtFront3cm; }

/**
 * @brief Tính toán lượng hiệu chỉnh PD bám vách
 */
float Sensors_CalculateSteeringAdjustment(void)
{
    float pTerm = STEERING_KP * sensor.CrossTrackError;
    float dTerm = STEERING_KD * (sensor.CrossTrackError - sensor.LastSteeringError);
    float raw_adjustment = pTerm + dTerm;

    float adjustment = Clamp_Float(raw_adjustment, -STEERING_ADJUST_LIMIT, STEERING_ADJUST_LIMIT);

    sensor.LastSteeringError  = sensor.CrossTrackError;
    sensor.SteeringAdjustment = adjustment;
    return adjustment;
}

void Sensors_SetSteeringMode(SteeringMode mode)
{
    sensor.LastSteeringError  = sensor.CrossTrackError;
    sensor.SteeringAdjustment = 0.0f;
    sensor.SteeringMode       = mode;
}

void Sensors_Enable(void)
{
    sensor.Active = true;
}

void Sensors_Disable(void)
{
    sensor.Active             = false;
    sensor.LastSteeringError  = 0.0f;
    sensor.CrossTrackError    = 0.0f;
    sensor.SteeringAdjustment = 0.0f;
    GPIOB->BSRR               = GPIO_BSRR_BR6 | GPIO_BSRR_BR7;
}

/**
 * @brief Tự động lấy mẫu trung bình ADC 2 mắt ngang L và R trong 1 giây khi đặt xe giữa ô
 */
void Sensors_AutoCalibrateLR(void)
{
    uint32_t sum_l = 0U, sum_r = 0U;
    uint32_t count = 0U;
    uint32_t t_start = sys_tick_ms;

    while ((sys_tick_ms - t_start) < 1000U) {
        ADC_Update();
        sum_l += ADC_Get_L();
        sum_r += ADC_Get_R();
        count++;
        Motion_Delay_MS(5U);
    }

    if (count > 0U) {
        float reference_l = (float)sum_l / (float)count;
        float reference_r = (float)sum_r / (float)count;
        /* Chỉ hiệu chuẩn phía có tường, không lấy nhiễu nền làm mẫu chia. */
        if (reference_l > LEFT_THRESHOLD) sensor.scale_lss = reference_l;
        if (reference_r > RIGHT_THRESHOLD) sensor.scale_rss = reference_r;
    }
}

/**
 * @brief Tự động lấy mẫu FrontSum chuẩn khi đặt mũi xe cách tường đúng 3cm (30mm)
 */
void Sensors_AutoCalibrateFront3cm(void)
{
    uint32_t sum_front = 0U;
    uint32_t count = 0U;
    uint32_t t_start = sys_tick_ms;

    while ((sys_tick_ms - t_start) < 1000U) {
        ADC_Update();
        sum_front += (ADC_Get_FL() + ADC_Get_FR());
        count++;
        Motion_Delay_MS(5U);
    }

    if (count > 0U) {
        sensor.target_front_3cm = (int)(sum_front / count);
    }
}

/**
 * @brief Chu trình cập nhật cảm biến theo tín hiệu ADC (gọi ở tần số 200Hz)
 */
void Sensors_Update(void)
{
    if (!sensor.Active) {
        sensor.CrossTrackError    = 0.0f;
        sensor.SteeringAdjustment = 0.0f;
        sensor.AtFront3cm         = false;
        return;
    }

    // 1. Quét ADC 2 cụm thu phát và lưu giá trị ADC thô
    ADC_Update();

    uint16_t raw_lss = ADC_Get_L();
    uint16_t raw_rss = ADC_Get_R();
    uint16_t raw_lfs = ADC_Get_FL();
    uint16_t raw_rfs = ADC_Get_FR();

    sensor.adc_l  = raw_lss;
    sensor.adc_r  = raw_rss;
    sensor.adc_fl = raw_lfs;
    sensor.adc_fr = raw_rfs;

    // 2. Lọc thông thấp EMA trực tiếp trên giá trị ADC (70% cũ + 30% mới)
    sensor.lss_val = (sensor.lss_val * 7 + (int)raw_lss * 3) / 10;
    sensor.rss_val = (sensor.rss_val * 7 + (int)raw_rss * 3) / 10;
    sensor.lfs_val = (sensor.lfs_val * 7 + (int)raw_lfs * 3) / 10;
    sensor.rfs_val = (sensor.rfs_val * 7 + (int)raw_rfs * 3) / 10;

    // 3. Xác định trạng thái tường
    sensor.SeeLeftWall  = (sensor.lss_val > LEFT_THRESHOLD);
    sensor.SeeRightWall = (sensor.rss_val > RIGHT_THRESHOLD);

    sensor.FrontSum     = sensor.lfs_val + sensor.rfs_val;
    sensor.FrontDiff    = sensor.lfs_val - sensor.rfs_val;
    sensor.SeeFrontWall = ((sensor.FrontSum / 2) > FRONT_THRESHOLD);

    // 4. KIỂM SOÁT KHOẢNG CÁCH 3CM: Đạt cự ly 3cm khi FrontSum chạm mốc mục tiêu
    sensor.AtFront3cm   = (sensor.SeeFrontWall && (sensor.FrontSum >= sensor.target_front_3cm));

    /* 5. Mắt L/R giữ khoảng cách hai bên cân bằng. Chuẩn hóa theo mức
     * ADC tại tâm ô để hai mắt có độ nhạy khác nhau vẫn cho sai số bằng 0. */
    float error = 0.0f;
    float left_error = ((float)sensor.lss_val / sensor.scale_lss) - 1.0f;
    float right_error = ((float)sensor.rss_val / sensor.scale_rss) - 1.0f;

    if (sensor.SteeringMode == STEER_NORMAL) {
        if (sensor.SeeLeftWall && sensor.SeeRightWall) {
            error = SIDE_ERROR_SCALE * (left_error - right_error);
        } else if (sensor.SeeLeftWall) {
            error = 2.0f * SIDE_ERROR_SCALE * left_error;
        } else if (sensor.SeeRightWall) {
            error = -2.0f * SIDE_ERROR_SCALE * right_error;
        }
    } else if (sensor.SteeringMode == STEER_LEFT_WALL && sensor.SeeLeftWall) {
        error = 2.0f * SIDE_ERROR_SCALE * left_error;
    } else if (sensor.SteeringMode == STEER_RIGHT_WALL && sensor.SeeRightWall) {
        error = -2.0f * SIDE_ERROR_SCALE * right_error;
    }

    /* FL/FR chỉ chỉnh góc khi cả hai cùng nhìn thấy một tường trước.
     * FL mạnh hơn FR nghĩa là phía trái gần tường hơn: xoay nhẹ sang trái.
     * Tỉ số vi sai/tổng giúp tín hiệu ít phụ thuộc khoảng cách đến tường. */
    if (sensor.SteeringMode != STEERING_OFF && sensor.SeeFrontWall &&
        sensor.lfs_val > FRONT_THRESHOLD && sensor.rfs_val > FRONT_THRESHOLD &&
        sensor.FrontSum < (sensor.target_front_3cm - 250)) {
        float front_yaw = FRONT_YAW_ERROR_SCALE *
                          (float)sensor.FrontDiff / (float)sensor.FrontSum;
        error -= Clamp_Float(front_yaw, -FRONT_YAW_ERROR_LIMIT,
                            FRONT_YAW_ERROR_LIMIT);
    }

    // Khi đã tiến vào cự ly dưới 4cm trước tường, ngắt lái vách hông để tập trung phanh thẳng
    if (sensor.FrontSum > (sensor.target_front_3cm - 250)) {
        error = 0.0f;
    }

    sensor.CrossTrackError = Clamp_Float(error, -50.0f, 50.0f);
    Sensors_CalculateSteeringAdjustment();
}

/**
 * @brief Tự động tinh chỉnh căn vuông góc với tường trước và giữ khoảng cách đúng 3cm.
 *        Gọi hàm này sau khi dừng ở ô có tường chắn trước mặt.
 */
void Sensors_AlignToFrontWall3cm(void)
{
    if (!sensor.SeeFrontWall) return;

    uint32_t t_start = sys_tick_ms;

    while ((sys_tick_ms - t_start) < 200U) // Thực hiện tinh chỉnh trong tối đa 200ms
    {
        Sensors_Update();

        int dist_err  = sensor.target_front_3cm - sensor.FrontSum; // > 0: còn xa (>3cm), < 0: quá gần (<3cm)
        int angle_err = sensor.FrontDiff;                          // > 0: méo trái, < 0: méo phải

        // Đã nằm trong dung sai cho phép (cách 3cm ±1mm và vuông góc ±1 độ)
        if (abs(dist_err) < 40 && abs(angle_err) < 30) {
            break;
        }

        float u_fwd  = Clamp_Float((float)dist_err * 0.12f, -120.0f, 120.0f);
        float u_turn = Clamp_Float((float)angle_err * 0.25f, -140.0f, 140.0f);

        Motor_Set_Speeds((int16_t)(u_fwd - u_turn), (int16_t)(u_fwd + u_turn));
        Motion_Delay_MS(5U);
    }

    Motor_Short_Brake();
}

bool Sensors_OccludedLeft(void)
{
    return (sensor.lfs_val > FRONT_THRESHOLD && sensor.rfs_val < (FRONT_THRESHOLD / 2));
}

bool Sensors_OccludedRight(void)
{
    return (sensor.rfs_val > FRONT_THRESHOLD && sensor.lfs_val < (FRONT_THRESHOLD / 2));
}

uint16_t Sensors_WaitStart(void)
{
    Sensors_Enable();
    uint16_t choice = NoStart;

    while (choice == NoStart) {
        Sensors_Update();

        int count = 0;
        while (Sensors_OccludedLeft()) {
            count++;
            Motion_Delay_MS(20U);
            Sensors_Update();
        }
        if (count > 3) {
            choice = LeftStart;
            break;
        }

        count = 0;
        while (Sensors_OccludedRight()) {
            count++;
            Motion_Delay_MS(20U);
            Sensors_Update();
        }
        if (count > 3) {
            choice = RightStart;
            break;
        }
    }

    Sensors_Disable();
    return choice;
}

/* ================= CÁC HÀM WRAPPER ĐỒNG BỘ HỆ THỐNG ================= */
void IR_Init(void)                      { Sensors_Init(); }
void IR_Enable(void)                    { Sensors_Enable(); }
void IR_Disable(void)                   { Sensors_Disable(); }
void IR_Update(void)                    { Sensors_Update(); }
float Get_Steering_Feedback(void)       { return Sensors_GetSteeringFeedback(); }
void IR_AlignToFrontWall3cm(void)       { Sensors_AlignToFrontWall3cm(); }
void IR_CalibrateFront3cm(void)         { Sensors_AutoCalibrateFront3cm(); }
