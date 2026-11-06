#ifndef INC_MOTION_H_
#define INC_MOTION_H_

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "encoder.h"
#include "feedback_pid.h"

#ifndef M_PI
#define M_PI                        3.14159265358979323846f
#endif

/* ================= 1. THÔNG SỐ CƠ KHÍ & ENCODER ================= */
#define CELL_SIZE_MM                180.0f                                     // Chiều dài 1 ô chạy thực tế (170mm)
#define TARGET_CELL_PULSES          (CELL_SIZE_MM * PULSES_PER_MM)             // ~4861 xung cho 170mm
#define FORWARD_STOP_LEAD_PULSES    (3.5f * PULSES_PER_MM)                     // Bù quán tính phanh khi đi thẳng (~100 xung)

#ifndef WHEEL_TRACK_MM
#define WHEEL_TRACK_MM              70.0f                                      // Khoảng cách giữa 2 tâm vệt bánh xe (mm)
#endif
#define MOUSE_RADIUS_MM             (WHEEL_TRACK_MM * 0.5f)                    // Bán kính quay quanh tâm trục bánh (40mm)
#define PULSES_PER_TURN_RAD         (MOUSE_RADIUS_MM * PULSES_PER_MM)          // Số xung trên mỗi radian xoay (~1143.86 xung/rad)

/* ================= 2. TRỌNG SỐ KẾT HỢP ENCODER + MPU6500 ================= */
/* Fixed sensor polarity: forward wheel motion positive, CCW yaw positive.
 * Verify on hardware; NEVER infer gyro polarity from spinning wheels. */
#define MOTION_ENCODER_LEFT_SIGN    (-1.0f)
#define MOTION_ENCODER_RIGHT_SIGN   (1.0f)
#define MOTION_GYRO_Z_SIGN          (-1.0f)
/* Initial gains; tune on the robot after verifying sensor polarity.
 * P: PWM/rad, I: PWM/(rad*s), D: PWM/(rad/s). */
#define HEADING_PID_KP              720.0f
#define HEADING_PID_KI              45.0f
#define HEADING_PID_KD              36.0f
#define HEADING_PID_I_LIMIT         0.5f
#define HEADING_PID_PWM_LIMIT       550.0f
#define TURN_PID_KP                 190.0f
#define TURN_PID_KI                 30.0f
#define TURN_PID_KD                 90.0f
#define TURN_PID_KFF                170.0f
#define TURN_PID_I_LIMIT            0.5f
#define FWD_ENC_SYNC_KP             2.0f    // Hệ số khóa cứng đồng bộ số xung 2 bánh khi đi thẳng

#define TURN_ENC_SYNC_KP            0.60f   // Hệ số ép 2 bánh quay đối xứng (d_l == d_r) khi xoay tại chỗ

/* ================= 3. QUỸ ĐẠO ĐI THẲNG ================= */
#define V_START_MM_S                120.0f  // Vận tốc xuất phát (mm/s)
#define V_CRUISE_MAX_MM_S           420.0f  // Vận tốc hành trình tối đa (mm/s)
#define ACCEL_MM_S2                 1600.0f // Gia tốc tăng tốc (mm/s^2)
#define DECEL_MM_S2                 2200.0f // Gia tốc hãm phanh (mm/s^2)

#define FRONT_WALL_STOP_3CM_SUM     2800    // Ngưỡng FrontSum (FL + FR) dừng chuẩn cách vách 3cm
#define FRONT_WALL_EMERGENCY_SUM    3200    // Ngưỡng ADC phanh khẩn cấp khi sát tường trước

/* ================= 4. QUỸ ĐẠO XOAY 90 / 180 ĐỘ ================= */
#define W_START_RAD_S               1.4f    // Vận tốc góc khởi đầu (rad/s)
#define W_TURN_MAX_RAD_S            4.2f    // Vận tốc góc tối đa khi xoay (rad/s)
#define W_END_RAD_S                 0.65f   // Vận tốc góc tiếp cận đích trước khi phanh
#define ALPHA_TURN_RAD_S2           24.0f   // Gia tốc góc (rad/s^2)
#define TURN_DECEL_RAD              0.60f   // Bắt đầu giảm tốc khi còn cách đích ~34 độ
#define TURN_STOP_LEAD_RAD          0.030f  // Ngắt sớm ~1.7 độ bù quán tính trượt khi phanh
#define TURN_TOLERANCE_RAD          0.020f  // Sai số góc cho phép dừng (~1.1 độ)
#define TURN_MIN_PWM                185.0f  // PWM tối thiểu thắng ma sát tĩnh (Deadband floor)
#define TURN_MAX_PWM                560.0f  // Giới hạn PWM tối đa khi xoay chống trượt bánh

/* ================= 5. BÁM VÁCH HỒNG NGOẠI (IR CORRIDOR) ================= */
#ifndef WALL_RATE_LIMIT_PER_5MS
#define WALL_RATE_LIMIT_PER_5MS     18.0f   // Giới hạn biên độ giật của cảm biến IR qua khe hở vách
#endif
#define WALL_TO_HEADING_GAIN        0.0032f
#define WALL_HEADING_MAX_OFFSET_RAD 0.20f
#define GYRO_CORRIDOR_FUSION_RATE   0.012f

/* ================= 6. GIỚI HẠN CÔNG SUẤT ĐỘNG CƠ ================= */
#define MOTION_PWM_LIMIT            1350.0f
#define PWM_SLEW_RATE_PER_5MS       90.0f

typedef enum {
    MOTION_STOP = 0,
    MOTION_FORWARD,
    MOTION_TURN
} MotionMode_t;

typedef struct {
    float x;
    float y;
    float theta;
    float v_linear;
    float w_angular;
} RobotPose_t;

extern volatile uint32_t sys_tick_ms;
extern volatile RobotPose_t robot_pose;
extern volatile float target_heading_rad;
extern volatile float actual_pwm_l;
extern volatile float actual_pwm_r;
extern volatile float debug_traveled_mm;

/* Biến quan sát trên Live Expressions */
extern volatile float debug_turn_deg_gyro;
extern volatile float debug_turn_deg_enc;
extern volatile float debug_turn_deg_fused;
extern volatile float debug_heading_err_deg;
extern volatile float debug_fusion_encoder_weight;
extern volatile float debug_fusion_gyro_rad_s;
extern volatile float debug_fusion_encoder_rad_s;

void Motion_Init(void);
void Motion_SysTick_Handler_5ms(void);
void Motion_Delay_MS(uint32_t ms);

void Move_Forward_MM(float dist_mm);
void Move_Forward_One_Cell(void);

void Turn_Angle_Rad(float delta_rad);
void Turn_Left_90(void);
void Turn_Right_90(void);
void Turn_Around_180(void);

#endif /* INC_MOTION_H_ */
