/**
 * @file    motion.c
 * @brief   Bộ điều khiển động học 200Hz kết hợp Encoder + MPU6500:
 *          - Đi thẳng: Giữ hướng phản hồi tức thời (High-Gain PID + Direct Steering PWM + Fast Wheel Sync).
 *          - Xoay 90/180 độ: Hợp nhất Gyro (60%) + Encoder (40%) + Khóa đối xứng 2 bánh.
 */

#include "motion.h"
#include "motor.h"
#include "encoder.h"
#include "mpu6500.h"
#include "ir_sensor.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

volatile uint32_t sys_tick_ms = 0U;
volatile RobotPose_t robot_pose = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
volatile float target_heading_rad = 0.0f;

volatile float actual_pwm_l = 0.0f;
volatile float actual_pwm_r = 0.0f;
volatile float debug_traveled_mm = 0.0f;

/* Biến debug trực quan cho Live Expressions */
volatile float debug_turn_deg_gyro   = 0.0f;
volatile float debug_turn_deg_enc    = 0.0f;
volatile float debug_turn_deg_fused  = 0.0f;
volatile float debug_heading_err_deg = 0.0f;

static volatile MotionMode_t current_mode = MOTION_STOP;
static volatile float target_total_pulses = 0.0f;
static volatile float profile_v_target    = 0.0f;
static volatile float profile_w_target    = 0.0f;
static volatile float smooth_u_v          = 0.0f; // Chỉ làm mượt thành phần tiến thẳng
static float filtered_wall_corr            = 0.0f;
static float smooth_u_w                    = 0.0f;

static volatile uint8_t forward_done = 1U;
static volatile uint8_t turn_done    = 1U;

/* Biến phục vụ hợp nhất Encoder + MPU khi đi thẳng và khi xoay */
static volatile float prev_mag_enc_l        = 0.0f;
static volatile float prev_mag_enc_r        = 0.0f;
static volatile float fwd_start_theta       = 0.0f;

static volatile float turn_start_theta      = 0.0f;
static volatile float turn_target_delta_rad = 0.0f;
static volatile float turn_target_pulses    = 0.0f;
static volatile float turn_accum_gyro_rad   = 0.0f;
static volatile float turn_accum_enc_rad    = 0.0f;
static volatile float turn_accum_fused_rad  = 0.0f;
static volatile float mpu_polarity_sign     = 1.0f;

static PID_Controller_t pid_linear;
static PID_Controller_t pid_angular;

static inline float Clamp_Value(float val, float min_v, float max_v)
{
    if (val < min_v) return min_v;
    if (val > max_v) return max_v;
    return val;
}

static float PID_Compute_Linear(PID_Controller_t *pid, float error, float measurement, float dt)
{
    pid->integral += error * dt;
    pid->integral = Clamp_Value(pid->integral, -pid->int_limit, pid->int_limit);

    float d_meas = (measurement - pid->prev_measurement) / dt;
    pid->prev_measurement = measurement;

    float out = (pid->kp * error) + (pid->ki * pid->integral) - (pid->kd * d_meas);
    return Clamp_Value(out, -pid->out_limit, pid->out_limit);
}

static void PID_Reset(PID_Controller_t *pid, float current_meas)
{
    pid->integral = 0.0f;
    pid->prev_measurement = current_meas;
}

void Motion_Init(void)
{
    Encoder_Reset();

    prev_mag_enc_l = 0.0f;
    prev_mag_enc_r = 0.0f;
    smooth_u_v     = 0.0f;
    smooth_u_w     = 0.0f;
    filtered_wall_corr = 0.0f;

    /* Bộ điều khiển PID vận tốc dài */
    pid_linear.kp        = 1.8f;
    pid_linear.ki        = 4.5f;
    pid_linear.kd        = 0.0f;
    pid_linear.kff       = 2.1f;
    pid_linear.out_limit = MOTION_PWM_LIMIT;
    pid_linear.int_limit = 300.0f;

    /* Bộ điều khiển PID giữ hướng góc PHẢN HỒI NHANH:
     * - kp = 1150: Bẻ lái mạnh ngay khi chớm lệch 0.5 - 1 độ
     * - ki = 85:   Bù nhanh sự lệch công suất cơ khí giữa 2 động cơ
     * - kd = 48:   Giảm chấn mạnh theo vận tốc góc w_fused để chống vọt lố/lắc đuôi */
    pid_angular.kp        = 1150.0f;
    pid_angular.ki        = 85.0f;
    pid_angular.kd        = 48.0f;
    pid_angular.kff       = 0.0f;
    pid_angular.out_limit = 950.0f;
    pid_angular.int_limit = 220.0f;

    robot_pose.x         = 0.0f;
    robot_pose.y         = 0.0f;
    robot_pose.theta     = 0.0f;
    robot_pose.v_linear  = 0.0f;
    robot_pose.w_angular = 0.0f;
    target_heading_rad   = 0.0f;

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 200U);
    NVIC_SetPriority(SysTick_IRQn, 0U);
}

void Motion_SysTick_Handler_5ms(void)
{
    const float dt = 0.005f;
    sys_tick_ms += 5U;
    Encoder_Update_Right_Count();

    /* 1. Đọc biên độ xung từ 2 bánh */
    float cur_mag_l = fabsf((float)Encoder_Get_Left_Count());
    float cur_mag_r = fabsf((float)Encoder_Get_Right_Count());

    float step_mag_l = cur_mag_l - prev_mag_enc_l;
    float step_mag_r = cur_mag_r - prev_mag_enc_r;
    if (step_mag_l < 0.0f) step_mag_l = 0.0f;
    if (step_mag_r < 0.0f) step_mag_r = 0.0f;

    prev_mag_enc_l = cur_mag_l;
    prev_mag_enc_r = cur_mag_r;

    /* 2. Đọc vận tốc góc tức thời từ Gyro MPU6500 */
    float gyro_w       = mpu_polarity_sign * MPU_Get_Calibrated_GZ_RadS();
    float d_theta_gyro = gyro_w * dt;

    /* =========================================================================
     * TRẠNG THÁI DỪNG (MOTION_STOP)
     * ========================================================================= */
    if (current_mode == MOTION_STOP)
    {
        robot_pose.v_linear  = 0.0f;
        robot_pose.w_angular = gyro_w;
        robot_pose.theta     = MPU_Wrap_To_Pi(robot_pose.theta + d_theta_gyro);
        actual_pwm_l         = 0.0f;
        actual_pwm_r         = 0.0f;
        smooth_u_v           = 0.0f;
        return;
    }

    /* =========================================================================
         * CHẾ ĐỘ ĐI THẲNG ĐỀU, ÊM VÀ TRIỆT TIÊU RUNG LẮC (CASCADE TUNED)
         * ========================================================================= */
        if (current_mode == MOTION_FORWARD)
        {
            /* A. Tính quãng đường và góc vi sai từ Encoder */
            float ds_l = step_mag_l / PULSES_PER_MM;
            float ds_r = step_mag_r / PULSES_PER_MM;
            float ds   = 0.5f * (ds_l + ds_r);

            float d_theta_enc = (ds_r - ds_l) / WHEEL_TRACK_MM;
            float w_enc       = d_theta_enc / dt;

            /* Tự động đảo dấu nếu trục Z Gyro bị ngược cực tính */
            if (fabsf(d_theta_enc) > 0.008f && ((d_theta_gyro * d_theta_enc) < -0.00004f))
            {
                mpu_polarity_sign = -mpu_polarity_sign;
                gyro_w            = -gyro_w;
                d_theta_gyro      = -d_theta_gyro;
            }

            /* Hợp nhất góc hướng: 82% Gyro + 18% Encoder */
            float d_theta_fused = (FWD_GYRO_WEIGHT * d_theta_gyro) + (FWD_ENC_WEIGHT * d_theta_enc);
            float w_fused       = (0.85f * gyro_w) + (0.15f * w_enc);

            float mid_theta      = robot_pose.theta + 0.5f * d_theta_fused;
            robot_pose.x        += ds * cosf(mid_theta);
            robot_pose.y        += ds * sinf(mid_theta);
            robot_pose.theta     = MPU_Wrap_To_Pi(robot_pose.theta + d_theta_fused);

            float raw_v          = ds / dt;
            robot_pose.v_linear  = 0.70f * robot_pose.v_linear + 0.30f * raw_v;
            robot_pose.w_angular = w_fused;

            /* B. Đếm xung quãng đường và điều kiện dừng */
            float traveled_pulses = 0.5f * (cur_mag_l + cur_mag_r);
            debug_traveled_mm     = traveled_pulses / PULSES_PER_MM;

            IR_Update();

            /* Dừng khi đủ xung hoặc cách tường trước 3cm */
            bool front_at_3cm = (sensor.Active && sensor.SeeFrontWall &&
                                (traveled_pulses > (0.55f * target_total_pulses)) &&
                                (sensor.FrontSum >= sensor.target_front_3cm));
            bool front_emergency = (sensor.Active && (sensor.FrontSum > FRONT_WALL_EMERGENCY_SUM));

            if ((traveled_pulses >= target_total_pulses) || front_at_3cm || front_emergency)
            {
                Motor_Short_Brake();
                actual_pwm_l     = 0.0f;
                actual_pwm_r     = 0.0f;
                smooth_u_v       = 0.0f;
                profile_v_target = 0.0f;
                current_mode     = MOTION_STOP;
                forward_done     = 1U;
                return;
            }

            /* C. Quỹ đạo vận tốc hình thang làm mượt gia tốc */
            float remain_pulses = target_total_pulses - traveled_pulses;
            float remain_mm     = remain_pulses / PULSES_PER_MM;

            if (remain_mm <= 55.0f)
            {
                float decel_ratio  = Clamp_Value(remain_mm / 55.0f, 0.0f, 1.0f);
                float decel_target = V_START_MM_S + (V_CRUISE_MAX_MM_S - V_START_MM_S) * decel_ratio;
                if (profile_v_target > decel_target)
                {
                    profile_v_target -= DECEL_MM_S2 * dt;
                    if (profile_v_target < decel_target) profile_v_target = decel_target;
                }
            }
            else if (profile_v_target < V_CRUISE_MAX_MM_S)
            {
                profile_v_target += ACCEL_MM_S2 * dt;
                if (profile_v_target > V_CRUISE_MAX_MM_S) profile_v_target = V_CRUISE_MAX_MM_S;
            }

            float v_error = profile_v_target - robot_pose.v_linear;
            float raw_u_v = (pid_linear.kff * profile_v_target) +
                            PID_Compute_Linear(&pid_linear, v_error, robot_pose.v_linear, dt);

            /* Làm mượt tốc độ tiến để hai bánh bám sàn đều, không trượt */
            float diff_uv = Clamp_Value(raw_u_v - smooth_u_v, -PWM_SLEW_RATE_PER_5MS, PWM_SLEW_RATE_PER_5MS);
            smooth_u_v    = Clamp_Value(smooth_u_v + diff_uv, 0.0f, MOTION_PWM_LIMIT);

            /* D. BỘ ĐIỀU KHIỂN HƯỚNG CASCADE CHỐNG LẮC ĐUÔI (FISH-TAILING ELIMINATOR) */
            float wall_heading_offset = 0.0f;

            if (sensor.Active)
            {
                float raw_wall = Get_Steering_Feedback();
                /* Bộ lọc giới hạn biến thiên chống giật khi IR quét qua khe nối giữa các ô tường */
                float wall_diff = Clamp_Value(raw_wall - filtered_wall_corr,
                                              -WALL_RATE_LIMIT_PER_5MS, WALL_RATE_LIMIT_PER_5MS);
                filtered_wall_corr += wall_diff;

                wall_heading_offset = Clamp_Value(-filtered_wall_corr * WALL_TO_HEADING_GAIN,
                                                  -WALL_HEADING_MAX_OFFSET_RAD,
                                                   WALL_HEADING_MAX_OFFSET_RAD);

                /* Hợp nhất từ từ góc tường vào Gyro */
                if (sensor.SeeLeftWall && sensor.SeeRightWall)
                {
                    robot_pose.theta = MPU_Wrap_To_Pi(robot_pose.theta +
                                       (filtered_wall_corr * 0.005f * dt));
                }
            }
            else
            {
                filtered_wall_corr = 0.0f;
            }

            /* 1. Tầng ngoài: Sai số góc -> Vận tốc góc mục tiêu */
            float effective_target_heading = MPU_Wrap_To_Pi(target_heading_rad + wall_heading_offset);
            float heading_err = MPU_Wrap_To_Pi(effective_target_heading - robot_pose.theta);
            debug_heading_err_deg = heading_err * (180.0f / (float)M_PI);

            /* Chỉ tích phân khi sai số nhỏ (< 2.5 độ) để chống bão hòa tích phân gây lắc */
            if (fabsf(heading_err) < 0.0436f)
            {
                pid_angular.integral += heading_err * dt;
                pid_angular.integral  = Clamp_Value(pid_angular.integral, -60.0f, 60.0f);
            }
            else
            {
                pid_angular.integral *= 0.85f; // Xả tích phân nhanh khi xe bị lệch lớn
            }

            /* 2. Tầng trong: Lực đánh lái dựa trên góc và giảm chấn vận tốc xoay w_fused */
            // Kp_angle = 720.0f, Ki_angle = 45.0f, Kd_damping = 36.0f (êm và không bị giật lố)
            float u_w_pid = (720.0f * heading_err) +
                            (45.0f  * pid_angular.integral) -
                            (36.0f  * w_fused);

            /* 3. Đồng bộ xung Encoder mềm (chỉ can thiệp khi có trôi lệch tích lũy) */
            float expected_diff_pulses = MPU_Wrap_To_Pi(robot_pose.theta - fwd_start_theta) * (2.0f * PULSES_PER_TURN_RAD);
            float actual_diff_pulses   = cur_mag_r - cur_mag_l;
            float sync_pulse_err       = expected_diff_pulses - actual_diff_pulses;
            float u_w_enc_sync         = Clamp_Value(FWD_ENC_SYNC_KP * sync_pulse_err, -90.0f, 90.0f);

            /* 4. Tổng hợp lực đánh lái u_w có lọc thông thấp nhẹ để triệt tiêu tiếng rít gằn của động cơ */
            float raw_u_w = u_w_pid + u_w_enc_sync - (0.30f * filtered_wall_corr);
            raw_u_w = Clamp_Value(raw_u_w, -550.0f, 550.0f);

            /* Lọc 80% cũ + 20% mới cho tín hiệu bẻ lái: phản xạ vẫn cực nhanh nhưng không làm gắt bánh */
            smooth_u_w = (0.75f * smooth_u_w) + (0.25f * raw_u_w);

            float target_pwm_l = smooth_u_v - smooth_u_w;
            float target_pwm_r = smooth_u_v + smooth_u_w;

            /* Bảo toàn 100% mô-men đánh lái khi 1 bánh chạm đáy 0 */
            if (target_pwm_l < 0.0f)
            {
                target_pwm_r -= target_pwm_l;
                target_pwm_l  = 0.0f;
            }
            else if (target_pwm_r < 0.0f)
            {
                target_pwm_l -= target_pwm_r;
                target_pwm_r  = 0.0f;
            }

            float max_val = fmaxf(target_pwm_l, target_pwm_r);
            if (max_val > MOTION_PWM_LIMIT)
            {
                float excess = max_val - MOTION_PWM_LIMIT;
                target_pwm_l = fmaxf(0.0f, target_pwm_l - excess);
                target_pwm_r = fmaxf(0.0f, target_pwm_r - excess);
            }

            actual_pwm_l = target_pwm_l;
            actual_pwm_r = target_pwm_r;
            Motor_Set_Speeds((int16_t)actual_pwm_l, (int16_t)actual_pwm_r);
            return;
        }

    /* =========================================================================
     * CHẾ ĐỘ RẼ 90 / 180 ĐỘ (MOTION_TURN: FUSED ENCODER + MPU + WHEEL SYNC)
     * ========================================================================= */
    if (current_mode == MOTION_TURN)
    {
        float dir_sign   = (turn_target_delta_rad >= 0.0f) ? 1.0f : -1.0f;
        float cur_pulses = 0.5f * (cur_mag_l + cur_mag_r);

        turn_accum_gyro_rad += d_theta_gyro;

        if (cur_pulses > 90.0f && ((turn_accum_gyro_rad * dir_sign) < -0.06f))
        {
            mpu_polarity_sign   = -mpu_polarity_sign;
            turn_accum_gyro_rad = -turn_accum_gyro_rad;
            gyro_w              = -gyro_w;
        }

        turn_accum_enc_rad = dir_sign * (cur_pulses / PULSES_PER_TURN_RAD);
        float d_theta_enc_step = dir_sign * (0.5f * (step_mag_l + step_mag_r) / PULSES_PER_TURN_RAD);
        float w_enc = d_theta_enc_step / dt;

        turn_accum_fused_rad = (TURN_GYRO_WEIGHT * turn_accum_gyro_rad) +
                               (TURN_ENC_WEIGHT  * turn_accum_enc_rad);

        robot_pose.theta     = MPU_Wrap_To_Pi(turn_start_theta + turn_accum_fused_rad);
        robot_pose.w_angular = (TURN_GYRO_WEIGHT * gyro_w) + (TURN_ENC_WEIGHT * w_enc);

        debug_turn_deg_gyro  = turn_accum_gyro_rad  * (180.0f / (float)M_PI);
        debug_turn_deg_enc   = turn_accum_enc_rad   * (180.0f / (float)M_PI);
        debug_turn_deg_fused = turn_accum_fused_rad * (180.0f / (float)M_PI);

        float target_abs   = fabsf(turn_target_delta_rad);
        float stop_target  = (target_abs > TURN_STOP_LEAD_RAD) ? (target_abs - TURN_STOP_LEAD_RAD) : target_abs;
        float progress_abs = dir_sign * turn_accum_fused_rad;
        float remain_rad   = stop_target - progress_abs;

        if (remain_rad <= TURN_TOLERANCE_RAD)
        {
            Motor_Short_Brake();
            actual_pwm_l     = 0.0f;
            actual_pwm_r     = 0.0f;
            profile_w_target = 0.0f;
            current_mode     = MOTION_STOP;
            turn_done        = 1U;
            return;
        }

        if (remain_rad <= TURN_DECEL_RAD)
        {
            float ratio   = Clamp_Value(remain_rad / TURN_DECEL_RAD, 0.0f, 1.0f);
            float decel_w = W_END_RAD_S + (W_TURN_MAX_RAD_S - W_END_RAD_S) * ratio;
            if (profile_w_target > decel_w) profile_w_target = decel_w;
        }
        else if (profile_w_target < W_TURN_MAX_RAD_S)
        {
            profile_w_target += ALPHA_TURN_RAD_S2 * dt;
            if (profile_w_target > W_TURN_MAX_RAD_S) profile_w_target = W_TURN_MAX_RAD_S;
        }

        float w_abs = fabsf(robot_pose.w_angular);
        float w_err = profile_w_target - w_abs;

        float u_mag = (80.0f * profile_w_target) + (90.0f * w_err) + (190.0f * remain_rad);
        u_mag = Clamp_Value(u_mag, TURN_MIN_PWM, TURN_MAX_PWM);

        float sync_diff = cur_mag_r - cur_mag_l;
        float sync_corr = Clamp_Value(TURN_ENC_SYNC_KP * sync_diff, -110.0f, 110.0f);

        float pwm_l_mag = Clamp_Value(u_mag + sync_corr, TURN_MIN_PWM, TURN_MAX_PWM);
        float pwm_r_mag = Clamp_Value(u_mag - sync_corr, TURN_MIN_PWM, TURN_MAX_PWM);

        actual_pwm_l = -dir_sign * pwm_l_mag;
        actual_pwm_r =  dir_sign * pwm_r_mag;
        Motor_Set_Speeds((int16_t)actual_pwm_l, (int16_t)actual_pwm_r);
        return;
    }
}

void Motion_Delay_MS(uint32_t ms)
{
    uint32_t start = sys_tick_ms;
    while ((sys_tick_ms - start) < ms) {
        __NOP();
    }
}

void Move_Forward_MM(float dist_mm)
{
    if (dist_mm <= 0.0f) return;

    float raw_target_pulses = dist_mm * PULSES_PER_MM;
    if (raw_target_pulses > FORWARD_STOP_LEAD_PULSES) {
        target_total_pulses = raw_target_pulses - FORWARD_STOP_LEAD_PULSES;
    } else {
        target_total_pulses = raw_target_pulses;
    }

    __disable_irq();
    Encoder_Reset();
    prev_mag_enc_l      = 0.0f;
    prev_mag_enc_r      = 0.0f;
    robot_pose.v_linear = 0.0f;
    actual_pwm_l        = 0.0f;
    actual_pwm_r        = 0.0f;
    smooth_u_v          = V_START_MM_S * pid_linear.kff; // Khởi tạo lực kéo nền ngay từ đầu
    smooth_u_w          = 0.0f;
    filtered_wall_corr  = 0.0f;

    if (fabsf(MPU_Wrap_To_Pi(target_heading_rad - robot_pose.theta)) > 0.45f)
    {
        target_heading_rad = robot_pose.theta;
    }
    fwd_start_theta  = robot_pose.theta;

    profile_v_target = V_START_MM_S;
    forward_done     = 0U;

    PID_Reset(&pid_linear, 0.0f);
    PID_Reset(&pid_angular, 0.0f);
    current_mode     = MOTION_FORWARD;
    __enable_irq();

    uint32_t t_start = sys_tick_ms;

    while (!forward_done)
        {
            float cur_l = fabsf((float)Encoder_Get_Left_Count());
            float cur_r = fabsf((float)Encoder_Get_Right_Count());
            float current_pulses = 0.5f * (cur_l + cur_r);

            debug_traveled_mm = current_pulses / PULSES_PER_MM;

            /* ĐIỀU KIỆN 1: Đủ số xung quãng đường (khi không có tường trước mặt) */
            if (current_pulses >= target_total_pulses)
            {
                break;
            }

            /* ĐIỀU KIỆN 2 (DỪNG CÁCH TƯỜNG 3CM):
             * Khi xe đã đi qua hơn 60% quãng đường ô (tránh bắt nhầm tường từ ô trước)
             * và cảm biến phát hiện tường trước chạm mốc 3cm -> Dừng ngay lập tức! */
            if (sensor.Active && sensor.SeeFrontWall && (current_pulses > (0.60f * target_total_pulses)))
            {
                if (sensor.AtFront3cm || (sensor.FrontSum >= sensor.target_front_3cm))
                {
                    break;
                }
            }

            /* Phanh khẩn cấp dự phòng nếu gặp vật cản bất ngờ */
            if (sensor.Active && (sensor.FrontSum > FRONT_WALL_EMERGENCY_SUM))
            {
                break;
            }

            if ((sys_tick_ms - t_start) > 10000U)
            {
                break;
            }
        }

        __disable_irq();
        Motor_Short_Brake();
        current_mode     = MOTION_STOP;
        actual_pwm_l     = 0.0f;
        actual_pwm_r     = 0.0f;
        smooth_u_v       = 0.0f;
        profile_v_target = 0.0f;
        forward_done     = 1U;
        __enable_irq();

        Motion_Delay_MS(30U);

        /* Nếu trước mặt là tường, tự động nhích nhẹ để vuông góc và cách tường chuẩn xác 3cm */
        if (sensor.Active && sensor.SeeFrontWall)
        {
            IR_AlignToFrontWall3cm();
        }

    __disable_irq();
    Motor_Short_Brake();
    current_mode     = MOTION_STOP;
    actual_pwm_l     = 0.0f;
    actual_pwm_r     = 0.0f;
    smooth_u_v       = 0.0f;
    profile_v_target = 0.0f;
    forward_done     = 1U;
    __enable_irq();

    Motion_Delay_MS(30U);
}

void Move_Forward_One_Cell(void)
{
    Move_Forward_MM(CELL_SIZE_MM);
}

void Turn_Angle_Rad(float delta_rad)
{
    if (fabsf(delta_rad) < 0.02f) return;

    __disable_irq();
    Encoder_Reset();
    prev_mag_enc_l        = 0.0f;
    prev_mag_enc_r        = 0.0f;
    actual_pwm_l          = 0.0f;
    actual_pwm_r          = 0.0f;

    turn_start_theta      = robot_pose.theta;
    turn_target_delta_rad = delta_rad;
    turn_target_pulses    = fabsf(delta_rad) * PULSES_PER_TURN_RAD;

    turn_accum_gyro_rad   = 0.0f;
    turn_accum_enc_rad    = 0.0f;
    turn_accum_fused_rad  = 0.0f;

    target_heading_rad    = MPU_Wrap_To_Pi(target_heading_rad + delta_rad);
    float snapped_grid    = roundf(target_heading_rad / (float)(M_PI * 0.5f)) * (float)(M_PI * 0.5f);
    target_heading_rad    = MPU_Wrap_To_Pi(snapped_grid);

    profile_w_target      = W_START_RAD_S;
    turn_done             = 0U;
    current_mode          = MOTION_TURN;
    __enable_irq();

    uint32_t t_start = sys_tick_ms;

    while (!turn_done)
    {
        float dir_sign     = (turn_target_delta_rad >= 0.0f) ? 1.0f : -1.0f;
        float target_abs   = fabsf(turn_target_delta_rad);
        float stop_target  = (target_abs > TURN_STOP_LEAD_RAD) ? (target_abs - TURN_STOP_LEAD_RAD) : target_abs;
        float progress_abs = dir_sign * turn_accum_fused_rad;

        if ((stop_target - progress_abs) <= TURN_TOLERANCE_RAD)
        {
            break;
        }

        if ((sys_tick_ms - t_start) > 1600U)
        {
            break;
        }

        __NOP();
    }

    __disable_irq();
    Motor_Short_Brake();
    current_mode     = MOTION_STOP;
    actual_pwm_l     = 0.0f;
    actual_pwm_r     = 0.0f;
    profile_w_target = 0.0f;
    turn_done        = 1U;
    __enable_irq();

    Motion_Delay_MS(35U);

    __disable_irq();
    float final_mag_l = fabsf((float)Encoder_Get_Left_Count());
    float final_mag_r = fabsf((float)Encoder_Get_Right_Count());
    float dir_sign    = (turn_target_delta_rad >= 0.0f) ? 1.0f : -1.0f;

    turn_accum_enc_rad   = dir_sign * (0.5f * (final_mag_l + final_mag_r) / PULSES_PER_TURN_RAD);
    turn_accum_fused_rad = (TURN_GYRO_WEIGHT * turn_accum_gyro_rad) +
                           (TURN_ENC_WEIGHT  * turn_accum_enc_rad);

    robot_pose.theta     = MPU_Wrap_To_Pi(turn_start_theta + turn_accum_fused_rad);

    debug_turn_deg_gyro  = turn_accum_gyro_rad  * (180.0f / (float)M_PI);
    debug_turn_deg_enc   = turn_accum_enc_rad   * (180.0f / (float)M_PI);
    debug_turn_deg_fused = turn_accum_fused_rad * (180.0f / (float)M_PI);
    __enable_irq();
}

void Turn_Left_90(void)     { Turn_Angle_Rad((float)(M_PI * 0.5f)); }
void Turn_Right_90(void)    { Turn_Angle_Rad((float)(-M_PI * 0.5f)); }
void Turn_Around_180(void)  { Turn_Angle_Rad((float)M_PI); }
