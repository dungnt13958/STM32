/**
 * @file    motion.c
 * @brief   Bộ điều khiển động học 200Hz kết hợp Encoder + MPU6500:
 *          - Đi thẳng: Giữ hướng phản hồi tức thời (High-Gain PID + Direct Steering PWM + Fast Wheel Sync).
 *          - Adaptive yaw fusion: reject wheel/gyro disagreement.
 */

#include "motion.h"
#include "motor.h"
#include "encoder.h"
#include "mpu6500.h"
#include "ir_sensor.h"
#include "yaw_fusion.h"
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
volatile float debug_fusion_encoder_weight = 0.0f;
volatile float debug_fusion_gyro_rad_s = 0.0f;
volatile float debug_fusion_encoder_rad_s = 0.0f;
static YawFusion_t yaw_fusion;
static uint32_t previous_left_count;
static uint16_t previous_right_count;
static volatile uint8_t turn_measure_active;

static volatile MotionMode_t current_mode = MOTION_STOP;
static volatile float target_total_pulses = 0.0f;
static volatile float profile_v_target    = 0.0f;
static volatile float profile_w_target    = 0.0f;
static volatile float smooth_u_v          = 0.0f; // Chỉ làm mượt thành phần tiến thẳng

static volatile uint8_t forward_done = 1U;
static volatile uint8_t turn_done    = 1U;

/* Biến phục vụ hợp nhất Encoder + MPU khi đi thẳng và khi xoay */
static volatile float prev_mag_enc_l        = 0.0f;
static volatile float prev_mag_enc_r        = 0.0f;
static volatile float fwd_start_theta       = 0.0f;

static volatile float turn_target_delta_rad = 0.0f;
static volatile float turn_target_pulses    = 0.0f;
static volatile float turn_accum_gyro_rad   = 0.0f;
static volatile float turn_accum_enc_rad    = 0.0f;
static volatile float turn_accum_fused_rad  = 0.0f;

static PID_Controller_t pid_linear;
static PID_Controller_t pid_angular;
static PID_Controller_t pid_turn;
static float smooth_u_w;

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

/* Caller holds interrupts disabled, or SysTick is not running yet. */
static void Motion_Reset_Encoders(void)
{
    Encoder_Reset();
    previous_left_count = 0U;
    previous_right_count = 0U;
    prev_mag_enc_l = 0.0f;
    prev_mag_enc_r = 0.0f;
    yaw_fusion = (YawFusion_t){0};
}

void Motion_Init(void)
{
    Motion_Reset_Encoders();
    turn_measure_active = 0U;

    prev_mag_enc_l = 0.0f;
    prev_mag_enc_r = 0.0f;
    smooth_u_v     = 0.0f;

    /* Bộ điều khiển PID vận tốc dài */
    pid_linear.kp        = 1.8f;
    pid_linear.ki        = 4.5f;
    pid_linear.kd        = 0.0f;
    pid_linear.kff       = 2.1f;
    pid_linear.out_limit = MOTION_PWM_LIMIT;
    pid_linear.int_limit = 300.0f;

    /* Angle PID closes the loop on fused yaw, with D from fused yaw rate. */
    pid_angular.kp        = HEADING_PID_KP;
    pid_angular.ki        = HEADING_PID_KI;
    pid_angular.kd        = HEADING_PID_KD;
    pid_angular.kff       = 0.0f;
    pid_angular.out_limit = HEADING_PID_PWM_LIMIT;
    pid_angular.int_limit = HEADING_PID_I_LIMIT;
    PID_Reset(&pid_angular, 0.0f);
    pid_turn = (PID_Controller_t){
        .kp = TURN_PID_KP, .ki = TURN_PID_KI, .kd = TURN_PID_KD,
        .kff = TURN_PID_KFF, .out_limit = TURN_MAX_PWM,
        .int_limit = TURN_PID_I_LIMIT
    };
    smooth_u_w = 0.0f;

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

    uint32_t left_count = (uint32_t)Encoder_Get_Left_Count();
    uint16_t right_count = (uint16_t)Encoder_Get_Right_Count();
    float dl = MOTION_ENCODER_LEFT_SIGN *
               (float)YawFusion_Delta32(left_count, previous_left_count);
    float dr = MOTION_ENCODER_RIGHT_SIGN *
               (float)YawFusion_Delta16(right_count, previous_right_count);
    previous_left_count = left_count;
    previous_right_count = right_count;
    float step_mag_l = fabsf(dl);
    float step_mag_r = fabsf(dr);
    prev_mag_enc_l += step_mag_l;
    prev_mag_enc_r += step_mag_r;
    float cur_mag_l = prev_mag_enc_l;
    float cur_mag_r = prev_mag_enc_r;

    /* Exactly one SPI read/Kalman update per 5 ms. This API returns a
     * calibrated, Kalman-filtered RATE, not an angle. Do not additionally
     * call MPU_Update_Angle() in this control path. */
    live_gz_rad_s = MPU_Get_Calibrated_GZ_RadS();
    float gyro_w       = MOTION_GYRO_Z_SIGN * live_gz_rad_s;
    float d_theta_gyro = gyro_w * dt;
    mpu_yaw_rad = MPU_Wrap_To_Pi(mpu_yaw_rad + d_theta_gyro);
    float encoder_w = ((dr - dl) / PULSES_PER_MM) / (WHEEL_TRACK_MM * dt);
    float w_fused = YawFusion_Update(&yaw_fusion, gyro_w, encoder_w, dt);
    float d_theta_fused = w_fused * dt;
    float previous_theta = robot_pose.theta;
    robot_pose.theta = MPU_Wrap_To_Pi(previous_theta + d_theta_fused);
    robot_pose.w_angular = w_fused;
    debug_fusion_encoder_weight = yaw_fusion.encoder_weight;
    debug_fusion_gyro_rad_s = gyro_w;
    debug_fusion_encoder_rad_s = yaw_fusion.encoder_rate;
    /* Continue measuring during braking/settling, without recomputing from
     * cumulative wheel angles at the end of the turn. */
    if (turn_measure_active) {
        turn_accum_gyro_rad += d_theta_gyro;
        turn_accum_enc_rad += encoder_w * dt;
        turn_accum_fused_rad += d_theta_fused;
        debug_turn_deg_gyro = turn_accum_gyro_rad * (180.0f / (float)M_PI);
        debug_turn_deg_enc = turn_accum_enc_rad * (180.0f / (float)M_PI);
        debug_turn_deg_fused = turn_accum_fused_rad * (180.0f / (float)M_PI);
    }

    /* =========================================================================
     * TRẠNG THÁI DỪNG (MOTION_STOP)
     * ========================================================================= */
    if (current_mode == MOTION_STOP)
    {
        robot_pose.v_linear  = 0.0f;
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
            float ds_l = dl / PULSES_PER_MM;
            float ds_r = dr / PULSES_PER_MM;
            float ds   = 0.5f * (ds_l + ds_r);

            float mid_theta      = previous_theta + 0.5f * d_theta_fused;
            robot_pose.x        += ds * cosf(mid_theta);
            robot_pose.y        += ds * sinf(mid_theta);

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
            static float filtered_wall_corr = 0.0f;
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

                /* Wall feedback steers the robot; it is not an absolute yaw
                 * measurement, so do not integrate it into the pose. */
            }
            else
            {
                filtered_wall_corr = 0.0f;
            }

            /* Angle setpoint minus fused angle; wrap across +/-pi. */
            float effective_target_heading = MPU_Wrap_To_Pi(target_heading_rad + wall_heading_offset);
            float heading_err = MPU_Wrap_To_Pi(effective_target_heading - robot_pose.theta);
            debug_heading_err_deg = heading_err * (180.0f / (float)M_PI);

            float u_w_pid = FeedbackPID_Step(&pid_angular, heading_err,
                                             w_fused, 0.0f, dt);

            /* 3. Đồng bộ xung Encoder mềm (chỉ can thiệp khi có trôi lệch tích lũy) */
            float expected_diff_pulses = MPU_Wrap_To_Pi(robot_pose.theta - fwd_start_theta) * (2.0f * PULSES_PER_TURN_RAD);
            float actual_diff_pulses   = cur_mag_r - cur_mag_l;
            float sync_pulse_err       = expected_diff_pulses - actual_diff_pulses;
            float u_w_enc_sync         = Clamp_Value(FWD_ENC_SYNC_KP * sync_pulse_err, -90.0f, 90.0f);

            /* 4. Tổng hợp lực đánh lái u_w có lọc thông thấp nhẹ để triệt tiêu tiếng rít gằn của động cơ */
            float raw_u_w = u_w_pid + u_w_enc_sync - (0.30f * filtered_wall_corr);
            raw_u_w = Clamp_Value(raw_u_w, -HEADING_PID_PWM_LIMIT, HEADING_PID_PWM_LIMIT);

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

        /* Express angle error and measured rate in the commanded direction.
         * Signed feedback detects rotation opposite to the command. */
        debug_heading_err_deg = dir_sign * remain_rad * (180.0f / (float)M_PI);
        float u_mag = FeedbackPID_Step(&pid_turn, remain_rad,
                                       dir_sign * w_fused,
                                       pid_turn.kff * profile_w_target, dt);
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
    Motion_Reset_Encoders();
    prev_mag_enc_l      = 0.0f;
    prev_mag_enc_r      = 0.0f;
    robot_pose.v_linear = 0.0f;
    actual_pwm_l        = 0.0f;
    actual_pwm_r        = 0.0f;
    smooth_u_v          = V_START_MM_S * pid_linear.kff; // Khởi tạo lực kéo nền ngay từ đầu

    if (fabsf(MPU_Wrap_To_Pi(target_heading_rad - robot_pose.theta)) > 0.45f)
    {
        target_heading_rad = robot_pose.theta;
    }
    fwd_start_theta  = robot_pose.theta;

    profile_v_target = V_START_MM_S;
    forward_done     = 0U;

    PID_Reset(&pid_linear, 0.0f);
    PID_Reset(&pid_angular, 0.0f);
    smooth_u_w = 0.0f;
    current_mode     = MOTION_FORWARD;
    __enable_irq();

    uint32_t t_start = sys_tick_ms;

    while (!forward_done)
        {
            float cur_l = prev_mag_enc_l;
            float cur_r = prev_mag_enc_r;
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
    Motion_Reset_Encoders();
    prev_mag_enc_l        = 0.0f;
    prev_mag_enc_r        = 0.0f;
    actual_pwm_l          = 0.0f;
    actual_pwm_r          = 0.0f;

    turn_target_delta_rad = delta_rad;
    turn_target_pulses    = fabsf(delta_rad) * PULSES_PER_TURN_RAD;

    turn_accum_gyro_rad   = 0.0f;
    turn_accum_enc_rad    = 0.0f;
    turn_accum_fused_rad  = 0.0f;
    turn_measure_active   = 1U;
    PID_Reset(&pid_turn, 0.0f);

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
    turn_measure_active = 0U;

    debug_turn_deg_gyro  = turn_accum_gyro_rad  * (180.0f / (float)M_PI);
    debug_turn_deg_enc   = turn_accum_enc_rad   * (180.0f / (float)M_PI);
    debug_turn_deg_fused = turn_accum_fused_rad * (180.0f / (float)M_PI);
    __enable_irq();
}

void Turn_Left_90(void)     { Turn_Angle_Rad((float)(M_PI * 0.5f)); }
void Turn_Right_90(void)    { Turn_Angle_Rad((float)(-M_PI * 0.5f)); }
void Turn_Around_180(void)  { Turn_Angle_Rad((float)M_PI); }
