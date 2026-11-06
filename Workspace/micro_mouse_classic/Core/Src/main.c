/**
 * @file    main.c
 * @brief   Chương trình điều khiển Micromouse tích hợp Menu 4 Mode:
 *          - Mode 1: Đi thẳng 120cm (Sàn trống - Tắt IR)
 *          - Mode 2: Đi thẳng 120cm (Có tường - Bật IR Corridor)
 *          - Mode 3: Vẽ hình vuông 18cm x 18cm
 *          - Mode 4: Giải mê cung (FloodFill Maze Solver)
 */

#include "stm32f4xx.h"
#include "system_ui.h"
#include "motor.h"
#include "encoder.h"
#include "mpu6500.h"
#include "ir_sensor.h"
#include "motion.h"
#include "telemetry.h"
#include "maze.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

/* USART1: PA9 TX -> ESP32 RX; PA10 RX <- ESP32 TX; common GND.
 * CSV at 115200 8N1: left_count,right_count,fused_yaw_deg\r\n
 * Implementation stays in main.c so existing CubeIDE build lists work. */
static char telemetry_tx[64];
static volatile uint32_t telemetry_length;
static volatile uint32_t telemetry_index;
static volatile uint8_t telemetry_ready;

static void Telemetry_Init(void)
{
    static const uint8_t apb_shift[8] = {0, 0, 0, 0, 1, 2, 3, 4};
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR;
    GPIOA->MODER = (GPIOA->MODER & ~((3U << 18) | (3U << 20))) |
                    (2U << 18) | (2U << 20);
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~((15U << 4) | (15U << 8))) |
                     (7U << 4) | (7U << 8);
    GPIOA->OTYPER &= ~((1U << 9) | (1U << 10));
    GPIOA->OSPEEDR |= (3U << 18) | (3U << 20);
    GPIOA->PUPDR = (GPIOA->PUPDR & ~((3U << 18) | (3U << 20))) | (1U << 20);
    SystemCoreClockUpdate();
    uint32_t pclk2 = SystemCoreClock >> apb_shift[(RCC->CFGR >> 13) & 7U];
    USART1->CR1 = 0U;
    USART1->CR2 = 0U;
    USART1->CR3 = 0U;
    USART1->BRR = (pclk2 + 57600U) / 115200U;
    NVIC_SetPriority(USART1_IRQn, 3U);
    NVIC_EnableIRQ(USART1_IRQn);
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    telemetry_ready = 1U;
}

void Telemetry_Tick_5ms(void)
{
    static uint8_t ticks;
    if (!telemetry_ready) return;
    if (++ticks < 10U) return;
    ticks = 0U;
    if (telemetry_length != 0U) return; /* Drop frame if UART is still busy. */

    /* theta là góc hướng đã hợp nhất MPU + encoder, đơn vị radian.
     * Gyro GZ trả về vận tốc góc rad/s nên không dùng làm trường yaw. */
    float yaw_deg_x100 = MPU_Wrap_To_Pi(robot_pose.theta) *
                         (18000.0f / 3.14159265358979323846f);
    int32_t yaw_cd = (int32_t)(yaw_deg_x100 +
                              ((yaw_deg_x100 >= 0.0f) ? 0.5f : -0.5f));
    uint32_t magnitude = (uint32_t)((yaw_cd < 0) ? -yaw_cd : yaw_cd);
    int length = snprintf(telemetry_tx, sizeof telemetry_tx, "%ld,%ld,%s%lu.%02lu\r\n",
                          -(long)Encoder_Get_Left_Count(),
                          (long)Encoder_Get_Right_Count(),
                          yaw_cd < 0 ? "-" : "",
                          (unsigned long)(magnitude / 100U),
                          (unsigned long)(magnitude % 100U));
    if (length <= 0 || (uint32_t)length >= sizeof telemetry_tx) return;
    telemetry_index = 0U;
    telemetry_length = (uint32_t)length;
    __DMB();
    USART1->CR1 |= USART_CR1_TXEIE;
}

void USART1_IRQHandler(void)
{
    if ((USART1->SR & USART_SR_TXE) && (USART1->CR1 & USART_CR1_TXEIE))
    {
        USART1->DR = (uint8_t)telemetry_tx[telemetry_index++];
        if (telemetry_index == telemetry_length)
        {
            USART1->CR1 &= ~USART_CR1_TXEIE;
            telemetry_length = 0U;
        }
    }
    /* No receive protocol yet: discard incoming bytes and clear RX errors. */
    if (USART1->SR & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE))
    {
        (void)USART1->DR;
    }
}

#define STRAIGHT_TEST_DIST_MM       1200.0f  // 120cm
#define SQUARE_SIDE_MM              180.0f   // 18cm

#define MAZE_GOAL_X1                7
#define MAZE_GOAL_Y1                7
#define MAZE_GOAL_X2                8
#define MAZE_GOAL_Y2                8

/* ================= BIẾN QUAN SÁT TEST 120CM (LIVE EXPRESSIONS) ================= */
volatile float   dbg_120cm_final_dist_mm   = 0.0f; // Quãng đường thực tế xe đo được
volatile float   dbg_120cm_heading_err_deg = 0.0f; // Góc lệch sau 1.2 mét (độ)
volatile int32_t dbg_120cm_enc_left        = 0;    // Tổng xung bánh trái
volatile int32_t dbg_120cm_enc_right       = 0;    // Tổng xung bánh phải
volatile int32_t dbg_120cm_enc_diff        = 0;    // Chênh lệch xung giữa 2 bánh (L - R)

/* Biến quan sát test hình vuông */
volatile float   dbg_sq_dist_err_mm        = 0.0f;
volatile float   dbg_sq_heading_err_deg    = 0.0f;

static uint8_t current_selected_mode = 1U;

/**
 * @brief Bật/tắt 4 đèn LED trạng thái
 */
static void Main_Set_4LEDs(uint8_t l1, uint8_t l2, uint8_t l3, uint8_t l4)
{
    if (l1) GPIOB->BSRR = (1U << 2U); else GPIOB->BSRR = (1U << (2U + 16U));
    if (l2) GPIOB->BSRR = (1U << 1U); else GPIOB->BSRR = (1U << (1U + 16U));
    if (l3) GPIOB->BSRR = (1U << 0U); else GPIOB->BSRR = (1U << (0U + 16U));
    if (l4) GPIOA->BSRR = (1U << 7U); else GPIOA->BSRR = (1U << (7U + 16U));
}

/**
 * @brief Hiển thị LED tương ứng với Mode đang chọn trong Menu
 */
static void Main_Display_Mode_LED(uint8_t mode)
{
    switch (mode)
    {
        case 1: Main_Set_4LEDs(1U, 0U, 0U, 0U); break; // LED 1
        case 2: Main_Set_4LEDs(0U, 1U, 0U, 0U); break; // LED 2
        case 3: Main_Set_4LEDs(0U, 0U, 1U, 0U); break; // LED 3
        case 4: Main_Set_4LEDs(0U, 0U, 0U, 1U); break; // LED 4
        default: break;
    }
}

/**
 * @brief Đếm ngược 3 giây nhấp nháy đèn LED trước khi xuất phát
 */
static void Main_Countdown_Start(void)
{
    for (uint8_t i = 0U; i < 3U; i++)
    {
        Main_Set_4LEDs(1U, 1U, 1U, 1U);
        Motion_Delay_MS(150U);
        Main_Set_4LEDs(0U, 0U, 0U, 0U);
        Motion_Delay_MS(150U);
    }
    Motion_Delay_MS(250U);
}

/* =========================================================================
 * THỰC THI MODE 1 & 2: TEST ĐI THẲNG 120CM
 * ========================================================================= */
static void Main_Run_Straight_120cm(bool enable_ir_wall)
{
    Main_Countdown_Start();

    if (enable_ir_wall)
    {
        IR_Enable();
        sensor.Active = true;
    }
    else
    {
        IR_Disable();
        sensor.Active = false; // Tắt bám vách khi test trên nền gạch/sàn trống
    }

    /* Đặt lại mốc tọa độ và góc trước khi chạy */
    robot_pose.x       = 0.0f;
    robot_pose.y       = 0.0f;
    robot_pose.theta   = 0.0f;
    target_heading_rad = 0.0f;

    /* Thực hiện chạy thẳng 120cm */
    Move_Forward_MM(STRAIGHT_TEST_DIST_MM);

    Motor_Short_Brake();
    IR_Disable();

    /* Đo đạc thông số chốt hạ */
    dbg_120cm_final_dist_mm   = debug_traveled_mm;
    dbg_120cm_heading_err_deg = MPU_Wrap_To_Pi(robot_pose.theta) * (180.0f / 3.14159265f);
    dbg_120cm_enc_left        = Encoder_Get_Left_Count();
    dbg_120cm_enc_right       = Encoder_Get_Right_Count();
    dbg_120cm_enc_diff        = dbg_120cm_enc_left - dbg_120cm_enc_right;

    /* Báo kết quả bằng LED:
     * - Độ lệch hướng < 1.5°: Rất thẳng -> Sáng cả 4 LED.
     * - Lệch sang trái: Sáng 2 LED trái (LED 1, 2).
     * - Lệch sang phải: Sáng 2 LED phải (LED 3, 4). */
    if (fabsf(dbg_120cm_heading_err_deg) < 1.5f)
    {
        for (uint8_t k = 0U; k < 5U; k++)
        {
            Main_Set_4LEDs(1U, 1U, 1U, 1U); Motion_Delay_MS(150U);
            Main_Set_4LEDs(0U, 0U, 0U, 0U); Motion_Delay_MS(150U);
        }
    }
    else if (dbg_120cm_heading_err_deg > 0.0f)
    {
        Main_Set_4LEDs(1U, 1U, 0U, 0U); // Lệch trái
    }
    else
    {
        Main_Set_4LEDs(0U, 0U, 1U, 1U); // Lệch phải
    }
}

/* =========================================================================
 * THỰC THI MODE 3: TEST VẼ HÌNH VUÔNG 18CM
 * ========================================================================= */
static void Main_Run_Square_18cm(void)
{
    Main_Countdown_Start();

    IR_Disable();
    sensor.Active = false;

    robot_pose.x       = 0.0f;
    robot_pose.y       = 0.0f;
    robot_pose.theta   = 0.0f;
    target_heading_rad = 0.0f;

    for (uint8_t side = 1U; side <= 4U; side++)
    {
        Main_Display_Mode_LED(side);

        Move_Forward_MM(SQUARE_SIDE_MM);
        Motion_Delay_MS(100U);

        Turn_Right_90();
        Motion_Delay_MS(100U);
    }

    Motor_Short_Brake();

    dbg_sq_dist_err_mm     = sqrtf((robot_pose.x * robot_pose.x) + (robot_pose.y * robot_pose.y));
    dbg_sq_heading_err_deg = MPU_Wrap_To_Pi(robot_pose.theta) * (180.0f / 3.14159265f);

    if (dbg_sq_dist_err_mm < 15.0f && fabsf(dbg_sq_heading_err_deg) < 2.5f)
    {
        Main_Set_4LEDs(1U, 1U, 1U, 1U);
    }
    else
    {
        Main_Set_4LEDs(1U, 0U, 0U, 1U);
    }
}

/* =========================================================================
 * THỰC THI MODE 4: GIẢI MÊ CUNG CHÍNH THỨC
 * ========================================================================= */
static bool Main_Is_Goal(uint8_t x, uint8_t y)
{
    return ((x == MAZE_GOAL_X1 || x == MAZE_GOAL_X2) &&
            (y == MAZE_GOAL_Y1 || y == MAZE_GOAL_Y2));
}

static void Main_Advance_Coordinates(void)
{
    switch ((int)robot_dir)
    {
        case 0: robot_y++; break;
        case 1: robot_x++; break;
        case 2: if (robot_y > 0U) robot_y--; break;
        case 3: if (robot_x > 0U) robot_x--; break;
        default: break;
    }
}

static void Main_Turn_To_Direction(Direction_t next_dir)
{
    int8_t diff = (int8_t)next_dir - (int8_t)robot_dir;
    if (diff == -3) diff = 1;
    if (diff == 3)  diff = -1;

    if (diff == 1)       Turn_Right_90();
    else if (diff == -1) Turn_Left_90();
    else if (diff == 2 || diff == -2) Turn_Around_180();

    robot_dir = next_dir;
}

static void Main_Run_Maze_Search(void)
{
    Main_Countdown_Start();
    IR_Enable();

    Maze_Init();
    robot_x   = 0U;
    robot_y   = 0U;
    robot_dir = (Direction_t)0;

    while (1)
    {
        IR_Update();
        Maze_Update_Walls();

        if (Main_Is_Goal(robot_x, robot_y))
        {
            Motor_Short_Brake();
            IR_Disable();
            /* Về đích: Chớp đuổi 4 LED */
            while (1)
            {
                Main_Set_4LEDs(1U, 0U, 0U, 0U); Motion_Delay_MS(80U);
                Main_Set_4LEDs(0U, 1U, 0U, 0U); Motion_Delay_MS(80U);
                Main_Set_4LEDs(0U, 0U, 1U, 0U); Motion_Delay_MS(80U);
                Main_Set_4LEDs(0U, 0U, 0U, 1U); Motion_Delay_MS(80U);
            }
        }

        Direction_t next_dir = Maze_Get_Best_Direction();
        if (next_dir != robot_dir)
        {
            Main_Turn_To_Direction(next_dir);
            Motion_Delay_MS(30U);
        }

        Move_Forward_One_Cell();
        Main_Advance_Coordinates();
        Motion_Delay_MS(25U);
    }
}

/* =========================================================================
 * HÀM CHÍNH (MAIN)
 * ========================================================================= */
int main(void)
{
    UI_Init();
    Motor_Init();
    Encoder_Init();
    MPU_Init();
    IR_Init();
    Motion_Init();

    /* Cân chỉnh điểm 0 Gyro (Xe phải nằm im trên sàn phẳng) */
    Main_Set_4LEDs(1U, 0U, 0U, 1U);
    MPU_Calibrate_Zero();
    Telemetry_Init();
    Main_Set_4LEDs(0U, 0U, 0U, 0U);

    current_selected_mode = 1U;
    Main_Display_Mode_LED(current_selected_mode);

    while (1)
    {
        ButtonEvent_t btn = UI_Read_Button();

        /* Nhấn ngắn: Chuyển Mode (1 -> 2 -> 3 -> 4 -> 1) */
        if (btn == BTN_EVENT_SHORT_CLICK)
        {
            current_selected_mode++;
            if (current_selected_mode > 4U)
            {
                current_selected_mode = 1U;
            }
            Main_Display_Mode_LED(current_selected_mode);
        }
        /* Nhấn giữ: Chốt thực thi Mode đang chọn */
        else if (btn == BTN_EVENT_LONG_PRESS)
        {
            switch (current_selected_mode)
            {
                case 1:
                    Main_Run_Straight_120cm(false); // Mode 1: 120cm sàn trống
                    break;
                case 2:
                    Main_Run_Straight_120cm(true);  // Mode 2: 120cm có vách
                    break;
                case 3:
                    Main_Run_Square_18cm();          // Mode 3: Hình vuông 18cm
                    break;
                case 4:
                    Main_Run_Maze_Search();          // Mode 4: Giải mê cung
                    break;
                default: break;
            }

            /* Sau khi chạy xong, quay lại hiển thị Menu Mode hiện tại */
            Main_Display_Mode_LED(current_selected_mode);
        }

        Motion_Delay_MS(15U);
    }
}
