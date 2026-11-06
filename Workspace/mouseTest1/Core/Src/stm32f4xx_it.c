#include "main.h"
#include "stm32f4xx_it.h"


/* ============================================================
 * g_ms được định nghĩa trong main.c
 *
 * Không tạo thêm biến g_ms ở file này.
 * ============================================================ */

extern volatile uint32_t g_ms;


/* ============================================================
 * Cortex-M4 Exception Handlers
 * ============================================================ */


/**
  * @brief NMI Handler
  */
void NMI_Handler(void)
{
    while (1)
    {
    }
}


/**
  * @brief Hard Fault Handler
  */
void HardFault_Handler(void)
{
    while (1)
    {
    }
}


/**
  * @brief Memory Management Fault Handler
  */
void MemManage_Handler(void)
{
    while (1)
    {
    }
}


/**
  * @brief Bus Fault Handler
  */
void BusFault_Handler(void)
{
    while (1)
    {
    }
}


/**
  * @brief Usage Fault Handler
  */
void UsageFault_Handler(void)
{
    while (1)
    {
    }
}


/**
  * @brief SVCall Handler
  */
void SVC_Handler(void)
{
}


/**
  * @brief Debug Monitor Handler
  */
void DebugMon_Handler(void)
{
}


/**
  * @brief PendSV Handler
  */
void PendSV_Handler(void)
{
}


/* ============================================================
 * SysTick Handler
 *
 * Đây là SysTick_Handler DUY NHẤT trong project.
 *
 * Không được tạo SysTick_Handler trong main.c.
 *
 * Mỗi interrupt = 1 ms
 * ============================================================ */

void SysTick_Handler(void)
{
    g_ms++;
}
