#include "flash_storage.h"
#include "stm32f411xe.h"

#define FLASH_KEY1               0x45670123U
#define FLASH_KEY2               0xCDEF89ABU

void Flash_Save_Maze(uint8_t *maze_data, uint16_t length) {
    // 1. Mở khóa bộ nhớ Flash
    if ((FLASH->CR & FLASH_CR_LOCK) != 0) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }

    // 2. Chờ Flash rảnh
    while (FLASH->SR & FLASH_SR_BSY);

    // 3. Xóa Sector 7
    FLASH->CR |= FLASH_CR_SER;
    FLASH->CR |= (MAZE_FLASH_SECTOR << 3); // Chọn SNB = 7
    FLASH->CR |= FLASH_CR_STRT;
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR &= ~FLASH_CR_SER;

    // 4. Ghi dữ liệu (Cấu hình PSIZE = 00 ghi từng byte 8-bit)
    FLASH->CR &= ~FLASH_CR_PSIZE;
    FLASH->CR |= FLASH_CR_PG;

    for (uint16_t i = 0; i < length; i++) {
        *(__IO uint8_t*)(MAZE_FLASH_ADDR + i) = maze_data[i];
        while (FLASH->SR & FLASH_SR_BSY);
    }

    // 5. Khóa lại Flash
    FLASH->CR &= ~FLASH_CR_PG;
    FLASH->CR |= FLASH_CR_LOCK;
}

void Flash_Load_Maze(uint8_t *maze_data, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        maze_data[i] = *(__IO uint8_t*)(MAZE_FLASH_ADDR + i);
    }
}
