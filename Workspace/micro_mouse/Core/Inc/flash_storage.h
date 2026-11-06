#ifndef INC_FLASH_STORAGE_H_
#define INC_FLASH_STORAGE_H_

#include <stdint.h>

#define MAZE_FLASH_SECTOR       7
#define MAZE_FLASH_ADDR         0x08060000 // Địa chỉ bắt đầu Sector 7

void Flash_Save_Maze(uint8_t *maze_data, uint16_t length);
void Flash_Load_Maze(uint8_t *maze_data, uint16_t length);

#endif /* INC_FLASH_STORAGE_H_ */
