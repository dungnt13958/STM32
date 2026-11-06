
#ifndef FLOODFILL_H
#define FLOODFILL_H

#include <stdint.h>

#define FF_MAZE_SIZE 16U

/* Quy ước hướng tuyệt đối */
typedef enum
{
    FF_DIR_NORTH = 0,
    FF_DIR_EAST,
    FF_DIR_SOUTH,
    FF_DIR_WEST
} FF_Direction;

/* Bit đánh dấu tường của từng ô */
#define FF_WALL_N (1U << 0)
#define FF_WALL_E (1U << 1)
#define FF_WALL_S (1U << 2)
#define FF_WALL_W (1U << 3)

/* Lệnh di chuyển tương đối so với heading hiện tại */
typedef enum
{
    FF_MOVE_FORWARD = 0,
    FF_MOVE_RIGHT,
    FF_MOVE_BACK,
    FF_MOVE_LEFT,
    FF_MOVE_STOP
} FF_Move;

FF_Move FF_MazeChooseMove(
    const uint8_t walls[FF_MAZE_SIZE][FF_MAZE_SIZE],
    uint8_t robot_x,
    uint8_t robot_y,
    uint8_t goal_x,
    uint8_t goal_y,
    FF_Direction heading
);

#endif
