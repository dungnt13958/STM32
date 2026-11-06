#ifndef INC_MAZE_H_
#define INC_MAZE_H_

#include <stdint.h>
#include <stdbool.h>

/* ================= KÍCH THƯỚC MÊ CUNG 16x16 & VÙNG ĐÍCH ================= */
#define MAZE_SIZE           16U
#define MAZE_GOAL_MIN       7U      // Tâm mê cung 16x16: các ô (7,7), (7,8), (8,7), (8,8)
#define MAZE_GOAL_MAX       8U
#define DIST_INF            0xFFFFU

/* ================= BITMASK TRẠNG THÁI TƯỜNG ================= */
#define WALL_NORTH          (1U << 0)
#define WALL_EAST           (1U << 1)
#define WALL_SOUTH          (1U << 2)
#define WALL_WEST           (1U << 3)
#define CELL_VISITED        (1U << 4)

typedef enum {
    DIR_NORTH = 0,
    DIR_EAST  = 1,
    DIR_SOUTH = 2,
    DIR_WEST  = 3
} Direction_t;

extern uint8_t     maze_walls[MAZE_SIZE][MAZE_SIZE];
extern uint16_t    maze_dist[MAZE_SIZE][MAZE_SIZE];
extern uint8_t     robot_x;
extern uint8_t     robot_y;
extern Direction_t robot_dir;

void Maze_Init(void);
void Maze_Update_Walls(void);
void Maze_Flood_Fill(uint8_t target_min_x,
                     uint8_t target_min_y,
                     uint8_t target_max_x,
                     uint8_t target_max_y);
Direction_t Maze_Get_Best_Direction(void);
void Maze_Execute_Move(Direction_t next_dir);
bool Maze_Is_At_Goal(void);
void Maze_Explore_To_Goal(void);

#endif /* INC_MAZE_H_ */
