/**
 * @file    maze.c
 * @brief   Giải thuật Look-Ahead Flood Fill (Nhận diện trước 1 ô) cho mê cung 16x16:
 *          - Xuất phát tại (0,0) hướng Bắc (DIR_NORTH), đích tại trung tâm (7,7)-(8,8).
 *          - Khi xe ở ô (robot_x, robot_y) hướng robot_dir, cảm biến IR xác định 3 vách
 *            của ô phía trước 1 bước: (robot_x + dx[robot_dir], robot_y + dy[robot_dir]).
 */

#include "maze.h"
#include "motion.h"
#include "ir_sensor.h"
#include "system_ui.h"
#include "motor.h"

#include <stdbool.h>
#include <stdint.h>

uint8_t maze_walls[MAZE_SIZE][MAZE_SIZE];
uint16_t maze_dist[MAZE_SIZE][MAZE_SIZE];

uint8_t robot_x = 0U;
uint8_t robot_y = 0U;
Direction_t robot_dir = DIR_NORTH;

/* Hàng đợi BFS tĩnh cho Flood Fill */
static uint8_t queue_x[MAZE_SIZE * MAZE_SIZE];
static uint8_t queue_y[MAZE_SIZE * MAZE_SIZE];
static uint16_t q_head = 0U;
static uint16_t q_tail = 0U;

/* Vector dịch chuyển theo 4 hướng: Bắc (0), Đông (1), Nam (2), Tây (3) */
static const int8_t dx[4] = { 0,  1,  0, -1 };
static const int8_t dy[4] = { 1,  0, -1,  0 };
static const uint8_t wall_bit_dir[4] = {
    WALL_NORTH,
    WALL_EAST,
    WALL_SOUTH,
    WALL_WEST
};

static bool Maze_Is_Inside(int16_t x, int16_t y)
{
    return (x >= 0 && x < (int16_t)MAZE_SIZE &&
            y >= 0 && y < (int16_t)MAZE_SIZE);
}

static uint8_t Maze_Opposite_Direction(uint8_t dir)
{
    return (uint8_t)((dir + 2U) % 4U);
}

/**
 * @brief Ghi một bức tường vào cả hai ô chia sẻ cạnh đó.
 */
static void Maze_Set_Wall(uint8_t x, uint8_t y, Direction_t direction)
{
    uint8_t dir = (uint8_t)direction;
    int16_t nx;
    int16_t ny;

    if (x >= MAZE_SIZE || y >= MAZE_SIZE || dir >= 4U)
        return;

    maze_walls[x][y] |= wall_bit_dir[dir];

    nx = (int16_t)x + dx[dir];
    ny = (int16_t)y + dy[dir];

    if (Maze_Is_Inside(nx, ny))
    {
        uint8_t opposite = Maze_Opposite_Direction(dir);
        maze_walls[(uint8_t)nx][(uint8_t)ny] |= wall_bit_dir[opposite];
    }
}

/**
 * @brief Thiết lập lại tường biên 16x16 và vách cố định tại ô xuất phát (0,0).
 */
static void Maze_Reset_Boundary_And_Start_Walls(void)
{
    for (uint8_t x = 0U; x < MAZE_SIZE; x++)
    {
        for (uint8_t y = 0U; y < MAZE_SIZE; y++)
        {
            maze_walls[x][y] = 0U;
            maze_dist[x][y]  = DIST_INF;

            if (y == (MAZE_SIZE - 1U)) maze_walls[x][y] |= WALL_NORTH;
            if (x == (MAZE_SIZE - 1U)) maze_walls[x][y] |= WALL_EAST;
            if (y == 0U)               maze_walls[x][y] |= WALL_SOUTH;
            if (x == 0U)               maze_walls[x][y] |= WALL_WEST;
        }
    }

    /* Ô xuất phát (0,0) luôn biết trước: Tây, Nam, Đông đóng; Bắc mở */
    Maze_Set_Wall(0U, 0U, DIR_WEST);
    Maze_Set_Wall(0U, 0U, DIR_SOUTH);
    Maze_Set_Wall(0U, 0U, DIR_EAST);
    maze_walls[0][0] |= CELL_VISITED;
}

/**
 * @brief Lấy mẫu liên tục cảm biến IR để bộ lọc EMA (70% cũ + 30% mới)
 *        hội tụ ổn định khi xe đang đứng yên trước khi chốt cờ tường.
 */
static void Maze_Refresh_IR(void)
{
    sensor.Active = true;
    for (uint8_t i = 0U; i < 10U; i++)
    {
        IR_Update();
        Motion_Delay_MS(2U);
    }
}

/**
 * @brief Khởi tạo bản đồ 16x16, tường biên và ô xuất phát (0,0).
 */
void Maze_Init(void)
{
    Maze_Reset_Boundary_And_Start_Walls();

    robot_x   = 0U;
    robot_y   = 0U;
    robot_dir = DIR_NORTH;

    q_head = 0U;
    q_tail = 0U;

    Maze_Flood_Fill(MAZE_GOAL_MIN, MAZE_GOAL_MIN,
                    MAZE_GOAL_MAX, MAZE_GOAL_MAX);
}

/**
 * @brief Quét và cập nhật tường cho Ô PHÍA TRƯỚC 1 BƯỚC (Look-Ahead 1 Cell).
 *        Ví dụ: Đứng ở (0,0) hướng Bắc -> Xác định 3 vách Trước/Phải/Trái của ô (0,1).
 */
void Maze_Update_Walls(void)
{
    if (robot_x >= MAZE_SIZE || robot_y >= MAZE_SIZE)
        return;

    maze_walls[robot_x][robot_y] |= CELL_VISITED;

    /* Nếu ngay trước mặt ô hiện tại (robot_x, robot_y) có tường chắn thì không nhìn xuyên qua được */
    if ((maze_walls[robot_x][robot_y] & wall_bit_dir[(uint8_t)robot_dir]) != 0U)
        return;

    /* Tọa độ ô phía trước 1 bước mà cảm biến đang chiếu tới */
    int16_t ahead_x = (int16_t)robot_x + dx[(uint8_t)robot_dir];
    int16_t ahead_y = (int16_t)robot_y + dy[(uint8_t)robot_dir];

    if (!Maze_Is_Inside(ahead_x, ahead_y))
        return;

    uint8_t tx = (uint8_t)ahead_x;
    uint8_t ty = (uint8_t)ahead_y;

    /* Làm mới bộ lọc EMA của IR trước khi đọc trạng thái tường */
    Maze_Refresh_IR();

    Direction_t dir_front = robot_dir;
    Direction_t dir_right = (Direction_t)(((uint8_t)robot_dir + 1U) % 4U);
    Direction_t dir_left  = (Direction_t)(((uint8_t)robot_dir + 3U) % 4U);

    /* Gán 3 vách Trước / Phải / Trái vừa đọc được vào ô phía trước (tx, ty) */
    if (sensor.SeeFrontWall)
        Maze_Set_Wall(tx, ty, dir_front);

    if (sensor.SeeRightWall)
        Maze_Set_Wall(tx, ty, dir_right);

    if (sensor.SeeLeftWall)
        Maze_Set_Wall(tx, ty, dir_left);

    /* Đánh dấu ô phía trước (tx, ty) đã được quét tường */
    maze_walls[tx][ty] |= CELL_VISITED;
}

/**
 * @brief Tính lại bảng khoảng cách tới vùng đích bằng BFS Flood Fill.
 */
void Maze_Flood_Fill(uint8_t target_min_x,
                     uint8_t target_min_y,
                     uint8_t target_max_x,
                     uint8_t target_max_y)
{
    uint16_t x;
    uint16_t y;

    for (x = 0U; x < MAZE_SIZE; x++)
    {
        for (y = 0U; y < MAZE_SIZE; y++)
            maze_dist[x][y] = DIST_INF;
    }

    q_head = 0U;
    q_tail = 0U;

    if (target_min_x > target_max_x || target_min_y > target_max_y ||
        target_max_x >= MAZE_SIZE || target_max_y >= MAZE_SIZE)
    {
        return;
    }

    /* Đưa 4 ô trung tâm (7,7), (7,8), (8,7), (8,8) vào hàng đợi với dist = 0 */
    for (x = target_min_x; x <= target_max_x; x++)
    {
        for (y = target_min_y; y <= target_max_y; y++)
        {
            if (q_tail >= (MAZE_SIZE * MAZE_SIZE))
                return;

            maze_dist[x][y] = 0U;
            queue_x[q_tail] = (uint8_t)x;
            queue_y[q_tail] = (uint8_t)y;
            q_tail++;
        }
    }

    while (q_head < q_tail)
    {
        uint8_t cx = queue_x[q_head];
        uint8_t cy = queue_y[q_head];
        uint16_t cur_d;
        uint8_t d;

        q_head++;
        cur_d = maze_dist[cx][cy];

        for (d = 0U; d < 4U; d++)
        {
            int16_t nx;
            int16_t ny;
            uint8_t ux;
            uint8_t uy;

            if ((maze_walls[cx][cy] & wall_bit_dir[d]) != 0U)
                continue;

            nx = (int16_t)cx + dx[d];
            ny = (int16_t)cy + dy[d];

            if (!Maze_Is_Inside(nx, ny))
                continue;

            ux = (uint8_t)nx;
            uy = (uint8_t)ny;

            if (maze_dist[ux][uy] == DIST_INF)
            {
                maze_dist[ux][uy] = (uint16_t)(cur_d + 1U);

                if (q_tail < (MAZE_SIZE * MAZE_SIZE))
                {
                    queue_x[q_tail] = ux;
                    queue_y[q_tail] = uy;
                    q_tail++;
                }
            }
        }
    }
}

/**
 * @brief Chọn hướng đi có khoảng cách tới đích nhỏ nhất.
 *        Khi bằng nhau, ưu tiên: Đi thẳng -> Rẽ phải -> Rẽ trái -> Quay đầu.
 */
Direction_t Maze_Get_Best_Direction(void)
{
    uint16_t min_dist = DIST_INF;
    Direction_t best_dir = robot_dir;
    Direction_t check_order[4];
    uint8_t i;

    if (robot_x >= MAZE_SIZE || robot_y >= MAZE_SIZE)
        return robot_dir;

    check_order[0] = robot_dir;
    check_order[1] = (Direction_t)(((uint8_t)robot_dir + 1U) % 4U);
    check_order[2] = (Direction_t)(((uint8_t)robot_dir + 3U) % 4U);
    check_order[3] = (Direction_t)(((uint8_t)robot_dir + 2U) % 4U);

    for (i = 0U; i < 4U; i++)
    {
        uint8_t d = (uint8_t)check_order[i];
        int16_t nx;
        int16_t ny;
        uint16_t candidate_dist;

        if (d >= 4U)
            continue;

        if ((maze_walls[robot_x][robot_y] & wall_bit_dir[d]) != 0U)
            continue;

        nx = (int16_t)robot_x + dx[d];
        ny = (int16_t)robot_y + dy[d];

        if (!Maze_Is_Inside(nx, ny))
            continue;

        candidate_dist = maze_dist[(uint8_t)nx][(uint8_t)ny];

        if (candidate_dist < min_dist)
        {
            min_dist = candidate_dist;
            best_dir = check_order[i];
        }
    }

    return best_dir;
}

/**
 * @brief Xoay xe tại chỗ về hướng next_dir (chưa tiến thẳng).
 */
static void Maze_Turn_To_Direction(Direction_t next_dir)
{
    int8_t turn_diff = (int8_t)next_dir - (int8_t)robot_dir;

    if (turn_diff == 1 || turn_diff == -3)
    {
        Turn_Right_90();
        Motion_Delay_MS(40U);
    }
    else if (turn_diff == -1 || turn_diff == 3)
    {
        Turn_Left_90();
        Motion_Delay_MS(40U);
    }
    else if (turn_diff == 2 || turn_diff == -2)
    {
        Turn_Around_180();
        Motion_Delay_MS(40U);
    }

    robot_dir = next_dir;
}

/**
 * @brief Thực thi xoay (nếu cần) và tiến 1 ô (180mm).
 */
void Maze_Execute_Move(Direction_t next_dir)
{
    int16_t next_x;
    int16_t next_y;

    if (robot_x >= MAZE_SIZE || robot_y >= MAZE_SIZE || (uint8_t)next_dir >= 4U)
        return;

    if (next_dir != robot_dir)
    {
        Maze_Turn_To_Direction(next_dir);
    }

    if ((maze_walls[robot_x][robot_y] & wall_bit_dir[(uint8_t)robot_dir]) != 0U)
    {
        Motor_Short_Brake();
        return;
    }

    next_x = (int16_t)robot_x + dx[(uint8_t)robot_dir];
    next_y = (int16_t)robot_y + dy[(uint8_t)robot_dir];

    if (!Maze_Is_Inside(next_x, next_y))
    {
        Motor_Short_Brake();
        return;
    }

    Move_Forward_One_Cell();

    robot_x = (uint8_t)next_x;
    robot_y = (uint8_t)next_y;
}

/**
 * @brief Kiểm tra chuột đã vào vùng đích trung tâm (7,7)-(8,8) hay chưa.
 */
bool Maze_Is_At_Goal(void)
{
    return (robot_x >= MAZE_GOAL_MIN && robot_x <= MAZE_GOAL_MAX &&
            robot_y >= MAZE_GOAL_MIN && robot_y <= MAZE_GOAL_MAX);
}

/**
 * @brief Vòng lặp thám hiểm mê cung với cảm biến nhìn trước 1 ô (Look-Ahead):
 *        - Quét ô phía trước 1 bước theo hướng đang nhìn.
 *        - Nếu cần đổi hướng: Xoay trước -> Quét luôn ô phía trước của hướng mới ->
 *          Tính lại Flood Fill -> Khi hướng tối ưu trùng hướng mũi xe thì mới tiến vào.
 */
void Maze_Explore_To_Goal(void)
{
    Maze_Flood_Fill(MAZE_GOAL_MIN, MAZE_GOAL_MIN,
                    MAZE_GOAL_MAX, MAZE_GOAL_MAX);

    while (!Maze_Is_At_Goal())
    {
        Direction_t next_dir = robot_dir;

        /* Cho phép tối đa 6 lượt thử để dù xe xoay qua cả 4 hướng vẫn kịp quét hướng cuối */
        for (uint8_t look_try = 0U; look_try < 6U; look_try++)
        {
            /* 1. Nếu trước mặt ô hiện tại không bị chắn, quét tường của ô phía trước 1 bước (ax, ay) */
            if ((maze_walls[robot_x][robot_y] & wall_bit_dir[(uint8_t)robot_dir]) == 0U)
            {
                int16_t ax = (int16_t)robot_x + dx[(uint8_t)robot_dir];
                int16_t ay = (int16_t)robot_y + dy[(uint8_t)robot_dir];

                /* Chỉ quét nếu ô phía trước chưa từng được quét để tiết kiệm thời gian dừng */
                if (Maze_Is_Inside(ax, ay) &&
                    ((maze_walls[(uint8_t)ax][(uint8_t)ay] & CELL_VISITED) == 0U))
                {
                    Maze_Update_Walls();
                }
            }

            /* 2. Cập nhật ma trận khoảng cách Flood Fill với dữ liệu tường mới nhất */
            Maze_Flood_Fill(MAZE_GOAL_MIN, MAZE_GOAL_MIN,
                            MAZE_GOAL_MAX, MAZE_GOAL_MAX);

            /* Tự phục hồi nếu cảm biến từng bắt nhầm tường ảo làm tắc đường (DIST_INF) */
            if (maze_dist[robot_x][robot_y] == DIST_INF)
            {
                Maze_Reset_Boundary_And_Start_Walls();
                Maze_Update_Walls();
                Maze_Flood_Fill(MAZE_GOAL_MIN, MAZE_GOAL_MIN,
                                MAZE_GOAL_MAX, MAZE_GOAL_MAX);
            }

            /* 3. Tìm hướng tối ưu nhất từ ô hiện tại */
            next_dir = Maze_Get_Best_Direction();

            /* Nếu hướng tối ưu trùng với hướng mũi xe đang nhìn (và đã quét xong ô phía trước) -> Tiến! */
            if (next_dir == robot_dir)
            {
                break;
            }

            /* Nếu cần rẽ sang hướng khác -> Xoay mũi xe sang hướng đó rồi lặp lại bước 1 để nhìn trước */
            Maze_Turn_To_Direction(next_dir);
        }

        /* 4. Bảo vệ an toàn và tiến thẳng 1 ô (180mm) sang ô tiếp theo */
        if ((uint8_t)robot_dir >= 4U ||
            (maze_walls[robot_x][robot_y] & wall_bit_dir[(uint8_t)robot_dir]) != 0U)
        {
            Motor_Short_Brake();
            return;
        }

        Maze_Execute_Move(robot_dir);
        Motion_Delay_MS(30U);
    }

    /* Đã chạm vùng đích trung tâm (7,7)-(8,8) */
    Motor_Short_Brake();
    UI_All_LEDs_On();
}
