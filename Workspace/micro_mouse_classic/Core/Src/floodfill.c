
#include "floodfill.h"

#define FF_INF 0xFFFFU

static const int8_t dx[4] = { 0, 1, 0, -1 };
static const int8_t dy[4] = {-1, 0, 1,  0};

static const uint8_t wall_bit[4] =
{
    FF_WALL_N,
    FF_WALL_E,
    FF_WALL_S,
    FF_WALL_W
};

static uint8_t FF_Opposite(uint8_t dir)
{
    return (uint8_t)((dir + 2U) % 4U);
}

static uint8_t FF_IsInside(int x, int y)
{
    return (x >= 0 && x < (int)FF_MAZE_SIZE &&
            y >= 0 && y < (int)FF_MAZE_SIZE);
}

/* Kiểm tra có thể đi qua cạnh giữa hai ô hay không */
static uint8_t FF_IsOpen(
    const uint8_t walls[FF_MAZE_SIZE][FF_MAZE_SIZE],
    uint8_t x,
    uint8_t y,
    uint8_t dir)
{
    int nx = (int)x + dx[dir];
    int ny = (int)y + dy[dir];

    if (!FF_IsInside(nx, ny))
        return 0U;

    /* Cả hai phía của cạnh đều không được đánh dấu có tường */
    if (walls[y][x] & wall_bit[dir])
        return 0U;

    if (walls[ny][nx] & wall_bit[FF_Opposite(dir)])
        return 0U;

    return 1U;
}

FF_Move FF_MazeChooseMove(
    const uint8_t walls[FF_MAZE_SIZE][FF_MAZE_SIZE],
    uint8_t robot_x,
    uint8_t robot_y,
    uint8_t goal_x,
    uint8_t goal_y,
    FF_Direction heading)
{
    uint16_t distance[FF_MAZE_SIZE][FF_MAZE_SIZE];
    uint16_t queue[FF_MAZE_SIZE * FF_MAZE_SIZE];

    uint16_t head = 0U;
    uint16_t tail = 0U;

    if (robot_x >= FF_MAZE_SIZE ||
        robot_y >= FF_MAZE_SIZE ||
        goal_x >= FF_MAZE_SIZE ||
        goal_y >= FF_MAZE_SIZE ||
        heading > FF_DIR_WEST)
    {
        return FF_MOVE_STOP;
    }

    /* Bước 1: Khởi tạo toàn bộ khoảng cách là vô cùng */
    for (uint8_t y = 0U; y < FF_MAZE_SIZE; y++)
    {
        for (uint8_t x = 0U; x < FF_MAZE_SIZE; x++)
        {
            distance[y][x] = FF_INF;
        }
    }

    /* Khoảng cách từ đích đến chính nó bằng 0 */
    distance[goal_y][goal_x] = 0U;

    queue[tail++] = (uint16_t)(goal_y * FF_MAZE_SIZE + goal_x);

    /*
     * Bước 2: Flood Fill bằng BFS từ đích về các ô khác.
     * Ô chưa có bit tường được coi là cạnh có thể đi qua.
     */
    while (head < tail)
    {
        uint16_t cell = queue[head++];

        uint8_t x = (uint8_t)(cell % FF_MAZE_SIZE);
        uint8_t y = (uint8_t)(cell / FF_MAZE_SIZE);

        for (uint8_t dir = 0U; dir < 4U; dir++)
        {
            if (!FF_IsOpen(walls, x, y, dir))
                continue;

            uint8_t nx = (uint8_t)((int)x + dx[dir]);
            uint8_t ny = (uint8_t)((int)y + dy[dir]);

            if (distance[ny][nx] == FF_INF)
            {
                distance[ny][nx] = distance[y][x] + 1U;

                if (tail < FF_MAZE_SIZE * FF_MAZE_SIZE)
                {
                    queue[tail++] =
                        (uint16_t)(ny * FF_MAZE_SIZE + nx);
                }
            }
        }
    }

    /* Đã đến đích */
    if (robot_x == goal_x && robot_y == goal_y)
        return FF_MOVE_STOP;

    /*
     * Bước 3: Chọn ô kề có khoảng cách nhỏ nhất.
     * Ưu tiên khi bằng nhau: đi thẳng, trái, phải, quay lại.
     */
    uint8_t preferred[4];

    preferred[0] = (uint8_t)heading;
    preferred[1] = (uint8_t)((heading + 3U) % 4U);
    preferred[2] = (uint8_t)((heading + 1U) % 4U);
    preferred[3] = (uint8_t)((heading + 2U) % 4U);

    uint16_t best_distance = FF_INF;
    uint8_t best_dir = 0U;
    uint8_t found = 0U;

    for (uint8_t i = 0U; i < 4U; i++)
    {
        uint8_t dir = preferred[i];

        if (!FF_IsOpen(walls, robot_x, robot_y, dir))
            continue;

        uint8_t nx = (uint8_t)((int)robot_x + dx[dir]);
        uint8_t ny = (uint8_t)((int)robot_y + dy[dir]);

        if (distance[ny][nx] < best_distance)
        {
            best_distance = distance[ny][nx];
            best_dir = dir;
            found = 1U;
        }
    }

    if (!found || best_distance == FF_INF)
        return FF_MOVE_STOP;

    /* Chuyển hướng tuyệt đối thành chuyển động tương đối */
    uint8_t relative =
        (uint8_t)((best_dir + 4U - (uint8_t)heading) % 4U);

    switch (relative)
    {
        case 0U: return FF_MOVE_FORWARD;
        case 1U: return FF_MOVE_RIGHT;
        case 2U: return FF_MOVE_BACK;
        case 3U: return FF_MOVE_LEFT;
        default: return FF_MOVE_STOP;
    }
}
