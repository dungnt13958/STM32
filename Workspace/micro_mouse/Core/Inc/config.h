
#ifndef CONFIG_H
#define CONFIG_H

/* Tham số hiệu chuẩn cảm biến, đơn vị khoảng cách mm */
#define WALL_TARGET_L 45.0f
#define WALL_TARGET_R 45.0f
#define FRONT_REFERENCE 80.0f

/* Ngưỡng phát hiện tường */
#define LEFT_THRESHOLD  80
#define RIGHT_THRESHOLD 80
#define FRONT_THRESHOLD 80

#define FRONT_WALL_RELIABILITY_LIMIT 100

/* Tham số PD: thay bằng thông số thực tế của robot */
#define STEERING_KP 1.0f
#define STEERING_KD 0.0f
#define STEERING_ADJUST_LIMIT 100.0f

#endif
