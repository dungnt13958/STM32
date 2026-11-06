#include "main.h"
#include "sensors.h" // Chỉ gọi thư viện sensors.h
#include <stdio.h>

// ==========================================
// 1. TỰ VIẾT CÁC HÀM ADAPTER ĐỂ CUNG CẤP DỮ LIỆU ADC THÔ CHO SENSORS
// ==========================================
// Giả sử bạn đọc trực tiếp từ các chân ADC1 (PA3, PA2, PA5, PA4) tương ứng với L, FL, FR, R.
static uint16_t Read_ADC_Channel(uint8_t channel) {
    ADC1->SQR3 = channel;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return (uint16_t)ADC1->DR;
}

int adc_get_L(void)  { return (int)Read_ADC_Channel(3); }  // PA3 -> Hông Trái
int adc_get_FL(void) { return (int)Read_ADC_Channel(2); }  // PA2 -> Trước Trái
int adc_get_FR(void) { return (int)Read_ADC_Channel(5); }  // PA5 -> Trước Phải
int adc_get_R(void)  { return (int)Read_ADC_Channel(4); }  // PA4 -> Hông Phải

// Hàm nhấp nháy LED theo yêu cầu của sensors.c
void blinkLedsFor(uint32_t ms) {
    // Thêm code bật/tắt LED của bạn ở đây, ví dụ:
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_Delay(ms);
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
}

// ==========================================
// 2. HÀM KHỞI TẠO PHẦN CỨNG ADC1 CHO RIÊNG BẠN
// ==========================================
void My_ADC_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Cấu hình chân PA2, PA3, PA4, PA5 ở chế độ Analog (MODER = 11)
    GPIOA->MODER |= (3U << (2 * 2)) | (3U << (3 * 2)) |
                    (3U << (4 * 2)) | (3U << (5 * 2));

    // Bật ADC1
    ADC1->CR2 = ADC_CR2_ADON;
}

// ==========================================
// 3. HÀM MAIN CHÍNH
// ==========================================
int main(void) {
    // Khởi tạo phần cứng cơ bản
    HAL_Init();
    SystemClock_Config();

    // Khởi tạo ADC để đọc cảm biến
    My_ADC_Init();

    // Khởi tạo đối tượng sensors từ thư viện sensors.h
    sensors_init(&sensors);

    // Tiến hành hiệu chuẩn cảm biến hông (calibrate scale)
    // Lưu ý: Đặt robot ở vị trí thích hợp khi gọi hàm này
    calibrate_scales(&sensors);

    while (1) {
        // Cập nhật toàn bộ trạng thái cảm biến, lọc nhiễu và tính toán sai số bẻ lái
        update(&sensors);

        // --- BẠN CÓ THỂ LẤY VÀ SỬ DỤNG CÁC DỮ LIỆU SAU: ---

        // 1. Đọc giá trị giá trị đã scale của từng mắt cảm biến
        int left_val      = sensors.lss.value;
        int front_left_val= sensors.lfs.value;
        int front_right_val= sensors.rfs.value;
        int right_val     = sensors.rss.value;

        // 2. Kiểm tra xem xung quanh có tường hay không
        bool has_left_wall  = sensors.see_left_wall;
        bool has_right_wall = sensors.see_right_wall;
        bool has_front_wall = sensors.see_front_wall;

        // 3. Lấy hệ số điều chỉnh hướng đi (Steering Feedback) để lái robot đi thẳng
        float steering_correction = get_steering_feedback(&sensors);

        // In giá trị ra màn hình debug (nếu có cấu hình UART printf)
        print_values(&sensors);

        // Độ trễ vòng lặp (ví dụ 2ms - 5ms cho vòng lặp điều khiển thời gian thực)
        HAL_Delay(2);
    }
}
