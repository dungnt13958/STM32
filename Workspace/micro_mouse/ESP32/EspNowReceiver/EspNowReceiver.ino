/* ESP32 WROOM DevKit V4 - ESP-NOW receiver, Serial Monitor 115200.
 * Arduino-ESP32 2.x / 3.x. No UART wires needed on this board.
 * Sender and receiver must use the same Wi-Fi channel and CSV format.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

static constexpr uint8_t ESPNOW_CHANNEL = 1;
static const uint8_t expectedMac[6] = {0xA8, 0x42, 0xE3, 0xCA, 0xF9, 0x88};
struct ReceivedFrame
{
    uint8_t sender[6];
    char csv[80];
};
static QueueHandle_t frames;
static uint32_t lastReceivedMs = 0;
static uint32_t lastWarningMs = 0;

static void requireOK(esp_err_t result, const char *operation)
{
    if (result == ESP_OK) return;
    Serial.printf("Loi %s: %s\n", operation, esp_err_to_name(result));
    while (true) delay(1000);
}

// Wi-Fi task callback: copy data only; parsing and printing happen in loop().
#if ESP_IDF_VERSION_MAJOR >= 5
static void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    const uint8_t *mac = info->src_addr;
#else
static void onReceive(const uint8_t *mac, const uint8_t *data, int len)
{
#endif
    if (len <= 0 || len >= 80 || memchr(data, '\0', len) != nullptr) return;
    ReceivedFrame frame = {};
    memcpy(frame.sender, mac, sizeof frame.sender);
    memcpy(frame.csv, data, len);
    frame.csv[len] = '\0';
    // Never wait in the Wi-Fi task; drop a frame if the queue is full.
    (void)xQueueSend(frames, &frame, 0);
}

void setup()
{
    Serial.begin(115200);
    frames = xQueueCreate(10, sizeof(ReceivedFrame));
    if (frames == nullptr)
    {
        Serial.println("Khong tao duoc hang doi.");
        while (true) delay(1000);
    }
    if (!WiFi.mode(WIFI_STA))
    {
        Serial.println("Khong khoi dong duoc Wi-Fi STA.");
        while (true) delay(1000);
    }
    requireOK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE), "set channel");
    uint8_t actualMac[6];
    requireOK(esp_wifi_get_mac(WIFI_IF_STA, actualMac), "get STA MAC");
    Serial.printf("MAC STA bo nhan: %s | Kenh: %u\n",
                  WiFi.macAddress().c_str(), ESPNOW_CHANNEL);
    if (memcmp(actualMac, expectedMac, 6) != 0)
        Serial.println("MAC khac A8:42:E3:CA:F9:88: sua receiverMac trong sketch bo gui.");
    requireOK(esp_now_init(), "esp_now_init");
    requireOK(esp_now_register_recv_cb(onReceive), "register receive callback");
    Serial.println("Dang cho du lieu ESP-NOW...");
}

void loop()
{
    ReceivedFrame frame;
    while (xQueueReceive(frames, &frame, 0) == pdTRUE)
    {
        long left, right;
        float yaw;
        char extra;
        if (sscanf(frame.csv, "%ld,%ld,%f %c", &left, &right, &yaw, &extra) != 3 ||
            !isfinite(yaw) || yaw < -180.0f || yaw > 180.0f)
        {
            Serial.println("Bo qua goi ESP-NOW sai dinh dang.");
            continue;
        }
        lastReceivedMs = millis();
        Serial.printf("Tu %02X:%02X:%02X:%02X:%02X:%02X | "
                      "Encoder trai: %ld | Encoder phai: %ld | Yaw: %.2f do\n",
                      frame.sender[0], frame.sender[1], frame.sender[2],
                      frame.sender[3], frame.sender[4], frame.sender[5], left, right, yaw);
    }
    uint32_t now = millis();
    if (now - lastReceivedMs >= 2000U && now - lastWarningMs >= 2000U)
    {
        Serial.println("Chua nhan du lieu hop le: kiem tra nguon bo gui, MAC va kenh Wi-Fi.");
        lastWarningMs = now;
    }
    delay(1);
}
