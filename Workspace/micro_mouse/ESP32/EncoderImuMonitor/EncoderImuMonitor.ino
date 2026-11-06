/* ESP32 WROOM DevKit V4 - Arduino IDE
 * STM32 PA9 (TX) -> ESP32 GPIO16 (RX2)
 * STM32 PA10 (RX) <- ESP32 GPIO17 (TX2), optional for receive-only use
 * STM32 GND      -> ESP32 GND
 * Input: left_count,right_count,yaw_deg\r\n, 115200 baud 8N1.
 * USB Serial Monitor: 115200 baud.
 */
#include <Arduino.h>
#include <stdio.h>
#include <math.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

#define RX_PIN 16
#define TX_PIN 17

static constexpr uint8_t ESPNOW_CHANNEL = 1;
static const uint8_t receiverMac[6] = {0xA8, 0x42, 0xE3, 0xCA, 0xF9, 0x88};

static void requireOK(esp_err_t result, const char *operation)
{
    if (result == ESP_OK) return;
    Serial.printf("Loi %s: %s\n", operation, esp_err_to_name(result));
    while (true) delay(1000);
}

HardwareSerial stm32Serial(2);
static char line[80];
static size_t lineLength = 0;
static bool discardLine = false;
static uint32_t lastReceivedMs = 0;
static uint32_t lastWarningMs = 0;

static void printFrame(const char *frame)
{
    long left, right;
    float yaw;
    char extra;
    // A fourth field or trailing non-whitespace characters make a frame invalid.
    if (sscanf(frame, "%ld,%ld,%f %c", &left, &right, &yaw, &extra) != 3 ||
        !isfinite(yaw) || yaw < -180.0f || yaw > 180.0f)
    {
        Serial.print("Khung du lieu khong hop le: ");
        Serial.println(frame);
        return;
    }

    lastReceivedMs = millis();
    Serial.printf("Encoder trai: %ld | Encoder phai: %ld | Yaw: %.2f do\n",
                  left, right, yaw);
    // Preserve the original CSV; the ESP-NOW packet boundary replaces '\n'.
    // ESP_OK means queued for transmission, not confirmed by the receiver.
    esp_err_t result = esp_now_send(receiverMac,
                                   reinterpret_cast<const uint8_t *>(frame),
                                   strlen(frame));
    if (result != ESP_OK)
        Serial.printf("Khong gui duoc ESP-NOW: %s\n", esp_err_to_name(result));
}

void setup()
{
    Serial.begin(115200);
    if (!WiFi.mode(WIFI_STA))
    {
        Serial.println("Khong khoi dong duoc Wi-Fi STA.");
        while (true) delay(1000);
    }
    requireOK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE), "set channel");
    requireOK(esp_now_init(), "esp_now_init");
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, receiverMac, sizeof receiverMac);
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    requireOK(esp_now_add_peer(&peer), "esp_now_add_peer");
    Serial.printf("MAC STA bo gui: %s\n", WiFi.macAddress().c_str());
    Serial.println("Dich ESP-NOW: A8:42:E3:CA:F9:88; kenh 1.");
    stm32Serial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    Serial.println("Dang cho STM32: RX=GPIO16, TX=GPIO17, 115200 baud...");
}

void loop()
{
    while (stm32Serial.available() > 0)
    {
        char ch = (char)stm32Serial.read();
        if (ch == '\n')
        {
            if (!discardLine && lineLength > 0)
            {
                line[lineLength] = '\0';
                printFrame(line);
            }
            lineLength = 0;
            discardLine = false;
        }
        else if (ch != '\r' && !discardLine)
        {
            if (lineLength < sizeof(line) - 1)
                line[lineLength++] = ch;
            else
            {
                discardLine = true; // Resynchronize at the next newline.
                Serial.println("Bo qua khung qua dai.");
            }
        }
    }

    uint32_t now = millis();
    if (now - lastReceivedMs >= 2000U && now - lastWarningMs >= 2000U)
    {
        Serial.println("Chua nhan du lieu hop le. Kiem tra day TX/RX va GND.");
        lastWarningMs = now;
    }
    delay(1);
}
