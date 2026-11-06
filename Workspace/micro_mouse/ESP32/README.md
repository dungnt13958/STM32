# UART -> ESP-NOW -> Serial Monitor

## Nap chuong trinh

Arduino IDE, board **ESP32 Dev Module**, Arduino-ESP32 2.x hoac 3.x.
Moi thu muc chua mot sketch rieng; khong ghep hai file .ino vao cung sketch.

1. ESP32 gan STM32: nap `EncoderImuMonitor/EncoderImuMonitor.ino`.
2. ESP32 co MAC STA `A8:42:E3:CA:F9:88`: nap `EspNowReceiver/EspNowReceiver.ino`.
3. Mo Serial Monitor cua bo nhan, baud **115200**. Neu bo nhan in MAC khac,
   dung MAC STA thuc te do de sua `receiverMac` trong sketch bo gui roi nap lai.

## Day UART tren bo gui

| STM32 | ESP32 gui |
| --- | --- |
| PA9 TX | GPIO16 RX2 |
| PA10 RX | GPIO17 TX2 (tuy chon, hien chi nhan UART) |
| GND | GND |

UART: 115200, 8N1; du lieu STM32: `1250,1248,-12.34\r\n`.
Bo gui kiem tra dong CSV roi gui nguyen noi dung, khong kem CR/LF,
trong mot goi ESP-NOW. Bo nhan in encoder trai/phai (xung), yaw (do).
Khong can sua chuong trinh STM32.

## Kenh va trang thai

Hai bo dung Wi-Fi STA, kenh 1 (`ESPNOW_CHANNEL`), khong ket noi router.
Neu doi kenh, sua ca hai sketch. Khong can day noi giua hai ESP32.
ESP-NOW khong ma hoa trong vi du nay. Bo nhan chap nhan goi dung dinh dang
tu bat ky bo gui nao; MAC dich chi dinh bo nhan, khong xac thuc nguon gui.

`esp_now_send()` thanh cong chi cho biet goi duoc xep de gui, khong phai
xac nhan du lieu da den ung dung nhan. Khong co ACK ung dung hay gui lai.
Bo nhan dung hang doi 10 goi; bo goi moi neu hang doi day.
Can kiem tra tren phan cung de xac nhan truyen nhan thuc te.

Tai lieu API: https://docs.espressif.com/projects/esp-idf/en/v5.3.2/esp32/api-reference/network/esp_now.html
