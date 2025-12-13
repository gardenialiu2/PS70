// ========== TRANSMITTER (REMOTE) — fixed for ESP32 core 3.x ==========
#include <WiFi.h>
#include <esp_now.h>

// ---- Button pins on YOUR remote board ----
// Adjust to match your hardware. (18/19 are fine on many ESP32 DevKits.)
const int BTN_A = 18;  // NORMAL toggle
const int BTN_B = 19;  // WAVE toggle

// ---- Receiver (servo XIAO) MAC ----
// Your XIAO servo receiver MAC: 58:8C:81:A8:A1:38
uint8_t receiverAddress[] = {0x58, 0x8C, 0x81, 0xA8, 0xA1, 0x38};

typedef struct __attribute__((packed)) {
  uint8_t seq;   // changes every press so receiver sees repeats
  uint8_t cmd;   // 1 = NORMAL toggle, 2 = WAVE toggle
} Packet;

bool lastA = HIGH, lastB = HIGH;
uint8_t seq = 0;

// --- ESP-NOW send callback: support both core 2.x and 3.x ---
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
// Arduino-ESP32 core 3.x / IDF5
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sent OK" : "Send FAIL");
}
#else
// Arduino-ESP32 core 2.x
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sent OK" : "Send FAIL");
}
#endif

void setup() {
  Serial.begin(115200);
  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while (true) {}
  }
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, receiverAddress, 6);
  peer.channel = 0;            // use current channel
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Add peer failed");
    while (true) {}
  }

  Serial.println("Button remote ready");
}

void sendCmd(uint8_t cmd) {
  Packet p{ seq++, cmd };
  // small reliability burst
  for (int i = 0; i < 3; i++) {
    esp_now_send(receiverAddress, (uint8_t*)&p, sizeof(p));
  }
}

void loop() {
  bool a = digitalRead(BTN_A);
  bool b = digitalRead(BTN_B);

  // Edge detect (pullups => HIGH -> LOW = press)
  if (lastA == HIGH && a == LOW) sendCmd(1); // NORMAL toggle
  if (lastB == HIGH && b == LOW) sendCmd(2); // WAVE toggle

  lastA = a; lastB = b;
  delay(20); // debounce
}
