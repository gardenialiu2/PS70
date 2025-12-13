// ========== RECEIVER (XIAO ESP32-C3 + SERVOS, NO LOCAL BUTTONS) ==========
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

// ---- Motion settings ----
const int openPos = 125;
const int closedPos = 180;
const unsigned long moveInterval = 10;
const unsigned long pauseDuration = 800;

// ---- Wave settings ----
const unsigned long waveGapMs = 500;

const int deltaDeg = (closedPos - openPos);
const unsigned long sweepTime = (unsigned long)deltaDeg * moveInterval;
const unsigned long period    = 2UL * sweepTime + 2UL * pauseDuration;

typedef struct __attribute__((packed)) {
  uint8_t seq;   // distinguishes repeated presses
  uint8_t cmd;   // 1=NORMAL toggle, 2=WAVE toggle
} Packet;

enum Mode { STOPPED, NORMAL, WAVE };
volatile uint8_t rxCmd = 0;
volatile uint8_t rxSeq = 255;

Mode mode = STOPPED;
unsigned long waveStartTime = 0;

class PetalServo {
public:
  Servo servo;
  int pos;
  int step;
  unsigned long lastMoveTime;
  unsigned long lastPauseTime;
  bool paused;

  PetalServo(int pin)
    : pos(closedPos), step(-1), lastMoveTime(0), lastPauseTime(0), paused(false)
  {
    servo.attach(pin, 500, 2400);  // attach to pin with pulse min/max
    servo.setPeriodHertz(50);      // standard hobby servo period
    servo.write(pos);
  }

  void updateNormal(bool enable) {
    if (!enable) return;
    unsigned long now = millis();

    if (paused) {
      if (now - lastPauseTime >= pauseDuration) paused = false;
      return;
    }

    if (now - lastMoveTime >= moveInterval) {
      pos += step;
      if (pos < openPos)  pos = openPos;
      if (pos > closedPos) pos = closedPos;

      servo.write(pos);
      lastMoveTime = now;

      if (pos <= openPos || pos >= closedPos) {
        step = -step;
        paused = true;
        lastPauseTime = now;
      }
    }
  }
};

// === Servos on XIAO header pins D0..D4 ===
PetalServo petal1(D0), petal2(D1), petal3(D2), petal4(D3), petal5(D4);

int waveAngleAt(unsigned long tMs) {
  if (tMs < pauseDuration) return closedPos;  tMs -= pauseDuration;
  if (tMs < sweepTime)     return closedPos - (int)((deltaDeg * (unsigned long long)tMs) / sweepTime);
  tMs -= sweepTime;
  if (tMs < pauseDuration) return openPos;    tMs -= pauseDuration;
  return openPos + (int)((deltaDeg * (unsigned long long)tMs) / sweepTime);
}

void driveWave(PetalServo &p, int index, unsigned long now) {
  const unsigned long offset = (unsigned long)index * waveGapMs;
  const unsigned long t = (now - waveStartTime + offset) % period;
  p.servo.write(waveAngleAt(t));
}

// ESP-NOW receive callback (Arduino-ESP32 3.x)
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < (int)sizeof(Packet)) return;
  Packet p; memcpy(&p, data, sizeof(Packet));
  rxSeq = p.seq;   // latch latest press
  rxCmd = p.cmd;
}

void setup() {
  Serial.begin(115200);

  // Allocate PWM timers (ESP32Servo requirement)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) {}
  }
  esp_now_register_recv_cb(onRecv);

  Serial.println("Servo receiver ready (XIAO ESP32-C3, servos on D0–D4)");
  Serial.print("Receiver MAC: "); Serial.println(WiFi.macAddress());
}

void loop() {
  // Handle remote presses (every unique seq)
  static uint8_t lastHandledSeq = 255;
  if (rxSeq != lastHandledSeq) {
    lastHandledSeq = rxSeq;
    if (rxCmd == 1) {                     // NORMAL toggle
      mode = (mode == NORMAL) ? STOPPED : NORMAL;
      if (mode == WAVE) mode = NORMAL;    // switch to NORMAL if coming from WAVE
      Serial.println(mode == NORMAL ? "Remote: NORMAL" : "Remote: STOPPED");
    } else if (rxCmd == 2) {              // WAVE toggle
      if (mode == WAVE) { mode = STOPPED; Serial.println("Remote: STOPPED"); }
      else { mode = WAVE; waveStartTime = millis(); Serial.println("Remote: WAVE"); }
    }
  }

  // Drive motion
  unsigned long now = millis();
  if (mode == NORMAL) {
    petal1.updateNormal(true);
    petal2.updateNormal(true);
    petal3.updateNormal(true);
    petal4.updateNormal(true);
    petal5.updateNormal(true);
  } else if (mode == WAVE) {
    driveWave(petal1, 0, now);
    driveWave(petal2, 1, now);
    driveWave(petal3, 2, now);
    driveWave(petal4, 3, now);
    driveWave(petal5, 4, now);
  } // STOPPED: hold current angles

  delay(3);
}
