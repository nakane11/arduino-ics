#include <Arduino.h>
#include <IcsHardSerialClass.h>
#include <RunningStats.h>
#include <M5AtomS3.h>
#include <button_manager.h>

#include <vector>

std::vector<RunningStats> stats_vector;

#define DEBUG_SERIAL M5.Lcd

#ifndef EN_PIN
    #define EN_PIN -1
#endif

#ifndef RX_PIN
    #define RX_PIN 18
#endif

#ifndef TX_PIN
    #define TX_PIN 17
#endif

const int TIMEOUT = 10;
IcsHardSerialClass *krs;
std::vector<int> ids;
ButtonManager button_manager;
bool servo_on = false;
bool button_updated = false;

void setup() {
    DEBUG_SERIAL.begin();
    DEBUG_SERIAL.println("start");
    // Wait for board to stabilize
    delay(2000);

    const long BAUDRATE = 1250000;
    Serial1.begin(BAUDRATE, SERIAL_8E1, RX_PIN, TX_PIN, false, TIMEOUT);
    krs = new IcsHardSerialClass(&Serial1, BAUDRATE, TIMEOUT, EN_PIN);
    krs->begin();
    while (ids.size() < 19) {
        ids.clear();
        stats_vector.clear();

        uint32_t idBits = krs->scanIDs();
        for (int servo_id = 0; servo_id < 32; ++servo_id) {
            if (static_cast<int>(idBits) & (1 << servo_id)) {
                ids.push_back(servo_id);
                RunningStats stats;
                stats_vector.push_back(stats);
            }
        }
        for (size_t i = 0; i < ids.size(); ++i) {
          DEBUG_SERIAL.printf("%d,", ids[i]);
          krs->setSpeed(ids[i], 10);
        }
        DEBUG_SERIAL.print("\n");
    }
    button_manager.createTask(0);
}

void loop() {
  switch (button_manager.getButtonState()) {
  case 0:
    button_updated = false;
    break;
  case 1:
    if (!button_updated) {
      DEBUG_SERIAL.clear();
      DEBUG_SERIAL.setCursor(0,0);
      if (servo_on) {
        for (size_t i = 0; i < ids.size(); ++i) {
          krs->setServoFree(ids[i]);
        }
        DEBUG_SERIAL.println("servo free");
      } else {
        for (size_t i = 0; i < ids.size(); ++i) {
          krs->setServoHold(ids[i]);
        }
        DEBUG_SERIAL.println("servo on");
      }
      servo_on = !servo_on;
      button_updated = true;
    }
    break;
  }
  delay(200);
}
