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
bool walking = false;

void servo_task(void *parameter) {
  while(true) {
    delay(100);
  }
}

void display() {
  DEBUG_SERIAL.clear();
  DEBUG_SERIAL.setCursor(0,0);
  for (size_t i = 0; i < ids.size(); ++i) {
    DEBUG_SERIAL.printf("%d,", ids[i]);
  }

  DEBUG_SERIAL.setCursor(0,30);
  if (servo_on) {
    DEBUG_SERIAL.println("servo on");
  } else {
    DEBUG_SERIAL.println("servo free");
  }

  DEBUG_SERIAL.setCursor(0,40);
  if(walking){
    DEBUG_SERIAL.println("walking start");
  }else{
    DEBUG_SERIAL.println("walking stop");
  }
}

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
          krs->setSpeed(ids[i], 10);
        }
    }
    button_manager.createTask(0);
    xTaskCreatePinnedToCore(servo_task, "Servo Task", 2048, NULL, 24, NULL, 1);
}

void loop() {
  switch (button_manager.getButtonState()) {
  case 0:
    button_updated = false;
    break;
  case 1:
    if (!button_updated) {
      if (servo_on) {
        walking = false;
        for (size_t i = 0; i < ids.size(); ++i) {
          krs->setServoFree(ids[i]);
        }
      } else {
        for (size_t i = 0; i < ids.size(); ++i) {
          krs->setServoHold(ids[i]);
        }
      }
      servo_on = !servo_on;
      button_updated = true;
    }
    break;
  case 2:
    if (!button_updated) {
      walking = !walking;
      button_updated = true;
    }
    break;
  }
  display();
  delay(200);
}
