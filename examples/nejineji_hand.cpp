#include <Arduino.h>
#include <IcsHardSerialClass.h>
#include <RunningStats.h>
#include <M5AtomS3.h>
#include <button_manager.h>
#include <sstream>
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
bool servo_on_state = false;
bool button_updated = false;
bool walking = false;

#define NUM_SERVO 19
#define NUM_SEQUENCE 9
int walk_angle_index = 0;
std::string walk_angle_string = R"(list
#f(102.33 28.08 40.77 32.8388 13.5 56.4638 -89.505 17.1788 -10.2937 23.49 -50.5912 -6.17624 -16.8412 58.2863 92.745 -14.6475 -5.46749 20.25 -48.06)
#f(102.026 27.7763 40.77 28.9575 -6.37874 58.1175 55.4513 16.2 -6.68249 23.49 -49.6125 -10.6987 -20.7562 53.9663 66.0825 -14.0062 -4.11749 20.25 -47.0475)
#f(103.005 9.14626 6.04126 38.0363 -9.61874 83.8688 84.0375 13.9725 -13.2975 9.11251 -6.47999 -11.34 -17.5162 95.2088 93.3188 -13.6687 7.02001 -8.87624 5.80501)
#f(100.373 20.385 39.825 19.2713 -9.61874 78.2663 93.4538 7.22251 -19.6087 0.81001 56.9025 -4.89374 -23.6925 82.6875 95.2763 -14.6475 -21.3975 39.1838 12.2513)
#f(101.689 16.5375 40.4663 28.6538 0.13501 27.0 100.946 4.01626 -8.30249 12.96 84.2738 -7.45874 -26.9662 42.525 99.7988 -19.8787 -9.51749 57.1388 12.5888)
#f(101.351 16.8413 40.77 28.9575 -8.30249 22.68 -10.5637 5.94001 -14.9512 67.9388 95.2425 -6.47999 -23.6925 25.8188 -12.3862 -18.5625 -18.6975 76.3088 58.6575)
#f(102.026 11.07 -1.72124 28.6538 20.7563 12.42 -18.09 6.27751 -5.02874 99.225 75.9713 -9.38249 -8.70749 10.6988 -13.3312 -27.3712 -7.15499 97.065 60.9188)
#f(103.646 20.0475 27.135 27.3375 8.91001 27.9788 1.48501 8.50501 -18.9675 74.5538 86.94 -8.43749 -24.6712 40.9725 -2.36249 -28.35 -24.435 67.5338 71.6175)
#f(104.963 23.5913 29.7338 28.3163 8.57251 27.675 1.14751 7.56001 -20.2837 54.4388 95.9175 -7.45874 -25.0087 39.7575 -2.69999 -27.7087 -24.0975 39.1838 71.6175)
)";
// int rcb4_index[NUM_SERVO] = {5, 17, 16, 15, 1, 6, 10, 4, 3, 2, 18, 9, 8, 7, 12, 11, 0, 13, 14};
int rcb4_index[NUM_SERVO] = {16, 4, 9, 8, 7, 0, 5, 13, 12, 11, 6, 15, 14, 17, 18, 3, 2, 1, 10};

std::vector<std::vector<float>> parseEuslispString(const std::string& s) {
    std::vector<std::vector<float>> data;
    size_t search_pos = 0;
    // 文字列の先頭から "#f(" を探していく
    while ((search_pos = s.find("#f(", search_pos)) != std::string::npos) {
        // "#f(" の終わり（数値の始まり）
        size_t start_num_pos = search_pos + 3;
        // 対応する ")" を探す
        size_t end_pos = s.find(")", start_num_pos);
        if (end_pos == std::string::npos) {
            break; // 見つからなければループを抜ける
        }
        // 数値部分の文字列を抜き出す
        std::string numbers_str = s.substr(start_num_pos, end_pos - start_num_pos);
        // stringstreamを使ってスペース区切りの文字列をfloatに変換
        std::stringstream ss(numbers_str);
        std::vector<float> row;
        float number;
        while (ss >> number) {
            row.push_back(number);
        }
        data.push_back(row);
        // 次の探索開始位置を更新
        search_pos = end_pos + 1;
    }
    return data;
}
std::vector<std::vector<float>> walk_angle_float = parseEuslispString(walk_angle_string);

int angle_to_position(float angle) {
  return 7500 + 4000 * angle / 135;
}

void servo_task(void *parameter) {
  while(true) {
    if (!walking) {
      vTaskDelay(100);
      continue;
    }
    // 角度指令
    for (size_t i = 0; i < ids.size(); ++i) {
      // while (krs->setServoPosition(ids[i], angle_to_position(walk_angle_float[walk_angle_index][rcb4_index[ids[i]]])) == -1) {
      //   vTaskDelay(10);
      // }
      while (krs->setServoPosition(rcb4_index[i], angle_to_position(walk_angle_float[walk_angle_index][i])) == -1) {
        vTaskDelay(10);
      }
      vTaskDelay(5);
    }
    // vTaskDelay(800);
    // 目標位置に達するまで待機
    bool flag = 1;
    int timeout_counter = 80;
    while (flag && timeout_counter > 0) {
      flag = 0;
      for (size_t i = 0; i < ids.size(); ++i) {
        if(abs(krs->getPosition(ids[i]) - angle_to_position(walk_angle_float[walk_angle_index][rcb4_index[ids[i]]])) >= 100) {
          flag = 1;
          break;
        }
      }
      vTaskDelay(10 / portTICK_PERIOD_MS);
      timeout_counter--;
    }
    walk_angle_index++;
    if (walk_angle_index >= NUM_SEQUENCE) {
      walk_angle_index = 0;
    }
  }
}

void servo_on(bool on) {
  if (servo_on_state) {
    if (!on) {
      walking = false;
      for (size_t i = 0; i < ids.size(); ++i) {
        krs->setServoFree(ids[i]);
      }
    }
  } else {
    if (on) {
      for (size_t i = 0; i < ids.size(); ++i) {
        krs->setServoHold(ids[i]);
      }
    }
  }
  servo_on_state = on;
}

void display() {
  DEBUG_SERIAL.clear();
  DEBUG_SERIAL.setCursor(0,0);
  for (size_t i = 0; i < ids.size(); ++i) {
    DEBUG_SERIAL.printf("%d,", ids[i]);
  }

  DEBUG_SERIAL.setCursor(0,30);
  if (servo_on_state) {
    DEBUG_SERIAL.println("servo on");
  } else {
    DEBUG_SERIAL.println("servo free");
  }

  DEBUG_SERIAL.setCursor(0,40);
  if(walking){
    DEBUG_SERIAL.println("walking start");
    for (size_t i = 0; i < ids.size(); ++i) {
      DEBUG_SERIAL.printf("%d: %d\n", ids[i], abs(krs->getPosition(ids[i]) - angle_to_position(walk_angle_float[walk_angle_index][rcb4_index[ids[i]]])));
    }
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
    while (ids.size() < NUM_SERVO) {
        ids.clear();
        stats_vector.clear();

        uint32_t idBits = krs->scanIDs();
        for (int servo_id = 0; servo_id < NUM_SERVO; ++servo_id) {
            if (static_cast<int>(idBits) & (1 << servo_id)) {
                ids.push_back(servo_id);
                RunningStats stats;
                stats_vector.push_back(stats);
            }
        }
        for (size_t i = 0; i < ids.size(); ++i) {
          krs->setSpeed(ids[i], 20);
        }
    }
    button_manager.createTask(0);
    xTaskCreatePinnedToCore(servo_task, "Servo Task", 2048, NULL, 24, NULL, 0);
}

void loop() {
  switch (button_manager.getButtonState()) {
  case 0:
    button_updated = false;
    break;
  case 1:
    if (!button_updated) {
      if (servo_on_state) {
        servo_on(false);
      }else {
        servo_on(true);
      }
      button_updated = true;
    }
    break;
  case 2:
    if (!button_updated) {
      if (!walking) {
        servo_on(true);
        // 初期姿勢になる
        walk_angle_index = 0;
      }
      walking = !walking;
      button_updated = true;
    }
    break;
  }
  display();
  delay(200);
}
