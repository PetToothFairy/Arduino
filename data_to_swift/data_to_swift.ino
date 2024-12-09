#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h> // BLE 라이브러리

float base_roll = 0, base_pitch = 0, base_yaw_x = 0, base_yaw_y = 0, base_yaw = 0; // 초깃값
float count = 0;
unsigned long startTime;
bool isCalibrated = false;
bool isStartInitialized = false;

// BLE 서비스와 특성 정의
BLEService toothbrushService("180F");
BLEStringCharacteristic dataCharacteristic("2A19", BLERead | BLENotify, 128); // 데이터 전송용

void setup() {
  Serial.begin(9600);
  //while (!Serial);

  // IMU 초기화
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("Sensors initialized.");

  // BLE 초기화
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }

  // BLE 장치 이름 설정 및 서비스 추가
  BLE.setLocalName("ToothbrushSensor");
  BLE.setAdvertisedService(toothbrushService);
  toothbrushService.addCharacteristic(dataCharacteristic);
  BLE.addService(toothbrushService);

  // BLE 광고 시작
  BLE.advertise();
  Serial.println("BLE advertising started...");

}

void loop() {
  // BLE 연결 상태 확인
  BLEDevice central = BLE.central();
  if (central) {
    float ax, ay, az, gx, gy, gz, mx, my, mz;

    if (!isStartInitialized) {
      startTime = millis(); // 프로그램 시작 시간 초기화
      isStartInitialized = true;
    }

    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
      // IMU 데이터 읽기
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);
      IMU.readMagneticField(mx, my, mz);

      float roll = atan2(ay, az) * 180 / PI;
      float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;
      float heading = atan2(-my, mx) * 180 / PI;
      if (heading < 0) heading += 360;

      // 3초 동안 캘리브레이션
      if (!isCalibrated) {
        // 3초 동안 평균 계산
        if (millis() - startTime <= 3000) {
          count += 1;
          base_roll += roll;
          base_pitch += pitch;
          base_yaw_x += cos(heading * PI / 180); // yaw를 단위 벡터로 변환 후 합산
          base_yaw_y += sin(heading * PI / 180);

          Serial.print("Roll: "); Serial.print(roll);
          Serial.print(" / Pitch: "); Serial.print(pitch);
          Serial.print(" / Yaw: "); Serial.println(heading);
        }

        if (millis() - startTime > 3000) {
          base_roll /= count;
          base_pitch /= count;

          // 단위 벡터 평균으로 기준 yaw 계산
          float avg_x = base_yaw_x / count;
          float avg_y = base_yaw_y / count;
          base_yaw = atan2(avg_y, avg_x) * 180 / PI;
          if (base_yaw < 0) base_yaw += 360;

          isCalibrated = true;
          Serial.println("Calibration Complete!");
          Serial.print("Base Roll: "); Serial.print(base_roll);
          Serial.print(", Base Pitch: "); Serial.print(base_pitch);
          Serial.print(", Base Yaw: "); Serial.println(base_yaw);
        }
      }

      if (isCalibrated) {
        // 360도 순환성을 고려한 차이 계산
        float yaw_diff = fmod((heading - base_yaw + 360), 360);

        // 180도를 기준으로 음수 방향 변환
        if (yaw_diff > 180) {
            yaw_diff -= 360;
        }

        String posture = "", result = "";
        if (roll >= -45 && roll < 45) {
          posture = "BRUSH_RIGHT";
          if (yaw_diff >= -15 && yaw_diff < 15) { // 140 ~ 170 앞니
            if (roll >= 0) {
              result = "UNDER_";
            } else {
              result = "UP_";
            }
            result += "FRONT";
          }
          if ((yaw_diff >= 15 && yaw_diff < 40) || (yaw_diff >= -40 && yaw_diff < -15)) { // 0 ~ 40  오른쪽 송곳니
            if (roll >= 0) {
              result = "UNDER_";
            } else {
              result = "UP_";
            }
            result += "RIGHT_CANINE"; // 송곳니
          }
          if ((yaw_diff >= 40 && yaw_diff < 120) || (yaw_diff >= -100 && yaw_diff < -40)) { // 40 ~ 80 오른쪽 어금니
            if (roll >= 0) {
              result = "UNDER_";
            } else {
              result = "UP_";
            }
            result += "RIGHT_MOLAR_OUTSIDE"; // 어금니 바깥쪽
          }
        }

        else if (roll >= 45 && roll < 135) {
          posture = "BRUSH_UP";
          if (yaw_diff > 40 && yaw_diff < 50) {
            result = "UP_LEFT_MOLAR_CHEWING_SIDE"; // 어금니 씹는 면
          }
          if (yaw_diff >= 20 && yaw_diff < 35) {
            result = "UP_RIGHT_MOLAR_CHEWING_SIDE";
          }
        }

        else if (roll >= -135 && roll < -45) {
          posture = "BRUSH_DOWN";
          if ((yaw_diff >= -180 && yaw_diff < -170) || (yaw_diff >= -100 && yaw_diff < -180)) {
            result = "DOWN_RIGHT_MOLAR_CHEWING_SIDE";
          }
          if ((yaw_diff >= 250 && yaw_diff < 260) || (yaw_diff >= -110 && yaw_diff < -90)){
            result = "DOWN_LEFT_MOLAR_CHEWING_SIDE";
          }
        }

        else {
          posture = "BRUSH_LEFT";
          if ((yaw_diff >= -10 && yaw_diff < 10) || (yaw_diff >= 90 && yaw_diff < 130)){ 
            if (roll >= 90) {
              result = "UNDER_";
            } else {
              result = "LEFT_";
            }
            result += "FRONT";
          }
          if ((yaw_diff >= 30 && yaw_diff < 60) || (yaw_diff >= -150 && yaw_diff < -120)) { 
            if (roll >= 90) {
              result = "UP_";
            } else {
              result = "UNDER_";
            }
            result += "LEFT_MOLAR_OUTSIDE";
          }
          if ((yaw_diff >= -180 && yaw_diff < -150) || (yaw_diff >= 0 && yaw_diff < 30)) { 
            if (roll >= 90) {
              result = "UNDER_";
            } else {
              result = "UP_";
            }
            result += "LEFT_CANINE";
          }
        }

        // 결과 출력
        Serial.print("Result: ");
        Serial.print(result);
        Serial.print("  / 기준값 yaw: ");
        Serial.print(base_yaw);
        Serial.print("°   /   Roll: ");
        Serial.print(roll);
        Serial.print("°, Pitch: ");
        Serial.print(pitch);
        Serial.print("°, Heading: ");
        Serial.print(heading);
        Serial.print("° -> Posture: ");
        Serial.print(posture);
        Serial.print(", 기준값과 현재 yaw값 차이");
        Serial.println(yaw_diff);

              
        // BLE를 통해 결과 전송
        dataCharacteristic.writeValue(result);
      }
    }
      delay(10); // 10ms 주기로 데이터 수집  
  }
}
