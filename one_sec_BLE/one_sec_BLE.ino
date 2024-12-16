#include <Arduino_LSM9DS1.h>  // Arduino_BMI270_BMM150.h, Arduino_LSM9DS1.h
#include <ArduinoBLE.h>       // BLE 라이브러리

#define FSR_HISTORY_LENGTH 100

float base_roll = 0, base_pitch = 0, base_yaw_x = 0, base_yaw_y = 0, base_yaw = 0;  // 초깃값
float count = 0;
unsigned long startTime, sectionStartTime;
bool isCalibrated = false;
bool isStartInitialized = false;
const int fsrPin = A2;  // FSR-400 센서를 A0 핀에 연결
int fsrValue = 0;       // FSR 값을 저장할 변수
bool isValid;

// BLE 서비스와 특성 정의
BLEService toothbrushService("E8E8");
BLEStringCharacteristic dataCharacteristic("2A19", BLERead | BLENotify, 128);  // 데이터 전송용

// 14가지 부위와 카운트 저장 배열
const char* resultNames[] = {
  "UNDER_FRONT", "UP_FRONT", "UNDER_RIGHT_CANINE", "UP_RIGHT_CANINE",
  "UNDER_RIGHT_MOLAR_OUTSIDE", "UP_RIGHT_MOLAR_OUTSIDE", "UP_LEFT_MOLAR_CHEWING_SIDE",
  "UP_RIGHT_MOLAR_CHEWING_SIDE", "DOWN_LEFT_MOLAR_CHEWING_SIDE", "DOWN_RIGHT_MOLAR_CHEWING_SIDE",
  "UNDER_LEFT_CANINE", "UP_LEFT_CANINE", "UNDER_LEFT_MOLAR_OUTSIDE", "UP_LEFT_MOLAR_OUTSIDE"
};
int resultCounts[14] = { 0 };  // 각 부위의 카운트

void setup() {
  Serial.begin(9600);
  pinMode(fsrPin, INPUT);  // FSR 핀을 입력 모드로 설정

  // IMU 초기화
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }
  Serial.println("Sensors initialized.");

  // BLE 초기화
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1)
      ;
  }

  // BLE 장치 이름 설정 및 서비스 추가
  BLE.setLocalName("ToothbrushSensor");
  BLE.setAdvertisedService(toothbrushService);
  toothbrushService.addCharacteristic(dataCharacteristic);
  BLE.addService(toothbrushService);

  // BLE 광고 시작
  BLE.advertise();
  Serial.println("BLE advertising started...");

  sectionStartTime = millis();  // 5초 측정 시작 시간
  isValid = false;
}

void loop() {
  // BLE 연결 상태 확인
  BLEDevice central = BLE.central();
  if (central) {
    float ax, ay, az, gx, gy, gz, mx, my, mz;

    if (!isStartInitialized) {
      startTime = millis();  // 프로그램 시작 시간 초기화
      isStartInitialized = true;
    }

    // FSR 센서 값 읽기 및 기록
    fsrValue = analogRead(fsrPin);

    // IMU 데이터 읽기
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);

    double raw_roll = atan2(ay, az);                         // Roll = atan2(ay, az)
    double raw_pitch = atan2(-ax, sqrt(ay * ay + az * az));  // Pitch = atan2(-ax, sqrt(ay^2 + az^2))

    double roll = raw_roll * 180 / PI;
    double pitch = raw_pitch * 180 / PI;

    // 지자기 센서 값을 사용하여 Yaw 계산
    // Roll, Pitch가 적용된 자기장 X, Y 값
    double mx_rot = mx * cos(raw_pitch) + mz * sin(raw_pitch);
    double my_rot = mx * sin(raw_roll) * sin(raw_pitch) + my * cos(raw_roll) - mz * sin(raw_roll) * cos(raw_pitch);

    // Yaw 계산
    double heading = atan2(my_rot, mx_rot);  // Yaw = atan2(my_rot, mx_rot)
    heading = heading * 180.0 / PI;          // 라디안 -> 도 변환
    if (heading < 0) {
      heading += 360.0;  // 0~360도로 보정
    }

    // 3초 동안 캘리브레이션
    if (!isCalibrated) {
      // 3초 동안 평균 계산
      if (millis() - startTime <= 3000) {
        count += 1;
        base_roll += roll;
        base_pitch += pitch;
        base_yaw_x += cos(heading * PI / 180);  // yaw를 단위 벡터로 변환 후 합산
        base_yaw_y += sin(heading * PI / 180);
      }

      if (millis() - startTime > 3000) {
        base_roll /= count;
        base_pitch /= count;

        // 단위 벡터 평균으로 기준 yaw 계산
        double avg_x = base_yaw_x / count;
        double avg_y = base_yaw_y / count;
        base_yaw = atan2(avg_y, avg_x) * 180 / PI;
        if (base_yaw < 0) base_yaw += 360;

        isCalibrated = true;
        Serial.println("Calibration Complete!");
        Serial.print("Base Roll: ");
        Serial.print(base_roll);
        Serial.print(", Base Pitch: ");
        Serial.print(base_pitch);
        Serial.print(", Base Yaw: ");
        Serial.println(base_yaw);
      }
    }

    if (isCalibrated) {
      if(fsrValue >= 100){
        isValid = true;
      }
      // 360도 순환성을 고려한 차이 계산
      double yaw_diff = fmod((heading - base_yaw + 360), 360);

      // 180도를 기준으로 음수 방향 변환
      if (yaw_diff > 180) {
        yaw_diff -= 360;
      }

      String posture = "", result = "";
      if (roll >= -45 && roll < 45) {
        posture = "BRUSH_RIGHT";
        if (yaw_diff >= -25 && yaw_diff < 25) {  // 140 ~ 170 앞니
          if (roll >= 0) {
            result = "UNDER_";
          } else {
            result = "UP_";
          }
          result += "FRONT";
        }
        if (yaw_diff >= -80 && yaw_diff < -25) {  // 0 ~ 40  오른쪽 송곳니
          if (roll >= 0) {
            result = "UNDER_";
          } else {
            result = "UP_";
          }
          result += "RIGHT_CANINE";
        }
        if ((yaw_diff >= -140 && yaw_diff < -80)) {  // 40 ~ 80 오른쪽 어금니
          if (roll >= 0) {
            result = "UNDER_";
          } else {
            result = "UP_";
          }
          result += "RIGHT_MOLAR_OUTSIDE";
        }
      }

      else if (roll >= 45 && roll < 135) {
        posture = "BRUSH_UP";
        if (roll >= 45 && roll < 90) {
          result = "UP_LEFT_MOLAR_CHEWING_SIDE";
        }
        if (roll >= 90 && roll < 130) {
          result = "UP_RIGHT_MOLAR_CHEWING_SIDE";
        }
      }

      else if (roll >= -135 && roll < -45) {
        posture = "BRUSH_DOWN";
        if (roll >= -90 && roll < -45) {
          result = "DOWN_LEFT_MOLAR_CHEWING_SIDE";
        }
        if (roll >= -135 && roll < -90) {
          result = "DOWN_RIGHT_MOLAR_CHEWING_SIDE";
        }
      }

      else {
        posture = "BRUSH_LEFT";
        if (yaw_diff >= 150 || yaw_diff < -150) {
          if (roll >= 90) {
            result = "UNDER_";
          } else {
            result = "UP_";
          }
          result += "FRONT";
        }
        if (yaw_diff >= -115 && yaw_diff < -50) {
          if (roll >= 90) {
            result = "UP_";
          } else {
            result = "UNDER_";
          }
          result += "LEFT_MOLAR_OUTSIDE";
        }
        if (yaw_diff >= -150 && yaw_diff < -115) {
          if (roll >= 90) {
            result = "UNDER_";
          } else {
            result = "UP_";
          }
          result += "LEFT_CANINE";
        }

        for (int i = 0; i < 14; i++) {
          if (result == resultNames[i]) {
            resultCounts[i]++;
            break;
          }
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

      // 1초 후 최다 빈도 result 찾기
      if (millis() - sectionStartTime >= 1000) {
        int maxIndex = 0;
        for (int i = 1; i < 14; i++) {
          if (resultCounts[i] > resultCounts[maxIndex]) maxIndex = i;
        }

        // 최다 빈도 결과 BLE 전송
        String mostFrequentResult = resultNames[maxIndex];
        // if(!isValid){
        //   mostFrequentResult = "";
        // }
        dataCharacteristic.writeValue(mostFrequentResult);
        Serial.println("Most Frequent Result: " + mostFrequentResult);

        // 카운트 초기화
        for (int i = 0; i < 14; i++) resultCounts[i] = 0;
        sectionStartTime = millis();  // 다음 5초 시작 시간 초기화
      }
    }

    delay(10);  // 10ms 주기로 데이터 수집
  }
}
