#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <math.h>

// 사용자 정의 UUID
BLEService imuService("123e4567-e89b-12d3-a456-426614174000");  // IMU 서비스 UUID
BLECharacteristic imuCharacteristic("abcdef01-2345-6789-abcd-ef0123456789", BLERead | BLENotify, 150);  // IMU 데이터 특성 UUID (150바이트로 확장)

// Roll, Pitch, Yaw를 저장할 변수
float roll = 0, pitch = 0, yaw = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // IMU 초기화
  if (!IMU.begin()) {
    Serial.println("IMU 시작 실패");
    while (1);
  }

  // BLE 초기화
  if (!BLE.begin()) {
    Serial.println("BLE 시작 실패");
    while (1);
  }

  // BLE 장치 이름 및 서비스 설정
  BLE.setLocalName("Nano33BLE_IMU");  // 장치 이름 설정
  BLE.setAdvertisedService(imuService);  // 서비스 광고 설정
  imuService.addCharacteristic(imuCharacteristic);  // 특성 추가
  BLE.addService(imuService);  // 서비스 추가

  // BLE 광고 시작
  BLE.advertise();
  Serial.println("BLE 광고 시작");
}

void loop() {
  // BLE 장치와 연결 대기
  BLEDevice central = BLE.central();

  if (central) {
    Serial.println("BLE 장치 연결됨");

    // 연결된 상태에서 IMU 데이터를 전송
    while (central.connected()) {
      float accX, accY, accZ;
      float gyroX, gyroY, gyroZ;

      // 가속도계 및 자이로 값 읽기
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accX, accY, accZ);
      }
      if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gyroX, gyroY, gyroZ);
      }

      // Roll 및 Pitch 계산 (각도를 라디안에서 도로 변환)
      pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / M_PI;
      roll = atan2(accY, accZ) * 180.0 / M_PI;
      
      // Yaw는 자이로스코프 데이터를 이용하여 누적합으로 계산 (단순화된 방식)
      yaw += gyroZ * 0.01;  // 루프 지연 시간에 따른 단위 변경 (0.01초 가정)

      // JSON 형식으로 IMU 데이터 생성
      String imuData = "{\"orientation\":{\"roll\":" + String(roll, 2) + ", \"pitch\":" + String(pitch, 2) + ", \"yaw\":" + String(yaw, 2) + "}}";

      // 데이터 전송 전에 크기 확인
      if (imuData.length() <= 150) {  // 150바이트 이하 데이터 전송 가능
        imuCharacteristic.writeValue(imuData.c_str());  // 문자열로 데이터 전송
        Serial.println("전송된 데이터: " + imuData);
      } else {
        Serial.println("데이터 길이 초과: " + String(imuData.length()));
      }

      delay(10);  // 10ms마다 데이터 갱신
    }

    Serial.println("BLE 장치 연결 끊김");
  }
}
