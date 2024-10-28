#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

// 사용자 정의 UUID
BLEService imuService("123e4567-e89b-12d3-a456-426614174000");  // IMU 서비스 UUID
BLECharacteristic accelCharacteristic("abcdef01-2345-6789-abcd-ef0123456789", BLERead | BLENotify, 150);  // IMU 데이터 특성 UUID (150바이트로 확장)

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
  imuService.addCharacteristic(accelCharacteristic);  // 특성 추가
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
      float magX, magY, magZ;

      // 가속도계 값 읽기
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accX, accY, accZ);
      }

      // 자이로 값 읽기
      if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gyroX, gyroY, gyroZ);
      }

      // 자기장 값 읽기
      if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(magX, magY, magZ);
      }

      // JSON 형식으로 IMU 데이터 생성
      String imuData = "{\"acceleration\":{\"x\":" + String(accX, 2) + ", \"y\":" + String(accY, 2) + ", \"z\":" + String(accZ, 2) + "},"
                     + "\"magnetic\":{\"x\":" + String(magX, 2) + ", \"y\":" + String(magY, 2) + ", \"z\":" + String(magZ, 2) + "}}";

      // 데이터 전송 전에 크기 확인
      if (imuData.length() <= 150) {  // 150바이트 이하 데이터 전송 가능
        accelCharacteristic.writeValue(imuData.c_str());  // 문자열로 데이터 전송
        Serial.println("전송된 데이터: " + imuData);
      } else {
        Serial.println("데이터 길이 초과: " + String(imuData.length()));
      }

      delay(500);  // 0.5초마다 전송
    }

    Serial.println("BLE 장치 연결 끊김");
  }
}
