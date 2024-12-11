#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

// 사용자 정의 UUID
BLEService imuService("180A");  // 간략화된 IMU 서비스 UUID
BLECharacteristic orientationCharacteristic("2A19", BLERead | BLENotify, 200);  // 200바이트로 확장된 특성 UUID

// Roll, Pitch, Yaw 계산 함수
float calculateRoll(float accX, float accY, float accZ) {
  return atan2(accY, accZ) * 180.0 / PI;
}

float calculatePitch(float accX, float accY, float accZ) {
  return atan(-accX / sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;
}

float calculateYaw(float magX, float magY) {
  return atan2(magY, magX) * 180.0 / PI;
}

void setup() {
  Serial.begin(9600);
  
  // Serial 연결 대기를 제한 시간으로 대체 (최대 3초)
  unsigned long serialTimeout = millis();
  while (!Serial && millis() - serialTimeout < 3000) {
    // 디버깅 중에는 Serial 대기를 허용
  }

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
  imuService.addCharacteristic(orientationCharacteristic);  // 특성 추가
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
      float magX, magY, magZ;

      // 가속도계 값 읽기
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accX, accY, accZ);
      }

      // 자기장 값 읽기
      if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(magX, magY, magZ);
      }

      // Roll, Pitch, Yaw 계산
      float roll = calculateRoll(accX, accY, accZ);
      float pitch = calculatePitch(accX, accY, accZ);
      float yaw = calculateYaw(magX, magY);

      // JSON 형식으로 IMU 데이터 생성
      String imuData = "{\"orientation\":{\"roll\":" + String(roll, 2) + ", \"pitch\":" + String(pitch, 2) + ", \"yaw\":" + String(yaw, 2) + "}}";

      // 데이터 전송 전에 크기 확인
      if (imuData.length() <= 150) {  // 200바이트 이하 데이터 전송 가능
        orientationCharacteristic.writeValue(imuData.c_str());  // 문자열로 데이터 전송
        Serial.println("전송된 데이터: " + imuData);
      } else {
        Serial.println("데이터 길이 초과: " + String(imuData.length()));
      }

      delay(500);  // 0.5초마다 전송
    }

    Serial.println("BLE 장치 연결 끊김");
  }
}
