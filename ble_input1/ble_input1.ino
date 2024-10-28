#include <Arduino_LSM9DS1.h>  // Arduino LSM9DS1 라이브러리 (IMU 센서)

// 설정 함수
void setup() {
  Serial.begin(9600);  // 시리얼 통신 시작
  while (!Serial);

  // IMU 센서 초기화
  if (!IMU.begin()) {
    Serial.println("IMU 시작 실패");
    while (1);  // IMU 초기화 실패 시 멈춤
  }

  Serial.println("IMU 초기화 완료");
}

// 루프 함수
void loop() {
  float accX, accY, accZ;
  float gyroX, gyroY, gyroZ;
  float magX, magY, magZ;

  // 가속도계 값 읽기
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accX, accY, accZ);
    Serial.print("가속도 (X, Y, Z): ");
    Serial.print(accX, 2); Serial.print(", ");
    Serial.print(accY, 2); Serial.print(", ");
    Serial.println(accZ, 2);
  }

  // 자이로 값 읽기
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
    Serial.print("자이로 (X, Y, Z): ");
    Serial.print(gyroX, 2); Serial.print(", ");
    Serial.print(gyroY, 2); Serial.print(", ");
    Serial.println(gyroZ, 2);
  }

  // 자기장 값 읽기
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(magX, magY, magZ);
    Serial.print("자기장 (X, Y, Z): ");
    Serial.print(magX, 2); Serial.print(", ");
    Serial.print(magY, 2); Serial.print(", ");
    Serial.println(magZ, 2);
  }

  Serial.println("--------------------");  // 구분선
  delay(500);  // 0.5초 대기
}
