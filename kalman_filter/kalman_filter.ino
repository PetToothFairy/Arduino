#include <Arduino.h>
#include "NanoBLEFlashPrefs.h"  // 플래시 메모리 라이브러리
#include <Arduino_LSM9DS1.h>    // LSM9DS1 센서 라이브러리

// 칼만 필터용 구조체 정의
struct Kalman {
  float angle;    // 추정된 각도
  float bias;     // 자이로 바이어스
  float rate;     // 각속도
  float P[2][2];  // 오차 공분산 행렬
};

// 자기 센서 보정값과 Kalman 필터 상태를 저장할 구조체 정의
struct CalibrationData {
  float magX_min, magX_max;        // 자기 센서 X축 최소/최대값
  float magY_min, magY_max;        // 자기 센서 Y축 최소/최대값
  float magZ_min, magZ_max;        // 자기 센서 Z축 최소/최대값
  Kalman kalmanRoll, kalmanPitch;  // Roll과 Pitch용 Kalman 필터 상태
};

// 초기 보정값 설정
CalibrationData calibrationData = {
  9999, -9999,  // magX_min, magX_max 초기값
  9999, -9999,  // magY_min, magY_max 초기값
  9999, -9999   // magZ_min, magZ_max 초기값
};

// 플래시 메모리 사용을 위한 인스턴스 생성
NanoBLEFlashPrefs flashPrefs;

// Kalman 필터 초기화 함수
void initKalman(Kalman &kf) {
  kf.angle = 0.0f;    // 초기 각도 값
  kf.bias = 0.0f;     // 초기 바이어스 값
  kf.P[0][0] = 1.0f;  // 공분산 행렬 초기화
  kf.P[0][1] = 0.0f;
  kf.P[1][0] = 0.0f;
  kf.P[1][1] = 1.0f;
}

// Kalman 필터 업데이트 함수
float updateKalman(Kalman &kf, float newAngle, float newRate, float dt) {
  float Q_angle = 0.01;    // 프로세스 노이즈
  float Q_bias = 0.003;    // 바이어스 노이즈
  float R_measure = 0.01;  // 측정 노이즈

  // 예측 단계
  kf.rate = newRate - kf.bias;  // 바이어스 제거
  kf.angle += dt * kf.rate;     // 새로운 각도 예측

  kf.P[0][0] += dt * (dt * kf.P[1][1] - kf.P[0][1] - kf.P[1][0] + Q_angle);
  kf.P[0][1] -= dt * kf.P[1][1];
  kf.P[1][0] -= dt * kf.P[1][1];
  kf.P[1][1] += Q_bias * dt;

  // 보정 단계
  float S = kf.P[0][0] + R_measure;
  float K[2];
  K[0] = kf.P[0][0] / S;  // 칼만 이득 계산
  K[1] = kf.P[1][0] / S;

  float y = newAngle - kf.angle;  // 관측값과 예측값 차이
  kf.angle += K[0] * y;           // 보정된 각도 계산
  kf.bias += K[1] * y;            // 보정된 바이어스 계산

  float P00_temp = kf.P[0][0];
  float P01_temp = kf.P[0][1];

  kf.P[0][0] -= K[0] * P00_temp;
  kf.P[0][1] -= K[0] * P01_temp;
  kf.P[1][0] -= K[1] * P00_temp;
  kf.P[1][1] -= K[1] * P01_temp;

  return kf.angle;  // 최종 각도 반환
}

// 센서 초기화 및 저장된 보정값 로드
void setup() {
  Serial.begin(9600);  // 시리얼 통신 시작
  while (!Serial)
    ;

  // IMU 센서 초기화
  if (!IMU.begin()) {
    Serial.println("IMU 초기화 실패!");
    while (1)
      ;
  }

  // 플래시 메모리에서 저장된 보정값 불러오기
  if (flashPrefs.readPrefs(&calibrationData, sizeof(calibrationData)) == FDS_SUCCESS) {
    Serial.println("플래시에서 보정 데이터 로드 완료.");
    // 저장된 데이터를 출력하여 확인

    // Kalman 필터 상태 출력
    Serial.println("Loaded Kalman Filter State:");
    Serial.print("Roll Bias: ");
    Serial.println(calibrationData.kalmanRoll.bias);
    Serial.print("Roll P Matrix: ");
    Serial.print(calibrationData.kalmanRoll.P[0][0]);
    Serial.print(", ");
    Serial.println(calibrationData.kalmanRoll.P[1][1]);
    Serial.print("Pitch Bias: ");
    Serial.println(calibrationData.kalmanPitch.bias);
    Serial.print("Pitch P Matrix: ");
    Serial.print(calibrationData.kalmanPitch.P[0][0]);
    Serial.print(", ");
    Serial.println(calibrationData.kalmanPitch.P[1][1]);
  } else {
    Serial.println("보정 데이터가 없습니다. 기본값을 사용합니다.");
    initKalman(calibrationData.kalmanRoll);   // Roll 필터 초기화
    initKalman(calibrationData.kalmanPitch);  // Pitch 필터 초기화
  }

  Serial.println("IMU 초기화 완료.");
  //Serial.println("센서를 모든 방향으로 움직여 자기장을 보정하세요...");
}

void loop() {
  static unsigned long prevTime = 0;             // 이전 루프 시간
  static unsigned long lastSaveTime = 0;         // 마지막 저장 시간
  unsigned long currentTime = millis();          // 현재 시간
  float dt = (currentTime - prevTime) / 1000.0;  // 경과 시간 (초 단위)
  prevTime = currentTime;

  // 센서 데이터 읽기
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);

    // 보정값 업데이트
    if (mx < calibrationData.magX_min) calibrationData.magX_min = mx;
    if (mx > calibrationData.magX_max) calibrationData.magX_max = mx;
    if (my < calibrationData.magY_min) calibrationData.magY_min = my;
    if (my > calibrationData.magY_max) calibrationData.magY_max = my;
    if (mz < calibrationData.magZ_min) calibrationData.magZ_min = mz;
    if (mz > calibrationData.magZ_max) calibrationData.magZ_max = mz;

    // Roll과 Pitch 계산
    float accelRoll = atan2(ay, az) * 180 / PI;
    float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;
    float roll = updateKalman(calibrationData.kalmanRoll, accelRoll, gx, dt);
    float pitch = updateKalman(calibrationData.kalmanPitch, accelPitch, gy, dt);

    // 자기 센서 데이터 정규화 및 Heading 계산
    float normMx = (mx - (calibrationData.magX_min + calibrationData.magX_max) / 2) / (calibrationData.magX_max - calibrationData.magX_min);
    float normMy = (my - (calibrationData.magY_min + calibrationData.magY_max) / 2) / (calibrationData.magY_max - calibrationData.magY_min);
    float heading = atan2(-normMy, normMx) * 180 / PI;
    if (heading < 0) heading += 360;

    // 30초마다 보정 데이터 저장
    if (currentTime - lastSaveTime > 30000) {
      if (flashPrefs.writePrefs(&calibrationData, sizeof(calibrationData)) == FDS_SUCCESS) {
        Serial.println("보정 데이터 플래시에 저장 완료.");
        // Kalman 필터 상태 출력
        Serial.println("Saved Kalman Filter State:");
        Serial.print("Roll Bias: ");
        Serial.println(calibrationData.kalmanRoll.bias);
        Serial.print("Roll P Matrix: ");
        Serial.print(calibrationData.kalmanRoll.P[0][0]);
        Serial.print(", ");
        Serial.println(calibrationData.kalmanRoll.P[1][1]);
        Serial.print("Pitch Bias: ");
        Serial.println(calibrationData.kalmanPitch.bias);
        Serial.print("Pitch P Matrix: ");
        Serial.print(calibrationData.kalmanPitch.P[0][0]);
        Serial.print(", ");
        Serial.println(calibrationData.kalmanPitch.P[1][1]);
      } else {
        Serial.println("보정 데이터 저장 실패!");
      }
      lastSaveTime = currentTime;
    }
  }

  delay(10);  // 10ms 대기 (샘플링 주기)
}
