#include <ArduinoBLE.h>       // BLE 라이브러리
#include <Arduino_LSM9DS1.h>  // IMU 센서 라이브러리

// BLE 서비스 및 특성 정의
BLEService postureService("180A");  // 사용자 정의 BLE 서비스 UUID
BLECharacteristic postureCharacteristic("2A19", BLERead | BLENotify, 20);  // Posture 특성 (최대 20바이트)

// 자기 센서 보정을 위한 변수
float magX_min = 9999, magX_max = -9999; // 초기값 설정
float magY_min = 9999, magY_max = -9999;
float magZ_min = 9999, magZ_max = -9999;

// 칼만 필터용 구조체
struct Kalman {
  float angle; // 추정된 각도
  float bias;  // 자이로 바이어스
  float rate;  // 각속도
  float P[2][2]; // 오차 공분산 행렬
};

// Roll 및 Pitch용 칼만 필터
Kalman kalmanRoll, kalmanPitch;

// 칼만 필터 초기화
void initKalman(Kalman &kf) {
  kf.angle = 0.0f;
  kf.bias = 0.0f;
  kf.P[0][0] = 1.0f;
  kf.P[0][1] = 0.0f;
  kf.P[1][0] = 0.0f;
  kf.P[1][1] = 1.0f;
}

// 칼만 필터 업데이트
float updateKalman(Kalman &kf, float newAngle, float newRate, float dt) {
  float Q_angle = 0.01;  // 프로세스 노이즈
  float Q_bias = 0.003;  // 바이어스 노이즈
  float R_measure = 0.01; // 측정 노이즈

  // 예측 단계
  kf.rate = newRate - kf.bias;
  kf.angle += dt * kf.rate;

  kf.P[0][0] += dt * (dt * kf.P[1][1] - kf.P[0][1] - kf.P[1][0] + Q_angle);
  kf.P[0][1] -= dt * kf.P[1][1];
  kf.P[1][0] -= dt * kf.P[1][1];
  kf.P[1][1] += Q_bias * dt;

  // 보정 단계
  float S = kf.P[0][0] + R_measure;
  float K[2];
  K[0] = kf.P[0][0] / S;
  K[1] = kf.P[1][0] / S;

  float y = newAngle - kf.angle;
  kf.angle += K[0] * y;
  kf.bias += K[1] * y;

  float P00_temp = kf.P[0][0];
  float P01_temp = kf.P[0][1];

  kf.P[0][0] -= K[0] * P00_temp;
  kf.P[0][1] -= K[0] * P01_temp;
  kf.P[1][0] -= K[1] * P00_temp;
  kf.P[1][1] -= K[1] * P01_temp;

  return kf.angle;
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // IMU 초기화
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // BLE 초기화
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }

  // BLE 장치 이름 및 서비스 설정
  BLE.setLocalName("IMU_Posture_Device");
  BLE.setAdvertisedService(postureService);
  postureService.addCharacteristic(postureCharacteristic);
  BLE.addService(postureService);

  // BLE 광고 시작
  BLE.advertise();
  Serial.println("BLE advertising started...");

  initKalman(kalmanRoll);
  initKalman(kalmanPitch);
}

void loop() {
  // BLE 연결 확인
  BLEDevice central = BLE.central();

  if (central) {
    Serial.println("BLE device connected!");

    while (central.connected()) {
      static unsigned long prevTime = 0;
      unsigned long currentTime = millis();
      float dt = (currentTime - prevTime) / 1000.0;  // 초 단위 시간 계산
      prevTime = currentTime;

      float ax, ay, az, gx, gy, gz, mx, my, mz;

      // IMU 데이터 읽기
      if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);
        IMU.readMagneticField(mx, my, mz);

        // Roll 및 Pitch 계산
        float accelRoll = atan2(ay, az) * 180 / PI;
        float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

        float gyroRollRate = gx;  // 각속도 (°/s)
        float gyroPitchRate = gy;

        // 칼만 필터로 Roll, Pitch 계산
        float roll = updateKalman(kalmanRoll, accelRoll, gyroRollRate, dt);
        float pitch = updateKalman(kalmanPitch, accelPitch, gyroPitchRate, dt);

        // 자기 센서 보정
        float normMx = (mx - (magX_min + magX_max) / 2) / (magX_max - magX_min);
        float normMy = (my - (magY_min + magY_max) / 2) / (magY_max - magY_min);

        // Heading 계산
        float heading = atan2(-normMy, normMx) * 180 / PI;
        if (heading < 0) heading += 360;

        // 자세 상태 분류
        String posture;
        if (roll >= -45 && roll < 45) {
          posture = "BRUSH_DOWN";
        } else if (roll >= 45 && roll < 135) {
          posture = "BRUSH_LEFT";
        } else if (roll >= -135 && roll < -45) {
          posture = "BRUSH_RIGHT";
        } else {
          posture = "BRUSH_UP";
        }

        // Serial 및 BLE로 posture 데이터 전송
        Serial.println("Posture: " + posture);
        postureCharacteristic.writeValue(posture.c_str());
      }

      delay(100);  // 데이터 전송 주기
    }

    Serial.println("BLE device disconnected.");
  }
}
