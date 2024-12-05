//#include <Arduino_LSM9DS1.h> // LSM9DS1 라이브러리
#include <Arduino_LSM6DSOX.h>

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
  float Q_angle = 0.01; // 프로세스 노이즈
  float Q_bias = 0.003; // 바이어스 노이즈
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

// 센서 초기화
void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  initKalman(kalmanRoll);
  initKalman(kalmanPitch);

  Serial.println("IMU initialized.");
  Serial.println("Move the sensor in all directions to calibrate magnetic field...");
}

void loop() {
  static unsigned long prevTime = 0;
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0; // 초 단위로 시간 계산
  prevTime = currentTime;

  float ax, ay, az, gx, gy, gz, mx, my, mz;

  // IMU 데이터 읽기
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);

    // 보정값 갱신 (최소/최대값)
    if (mx < magX_min) magX_min = mx;
    if (mx > magX_max) magX_max = mx;
    if (my < magY_min) magY_min = my;
    if (my > magY_max) magY_max = my;
    if (mz < magZ_min) magZ_min = mz;
    if (mz > magZ_max) magZ_max = mz;

    // 가속도계로 계산한 Roll, Pitch
    float accelRoll = atan2(ay, az) * 180 / PI;
    float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

    // 자이로스코프 각속도
    float gyroRollRate = gx; // 각속도 (°/s)
    float gyroPitchRate = gy;

    // 칼만 필터로 Roll, Pitch 계산
    float roll = updateKalman(kalmanRoll, accelRoll, gyroRollRate, dt);
    float pitch = updateKalman(kalmanPitch, accelPitch, gyroPitchRate, dt);

    // 자기 센서 보정
    float normMx = (mx - (magX_min + magX_max) / 2) / (magX_max - magX_min);
    float normMy = (my - (magY_min + magY_max) / 2) / (magY_max - magY_min);
    float normMz = (mz - (magZ_min + magZ_max) / 2) / (magZ_max - magZ_min);

    // Heading (방위각) 계산
    float heading = atan2(-normMy, normMx) * 180 / PI;
    if (heading < 0) heading += 360;

    // 칫솔의 상태 분류
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

    // 결과 출력
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print("°, Pitch: ");
    Serial.print(pitch);
    Serial.print("°, Heading: ");
    Serial.print(heading);
    Serial.print("° -> Posture: ");
    Serial.println(posture);
  }

  delay(10); // 10ms 주기로 데이터 수집
}
