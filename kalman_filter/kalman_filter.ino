<<<<<<< HEAD
//#include <Arduino_LSM9DS1.h> // LSM9DS1 라이브러리
#include <Arduino_LSM6DSOX.h>
=======
#include <Arduino_BMI270_BMM150.h> // BMI270 + BMM150 라이브러리

float base_roll = 0, base_pitch = 0, base_yaw = 0; // 초깃값
>>>>>>> 05c80a4fc003bacc82908bd73c4050e4d51c2395

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

// 함수: Yaw (Heading) 계산
float calculateYaw(float normMx, float normMy, float normMz, float roll, float pitch) {
    // Roll (θ)와 Pitch (φ)를 라디안으로 변환
    float theta = roll * M_PI / 180.0; // Roll
    float phi = pitch * M_PI / 180.0; // Pitch

    // 논문 공식에 따라 보정된 Yaw 계산
    float numerator = -normMy * cos(phi) + normMz * sin(phi);
    float denominator = normMx * cos(theta) + normMy * sin(theta) * sin(phi) + normMz * sin(theta) * cos(phi);
    float yaw = atan2(numerator, denominator) * 180.0 / M_PI; // 라디안을 도 단위로 변환

    // Yaw 값이 음수일 경우 0~360도로 변환
    if (yaw < 0) yaw += 360;

    return yaw;
}

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

void calibrateSensors() {
  Serial.println("Calibrating sensors... Place the toothbrush in front teeth position.");
  delay(5000); // 사용자가 위치를 맞추는 시간을 제공

  float ax, ay, az, gx, gy, gz, mx, my, mz;

  int numSamples = 100;

  for (int i = 0; i < numSamples; i++) {
    if (IMU.accelerationAvailable() && IMU.magneticFieldAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);
      IMU.readMagneticField(mx, my, mz);
      float roll = atan2(ay, az) * 180 / PI;
      float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;
      float heading = atan2(-my, mx) * 180 / PI;
      if (heading < 0) heading += 360;

      base_roll += roll;
      base_pitch += pitch;
      base_yaw += heading;
    }
    delay(10);
  }

  base_roll /= numSamples;
  base_pitch /= numSamples;
  base_yaw /= numSamples;

  Serial.println("Calibration completed.");
  Serial.print("Base Roll: ");
  Serial.print(base_roll);
  Serial.print("°, Base Pitch: ");
  Serial.print(base_pitch);
  Serial.print("°, Base Yaw: ");
  Serial.println(base_yaw);
}

// 센서 초기화
void setup() {
  Serial.begin(9600);
  while (!Serial);

  // BMI270 + BMM150 초기화
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // initKalman(kalmanRoll);
  // initKalman(kalmanPitch);

<<<<<<< HEAD
  calibrateSensors();

  Serial.println("Sensors initialized and calibrated.");
}

void loop() {
  // static unsigned long prevTime = 0;
  // unsigned long currentTime = millis();
  // float dt = (currentTime - prevTime) / 1000.0; // 초 단위로 시간 계산
  // prevTime = currentTime;

  float ax, ay, az, gx, gy, gz, mx, my, mz;

  // // IMU 데이터 읽기
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);
  //   // 보정값 갱신 (최소/최대값)
  //   if (mx < magX_min) magX_min = mx;
  //   if (mx > magX_max) magX_max = mx;
  //   if (my < magY_min) magY_min = my;
  //   if (my > magY_max) magY_max = my;
  //   if (mz < magZ_min) magZ_min = mz;
  //   // 가속도계로 계산한 Roll, Pitch
  //   float accelRoll = atan2(ay, az) * 180 / PI;
  //   float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

  //   // 자이로스코프 각속도
  //   float gyroRollRate = gx; // 각속도 (°/s)
  //   float gyroPitchRate = gy;

  //   // 칼만 필터로 Roll, Pitch 계산
  //   float roll = updateKalman(kalmanRoll, accelRoll, gyroRollRate, dt);
  //   float pitch = updateKalman(kalmanPitch, accelPitch, gyroPitchRate, dt);

  //   // 자기 센서 보정
  //   float normMx = (mx - (magX_min + magX_max) / 2) / (magX_max - magX_min);
  //   float normMy = (my - (magY_min + magY_max) / 2) / (magY_max - magY_min);
  //   float normMz = (mz - (magZ_min + magZ_max) / 2) / (magZ_max - magZ_min);

  //   // Heading (방위각) 계산
  //   float heading = calculateYaw(normMx, normMy, normMz, roll, pitch);

    // 칼만 필터 안 쓴 버전
    float roll = atan2(ay, az) * 180 / PI;
    float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;
    float heading = atan2(-my, mx) * 180 / PI;
    if (heading < 0) heading += 360;

    // 칫솔의 상태 분류
    String posture = "", result= "";
    if (roll >= -45 && roll < 45) {
      posture = "BRUSH_DOWN"; 
      if (heading >= 100 && heading < 120 ) {
        result = "완쪽 아래 어금니 씹는면";
      }
      if (heading >= 130 && heading < 150) {
        result = "오른쪽 아래 어금니 씹는면";
      }
    } 
    else if (roll >= 45 && roll < 135) {
      posture = "BRUSH_LEFT"; 
      if (heading >= 160 && heading < 180) {
          if(roll >= 90) {
          result = "아랫쪽 ";
        } else {
          result = "윗쪽 ";
        }
        result += "앞니";
      }
      if (heading >= 140 && heading < 160) { // 0 ~ 40  왼쪽 송곳니
        if(roll >= 90) {
          result = "윗쪽 ";
        } else {
          result = "아랫쪽 ";
        }
        result += "왼쪽 송곳니";
      }
      if (heading >= 80 && heading <110) { // 40 ~ 80 왼쪽 어금니
        if(roll >= 90) {
          result = "아랫쪽 ";
        } else {
          result = "윗쪽 ";
        }
        result += "왼쪽 어금니";
      }
    } 
    else if (roll >= -135 && roll < -45) {
      posture = "BRUSH_RIGHT";
      if (heading >= 140 && heading < 180) { // 140 ~ 170 앞니
        if(roll >= -90) {
          result = "윗쪽 ";
        } else {
          result = "아랫쪽 ";
        }
        result += "앞니";
      }
      if (heading >= 10 && heading < 40) { // 0 ~ 40  오른쪽 송곳니
        if(roll >= -90) {
          result = "윗쪽 ";
        } else {
          result = "아랫쪽 ";
        }
        result += "오른쪽 송곳니";
      }
      if (heading >= 45 && heading <90) { // 40 ~ 80 오른쪽 어금니
        if(roll >= -90) {
          result = "아랫쪽 ";
        } else {
          result = "윗쪽 ";
        }
        result += "오른쪽 어금니";
      }
    } 
    else {
      posture = "BRUSH_UP"; 
      if (heading >= 90 && heading < 120) {
        result = "왼쪽 위 어금니 씹는면";
      }
      if (heading >= 30 && heading < 60) {
        result = "오른쪽 위 어금니 씹는면";
      }
    }


    // 결과 출력
    Serial.print("기준값 roll: ");
    Serial.print(base_roll);
    Serial.print("°, 기준값 pitch: ");
    Serial.print(base_pitch);
    Serial.print("°, 기준값 yaw: ");
    Serial.print(base_yaw);
    Serial.print("°   /    Roll: ");
    Serial.print(roll);
    Serial.print("°, Pitch: ");
    Serial.print(pitch);    
    Serial.print("°, Heading: ");
    Serial.print(heading);
    Serial.print("° -> Posture: ");
    Serial.print(posture);
    Serial.print(", Result: ");
    Serial.print(result);
    Serial.print(", 기준값과 현재 yaw값 차이");
    Serial.println(base_yaw - heading);
  }

  delay(10); // 10ms 주기로 데이터 수집
}
