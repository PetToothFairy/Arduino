#include <Arduino_LSM9DS1.h> // LSM9DS1 라이브러리

// 자기 센서 보정을 위한 변수
float magX_min = -50, magX_max = 50;
float magY_min = -50, magY_max = 50;
float magZ_min = -50, magZ_max = 50;

// 필터 상태 변수
float roll = 0, pitch = 0; // 상보 필터를 통해 계산된 roll, pitch 값
unsigned long prevTime = 0; // 이전 시간 저장

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.println("IMU initialized.");
}

void loop() {
  float ax, ay, az, gx, gy, gz, mx, my, mz;

  // 현재 시간
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0; // 초 단위로 시간 계산
  prevTime = currentTime;

  // IMU 데이터 읽기
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);

    // 가속도계로 계산한 Roll, Pitch
    float accelRoll = atan2(ay, az) * 180 / PI;
    float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

    // 자이로스코프 데이터를 기반으로 Roll, Pitch 변화량 계산
    float gyroRollRate = gx; // 자이로스코프 각속도 (°/s)
    float gyroPitchRate = gy;

    // 상보 필터를 통해 Roll, Pitch 융합
    roll = 0.98 * (roll + gyroRollRate * dt) + 0.02 * accelRoll;
    pitch = 0.98 * (pitch + gyroPitchRate * dt) + 0.02 * accelPitch;

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
