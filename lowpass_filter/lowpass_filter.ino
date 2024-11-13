#include <Arduino_LSM9DS1.h> // LSM9DS1 라이브러리

// 자기 센서 보정을 위한 변수
float magX_min = -50, magX_max = 50;
float magY_min = -50, magY_max = 50;
float magZ_min = -50, magZ_max = 50;

// 필터된 값 저장용 변수
float filteredAx = 0, filteredAy = 0, filteredAz = 0;

// 필터 함수 (저역 필터)
float lowPassFilter(float previousValue, float newValue, float alpha) {
  return alpha * newValue + (1 - alpha) * previousValue;
}

// 센서 초기화
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
  float ax, ay, az, mx, my, mz;

  // IMU 데이터 읽기
  if (IMU.accelerationAvailable() && IMU.magneticFieldAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readMagneticField(mx, my, mz);

    // 가속도계 데이터 필터링 (저역 필터 적용)
    filteredAx = lowPassFilter(filteredAx, ax, 0.2);
    filteredAy = lowPassFilter(filteredAy, ay, 0.2);
    filteredAz = lowPassFilter(filteredAz, az, 0.2);

    // Roll 계산
    float roll = atan2(filteredAy, filteredAz) * 180 / PI;

    // Pitch 계산
    float pitch = atan2(-filteredAx, sqrt(filteredAy * filteredAy + filteredAz * filteredAz)) * 180 / PI;

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

  delay(100); // 100ms 주기로 데이터 수집
}
