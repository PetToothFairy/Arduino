#include <Arduino_BMI270_BMM150.h>
#include <MadgwickAHRS.h> // Madgwick 필터 라이브러리

Madgwick filter; // 센서 융합 필터

float base_roll = 0, base_pitch = 0, base_yaw = 0; // 기준값
unsigned long startTime;
bool isCalibrated = false;
float yaw_offset = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // BMI270 + BMM150 초기화
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("Sensors initialized.");

  filter.begin(100); // Madgwick 필터 샘플 속도 설정 (100Hz)
  startTime = millis(); // 캘리브레이션 시작 시간
}

void loop() {
  float ax, ay, az, gx, gy, gz, mx, my, mz;

  // IMU 데이터 읽기
  if (!IMU.readAcceleration(ax, ay, az) ||
      !IMU.readGyroscope(gx, gy, gz) ||
      !IMU.readMagneticField(mx, my, mz)) {
    Serial.println("Failed to read IMU data!");
    return;
  }

  // Madgwick 필터 업데이트 (IMU 데이터를 사용한 센서 융합)
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

  // Roll, Pitch, Yaw 값 가져오기
  float roll = filter.getRoll(); // 단위: 도(degrees)
  float pitch = filter.getPitch();
  float yaw = filter.getYaw();

  // Yaw 값 범위를 0~360도로 변환
  yaw = fmod(yaw + 360.0, 360.0);

  // Yaw 값 보정 (기준값 설정)
  if (!isCalibrated) {
    if (millis() - startTime <= 3000) { // 3초 동안 캘리브레이션
      base_roll += roll;
      base_pitch += pitch;
      yaw_offset += yaw;
    } else {
      base_roll /= 300;
      base_pitch /= 300;
      yaw_offset /= 300;
      base_yaw = yaw_offset; // 기준 yaw 설정
      isCalibrated = true;
      Serial.println("Calibration Complete!");
      Serial.print("Base Roll: "); Serial.println(base_roll);
      Serial.print("Base Pitch: "); Serial.println(base_pitch);
      Serial.print("Base Yaw: "); Serial.println(base_yaw);
    }
    delay(10); // 캘리브레이션 샘플링 주기
    return;
  }

  // 기준값 기준으로 변환
  float roll_diff = roll - base_roll;
  float pitch_diff = pitch - base_pitch;
  float yaw_diff = yaw - base_yaw;

  // 360도 순환성 보정
  if (yaw_diff > 180) yaw_diff -= 360;
  else if (yaw_diff < -180) yaw_diff += 360;

  // 브러싱 위치 계산
  String posture = "", result = "";
  if (roll_diff >= -45 && roll_diff < 45) {
    posture = "BRUSH_RIGHT";
    if (yaw_diff >= -25 && yaw_diff < 25) result = "앞니";
    else if (yaw_diff >= -80 && yaw_diff < -25) result = "오른쪽 송곳니";
    else if (yaw_diff >= -140 && yaw_diff < -80) result = "오른쪽 어금니";
  } else if (roll_diff >= 45 && roll_diff < 135) {
    posture = "BRUSH_UP";
    result = roll_diff < 90 ? "왼쪽 위 어금니 씹는면" : "오른쪽 위 어금니 씹는면";
  } else if (roll_diff >= -135 && roll_diff < -45) {
    posture = "BRUSH_DOWN";
    result = roll_diff > -90 ? "오른쪽 아래 어금니 씹는면" : "왼쪽 아래 어금니 씹는면";
  } else {
    posture = "BRUSH_LEFT";
    if (yaw_diff >= 150 || yaw_diff < -150) result = "앞니";
    else if (yaw_diff >= -115 && yaw_diff < -50) result = "왼쪽 어금니";
    else if (yaw_diff >= -150 && yaw_diff < -115) result = "왼쪽 송곳니";
  }

  // 결과 출력
  Serial.print("Roll: "); Serial.print(roll_diff);
  Serial.print("°, Pitch: "); Serial.print(pitch_diff);
  Serial.print("°, Yaw: "); Serial.print(yaw_diff);
  Serial.print("° -> Posture: "); Serial.println(posture);
  // Serial.print("Detected Area: "); Serial.println(result);

  delay(10); // 10ms 주기
}
