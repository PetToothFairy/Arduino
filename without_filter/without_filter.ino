#include <Arduino_LSM9DS1.h> // Arduino_BMI270_BMM150.h, Arduino_LSM9DS1.h

double base_roll = 0, base_pitch = 0, base_yaw_x = 0, base_yaw_y = 0, base_yaw = 0; // 초깃값
double count = 0;
unsigned long startTime;
bool isCalibrated = false;

// 센서 초기화
void setup() {
  Serial.begin(9600);
  while (!Serial);

  // BMI270 + BMM150 초기화
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.println("Sensors initialized.");

  startTime = millis(); // 프로그램 시작 시간 초기화
}

void loop() {
  float ax, ay, az, gx, gy, gz, mx, my, mz;

  // IMU 데이터 읽기
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);
  IMU.readMagneticField(mx, my, mz);

  double raw_roll = atan2(ay, az);  // Roll = atan2(ay, az)
  double raw_pitch = atan2(-ax, sqrt(ay * ay + az * az));  // Pitch = atan2(-ax, sqrt(ay^2 + az^2))

  double roll = raw_roll * 180 / PI;
  double pitch = raw_pitch * 180 / PI;

  // 지자기 센서 값을 사용하여 Yaw 계산
  // Roll, Pitch가 적용된 자기장 X, Y 값
  double mx_rot = mx * cos(raw_pitch) + mz * sin(raw_pitch);
  double my_rot = mx * sin(raw_roll) * sin(raw_pitch) + my * cos(raw_roll) - mz * sin(raw_roll) * cos(raw_pitch);

  // Yaw 계산
  double heading = atan2(my_rot, mx_rot);  // Yaw = atan2(my_rot, mx_rot)
  heading = heading * 180.0 / PI;  // 라디안 -> 도 변환
  if (heading < 0) {
    heading += 360.0;  // 0~360도로 보정
  }

  // 3초 동안 캘리브레이션
  if (!isCalibrated) {
    // 3초 동안 평균 계산
    if (millis() - startTime <= 3000) {
      count += 1;
      base_roll += roll;
      base_pitch += pitch;
      base_yaw_x += cos(heading * PI / 180); // yaw를 단위 벡터로 변환 후 합산
      base_yaw_y += sin(heading * PI / 180);
    }

    if (millis() - startTime > 3000) {
      base_roll /= count;
      base_pitch /= count;

      // 단위 벡터 평균으로 기준 yaw 계산
      double avg_x = base_yaw_x / count;
      double avg_y = base_yaw_y / count;
      base_yaw = atan2(avg_y, avg_x) * 180 / PI;
      if (base_yaw < 0) base_yaw += 360;

      isCalibrated = true;
      Serial.println("Calibration Complete!");
      Serial.print("Base Roll: "); Serial.print(base_roll);
      Serial.print(", Base Pitch: "); Serial.print(base_pitch);
      Serial.print(", Base Yaw: "); Serial.println(base_yaw);
    }
  }

  if (isCalibrated) {
    // 360도 순환성을 고려한 차이 계산
    double yaw_diff = heading - base_yaw;
    if (yaw_diff > 180) {
        yaw_diff -= 360;
    } else if (yaw_diff < -180) {
        yaw_diff += 360;
    }

    double pitch_diff = pitch - base_pitch;

    // 180도를 기준으로 음수 방향 변환
    if (yaw_diff > 180) {
        yaw_diff -= 360;
    }

    String posture = "", result = "";
    if (roll >= -45 && roll < 45) {
      posture = "BRUSH_RIGHT";
      if ((yaw_diff >= -25 && yaw_diff < 25)) { // 140 ~ 170 앞니
        if (roll >= 0) {
          result = "아랫쪽 ";
        } else {
          result = "윗쪽 ";
        }
        result += "앞니";
      }
      if ( (yaw_diff >= -80 && yaw_diff < -25) ) { // 0 ~ 40  오른쪽 송곳니
        if (roll >= 0) {
          result = "아랫쪽 ";
        } else {
          result = "윗쪽 ";
        }
        result += "오른쪽 송곳니";
      }
      if ( (yaw_diff >= -140 && yaw_diff < -80) ) { // 40 ~ 80 오른쪽 어금니
        if (roll >= 0) {
          result = "아랫쪽 ";
        } else {
          result = "윗쪽 ";
        }
        result += "오른쪽 어금니";
      }
    }

    else if (roll >= 45 && roll < 135) {
      posture = "BRUSH_UP";
      if (roll >= 45 && roll < 90) {
        result = "왼쪽 위 어금니 씹는면";
      }
      if (roll >=90 && roll < 130) {
        result = "오른쪽 위 어금니 씹는면";
      }
    }

    else if (roll >= -135 && roll < -45) {
      posture = "BRUSH_DOWN";
      if(roll >= -90 && roll < -45) {
        result = "오른쪽 아래 어금니 씹는면";
      }
      if (roll >= -135 && roll <-90){
        result = "왼쪽 아래 어금니 씹는면";
      }
    }

    else {
      posture = "BRUSH_LEFT";
      if ((yaw_diff >= 150 || yaw_diff < -150) ) { 
        if (roll >= 90) {
          result = "아랫쪽 ";
        } else {
          result = "윗쪽 ";
        }
        result += "앞니";
      }
      if ( (yaw_diff >= -115 && yaw_diff < -50) ) { 
        if (roll >= 90) {
          result = "윗쪽 ";
        } else {
          result = "아랫쪽 ";
        }
        result += "왼쪽 어금니";
      }
      if ( (yaw_diff >= -150 && yaw_diff < -115)  ) { 
        if (roll >= 90) {
          result = "아랫쪽 ";
        } else {
          result = "윗쪽 ";
        }
        result += "왼쪽 송곳니";
      }
    }

    // 결과 출력
    Serial.print("Result: ");
    Serial.println(result);
    // Serial.print("  / 기준값 yaw: ");
    // Serial.print(base_yaw);
    // Serial.print("Roll: ");
    // Serial.print(roll);
    // Serial.print("°, Pitch: ");
    // Serial.print(pitch);
    // Serial.print("°, Heading: ");
    // Serial.print(heading);
    // Serial.print("° -> Posture: ");
    // Serial.print(posture);
    // Serial.print(", 기준값과 현재 yaw값 차이");
    // Serial.println(yaw_diff);
  }
  

  delay(10); // 10ms 주기로 데이터 수집
}
