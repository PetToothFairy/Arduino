#include <Arduino_BMI270_BMM150.h> // Arduino_BMI270_BMM150.h, Arduino_LSM9DS1.h

bool isStartInitialized = false;
int num = 1;

void setup() {
  Serial.begin(9600);
  //while (!Serial);

  // IMU 초기화
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("Sensors initialized.");

}

void loop() {
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  num++;

  if (!isStartInitialized) {
    startTime = millis(); // 프로그램 시작 시간 초기화
    isStartInitialized = true;
  }

  if (millis() - startTime <= 20000) {
    // IMU 데이터 읽기
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);

    Serial.print(num);
    Serial.print(". ax: "); Serial.print(ax);
    Serial.print(", ay: "); Serial.print(ay);
    Serial.print(", az: "); Serial.print(az);
    Serial.print(", gx: "); Serial.print(gx);
    Serial.print(", gy: "); Serial.print(gy);
    Serial.print(", gz: "); Serial.print(gz);
    Serial.print(", mx: "); Serial.print(mx);
    Serial.print(", my: "); Serial.print(my);
    Serial.print(", mz: "); Serial.println(mz);
  }
  delay(10);
}
