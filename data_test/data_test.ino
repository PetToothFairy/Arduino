#include <Arduino_LSM9DS1.h>

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }
}

void loop() {
  float ax, ay, az;
  float mx, my, mz;

  // Accelerometer data
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    Serial.println("a success");
  }
  else{
    Serial.println("a fail");
  }

  // Magnetometer data
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
    Serial.println("m success");
  }
  else{
    Serial.println("m fail");
  }

  delay(500);
}
