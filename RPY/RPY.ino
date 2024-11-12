#include <Arduino_LSM9DS1.h>

float Xm_max = -INFINITY;  // Initialize max value globally
float Xm_min = INFINITY;   // Initialize min value globally

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
  }

  // Magnetometer data
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
  }

  // Calculate Roll and Pitch
  float roll = atan2(ay, az) * 180 / PI;
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

  // Calculate Yaw
  float yaw = atan2(my, mx) * 180 / PI;

  // Update max and min values
  float Xm = mx;  // Current magnetic x-axis value
  if (Xm > Xm_max) {
    Xm_max = Xm;
  }
  if (Xm < Xm_min) {
    Xm_min = Xm;
  }

  const float latitude = 56.9; // Latitude value for magnetic inclination calculation
  // Calculate magnetic inclination angle
  float lambda = atan(2 * tan(latitude * PI / 180));  // Convert latitude to radians
  // Calculate heading angle
  float Bias_x = (Xm_max + Xm_min) / 2;                // Bias calculation
  float SF_x = (2 * cos(lambda)) / (Xm_max - Xm_min);  // Scale factor calculation
  float normalizedXm = (Xm - Bias_x) * SF_x;           // Normalized magnetic x-axis value

  // Heading angle calculation
  float heading = atan2(-(my * cos(pitch) + mz * sin(pitch)),
                        (normalizedXm * cos(roll) + my * sin(roll) * sin(pitch) + mz * sin(roll) * cos(pitch)));

  // Convert from radians to degrees
  heading = heading * 180.0 / PI;

  // Determine posture
  String posture = "NONE";

  if (-45 <= roll && roll < 45) {
    if (80 <= heading && heading < 110) {
      posture = "LEFT";
    } else if (60 <= heading && heading < 80) {
      posture = "RIGHT";
    } else if (170 <= heading && heading < 190) {
      posture = "FRONT";
    }
  } else if (45 <= roll && roll < 135) {
    posture = "UP";
  } else if (135 <= roll || roll < -135) {
    if (80 <= heading && heading < 110) {
      posture = "RIGHT";
    } else if (110 <= heading && heading < 120) {
      posture = "LEFT";
    } else if (350 <= heading || heading < 10) {
      posture = "FRONT";
    }
  } else if (-135 <= roll && roll < -45) {
    posture = "DOWN";
  }

  // Print results
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(", Pitch: ");
  Serial.print(pitch);
  Serial.print(", Yaw: ");
  Serial.print(yaw);
  Serial.print(", Heading: ");
  Serial.print(heading);
  Serial.print(", Posture: ");
  Serial.println(posture);

  delay(500);
}
