// including the libraries
#include <Arduino_LSM9DS1.h>

// serial communication 
void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("ax,ay,az,gx,gy,gz,mx,my,mz");
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}


// main loop
void loop() {
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  // reading the accelerometer, gyroscope, magnetometer values
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);

    Serial.print(ax); Serial.print(',');
    Serial.print(ay); Serial.print(',');
    Serial.print(az); Serial.print(',');
    Serial.print(gx); Serial.print(',');
    Serial.print(gy); Serial.print(',');
    Serial.print(gz); Serial.print(',');
    Serial.print(mx); Serial.print(',');
    Serial.print(my); Serial.print(',');
    Serial.println(mz);
  }
}