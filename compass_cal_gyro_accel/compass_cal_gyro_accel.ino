#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// Magnetometer calibration variables
float mag_min_x = 1000, mag_max_x = -1000;
float mag_min_y = 1000, mag_max_y = -1000;
float mag_min_z = 1000, mag_max_z = -1000;
float mag_offset_x = 0, mag_offset_y = 0, mag_offset_z = 0;

void setupSensor()
{
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G, lsm.LSM9DS1_ACCELDATARATE_10HZ);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}

void calibrateMagnetometer()
{
  Serial.println("Calibrating magnetometer... Rotate sensor 30 seconds in all directions!");
  unsigned long startTime = millis();

  while (millis() - startTime < 30000)  // Run for 30 seconds
  {
    lsm.read();
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);

    // Track min/max values
    if (m.magnetic.x < mag_min_x) mag_min_x = m.magnetic.x;
    if (m.magnetic.x > mag_max_x) mag_max_x = m.magnetic.x;
    if (m.magnetic.y < mag_min_y) mag_min_y = m.magnetic.y;
    if (m.magnetic.y > mag_max_y) mag_max_y = m.magnetic.y;
    if (m.magnetic.z < mag_min_z) mag_min_z = m.magnetic.z;
    if (m.magnetic.z > mag_max_z) mag_max_z = m.magnetic.z;

    Serial.print(".");
    delay(250);
  }

  // Compute offsets
  mag_offset_x = (mag_min_x + mag_max_x) / 2.0;
  mag_offset_y = (mag_min_y + mag_max_y) / 2.0;
  mag_offset_z = (mag_min_z + mag_max_z) / 2.0;

  Serial.println("\nCalibration complete!");
  Serial.print("Offsets -> X: "); Serial.print(mag_offset_x, 2);
  Serial.print(", Y: "); Serial.print(mag_offset_y, 2);
  Serial.print(", Z: "); Serial.println(mag_offset_z, 2);
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) { delay(1); }

  Serial.println("LSM9DS1 Compass with Magnetometer & Gyroscope Display");

  if (!lsm.begin())
  {
    Serial.println("Error: Unable to initialize LSM9DS1!");
    while (1);
  }

  Serial.println("LSM9DS1 Initialized");
  setupSensor();
  
  // Run magnetometer calibration
  calibrateMagnetometer();
}

void loop()
{
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);

  // Apply magnetometer calibration offsets
  float mag_x = m.magnetic.x - mag_offset_x;
  float mag_y = m.magnetic.y - mag_offset_y;
  float mag_z = m.magnetic.z - mag_offset_z;

  // Compute heading using only the magnetometer
  float heading = atan2(mag_y, mag_x) * 180.0 / PI;
  if (heading < 0) heading += 360;

  // Print formatted values with consistent width
  Serial.print("Heading: ");
  Serial.printf("%6.2f", heading);  // Print heading with 2 decimal places, width 6
  Serial.print("°\t");

  // Print gyroscope values with right alignment
  Serial.print("Gyro X: "); Serial.printf("%7.2f", g.gyro.x); 
  Serial.print(" rad/s, Y: "); Serial.printf("%7.2f", g.gyro.y);
  Serial.print(" rad/s, Z: "); Serial.printf("%7.2f", g.gyro.z);
  Serial.print(" rad/s\t");

  // Print gyroscope values with right alignment
  Serial.print("Accel X: "); Serial.printf("%7.2f", a.acceleration.x); 
  Serial.print(" m/s^2, Y: "); Serial.printf("%7.2f", a.acceleration.y);
  Serial.print(" m/s^2, Z: "); Serial.printf("%7.2f", a.acceleration.z);
  Serial.println(" m/s^2");

  delay(100);
}
