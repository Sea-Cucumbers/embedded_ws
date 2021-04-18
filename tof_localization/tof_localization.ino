/*
This example shows how to take simple range measurements with the VL53L1X. The
range readings are in units of mm.
*/

#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_LSM6DS33.h>

#define SHUTDOWN0 2
#define SHUTDOWN1 3
#define SHUTDOWN2 4
#define SHUTDOWN3 5


#define ADDR0 2
#define ADDR1 3
#define ADDR2 4
#define ADDR3 5

VL53L1X sensor0;
VL53L1X sensor1;
VL53L1X sensor2;
VL53L1X sensor3;

Adafruit_LSM6DS33 lsm6ds33;
float bias = 0;
float yaw = 0;
unsigned long imu_time = 0;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  // Turn off all sensors and begin address initialization
  pinMode(SHUTDOWN0, OUTPUT);
  pinMode(SHUTDOWN1, OUTPUT);
  pinMode(SHUTDOWN2, OUTPUT);
  pinMode(SHUTDOWN3, OUTPUT);

  digitalWrite(SHUTDOWN0, LOW);
  digitalWrite(SHUTDOWN1, LOW);
  digitalWrite(SHUTDOWN2, LOW);
  digitalWrite(SHUTDOWN3, LOW);
  delay(1000);

  // Bring each sensor online and change its address before bringing the next one online
  digitalWrite(SHUTDOWN0, HIGH);
  sensor0.setTimeout(500);
  if (!sensor0.init())
  {
    Serial.println("Failed to detect and initialize sensor 1!");
    while (1);
  }
  sensor0.setAddress(ADDR0);

  digitalWrite(SHUTDOWN1, HIGH);
  sensor1.setTimeout(500);
  if (!sensor1.init())
  {
    Serial.println("Failed to detect and initialize sensor 2!");
    while (1);
  }
  sensor1.setAddress(ADDR1);

  digitalWrite(SHUTDOWN2, HIGH);
  sensor2.setTimeout(500);
  if (!sensor2.init())
  {
    Serial.println("Failed to detect and initialize sensor 3!");
    while (1);
  }
  sensor2.setAddress(ADDR2);

  digitalWrite(SHUTDOWN3, HIGH);
  sensor3.setTimeout(500);
  if (!sensor3.init())
  {
    Serial.println("Failed to detect and initialize sensor 4!");
    while (1);
  }
  sensor3.setAddress(ADDR3);
  
  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor0.setDistanceMode(VL53L1X::Long);
  sensor0.setMeasurementTimingBudget(50000);
  sensor1.setDistanceMode(VL53L1X::Long);
  sensor1.setMeasurementTimingBudget(50000);
  sensor2.setDistanceMode(VL53L1X::Long);
  sensor2.setMeasurementTimingBudget(50000);
  sensor3.setDistanceMode(VL53L1X::Long);
  sensor3.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor0.startContinuous(50);
  sensor1.startContinuous(50);
  sensor2.startContinuous(50);
  sensor3.startContinuous(50);


  // Initialize IMU and calibrate bias
  if (!lsm6ds33.begin_I2C()) {
    Serial.println("Failed to find LSM6DS33 chip");
    while (1) {
      delay(10);
    }
  }
 
  lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  lsm6ds33.setGyroRange(LSM6DS_GYRO_RANGE_125_DPS);
  lsm6ds33.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  lsm6ds33.setGyroDataRate(LSM6DS_RATE_12_5_HZ);

  // First few IMU readings are garbage
  for (int i = 0; i < 10; ++i) {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    lsm6ds33.getEvent(&accel, &gyro, &temp);
    delay(100);
  }

  // Calibrate bias
  bias = 0;
  for (int i = 0; i < 10; ++i) {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    lsm6ds33.getEvent(&accel, &gyro, &temp);
  
    bias += gyro.gyro.z;
    delay(100);
  }

  bias /= 10;
  imu_time = millis();
  yaw = 0;
}

void loop()
{
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);

  yaw += (gyro.gyro.z - bias)*((millis() - imu_time)/1000.0);
  Serial.print(yaw);
  Serial.print(" ");
  imu_time = millis();
  
  Serial.print(sensor0.read()/10);
  Serial.print(" ");
  Serial.print(sensor1.read()/10);
  Serial.print(" ");
  Serial.print(sensor2.read()/10);
  Serial.print(" ");
  Serial.print(sensor3.read()/10);
  Serial.println();
}
