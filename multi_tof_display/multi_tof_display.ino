/*
This example shows how to take simple range measurements with the VL53L1X. The
range readings are in units of mm.
*/

#include <Wire.h>
#include <VL53L1X.h>

#define SHUTDOWN1 2
#define SHUTDOWN2 3
#define SHUTDOWN3 4
#define SHUTDOWN4 5

VL53L1X sensor1;
VL53L1X sensor2;
VL53L1X sensor3;
VL53L1X sensor4;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  // Turn off all sensors and begin address initialization
  pinMode(SHUTDOWN1, OUTPUT);
  pinMode(SHUTDOWN2, OUTPUT);
  pinMode(SHUTDOWN3, OUTPUT);
  pinMode(SHUTDOWN4, OUTPUT);

  digitalWrite(SHUTDOWN1, LOW);
  digitalWrite(SHUTDOWN2, LOW);
  digitalWrite(SHUTDOWN3, LOW);
  digitalWrite(SHUTDOWN4, LOW);
  delay(1000);

  // Bring each sensor online and change its address before bringing the next one online
  digitalWrite(SHUTDOWN1, HIGH);
  sensor1.setTimeout(500);
  if (!sensor1.init())
  {
    Serial.println("Failed to detect and initialize sensor 1!");
    while (1);
  }
  sensor1.setAddress(1);

  digitalWrite(SHUTDOWN2, HIGH);
  sensor2.setTimeout(500);
  if (!sensor2.init())
  {
    Serial.println("Failed to detect and initialize sensor 2!");
    while (1);
  }
  sensor2.setAddress(2);

  digitalWrite(SHUTDOWN3, HIGH);
  sensor3.setTimeout(500);
  if (!sensor3.init())
  {
    Serial.println("Failed to detect and initialize sensor 3!");
    while (1);
  }
  sensor3.setAddress(3);

  digitalWrite(SHUTDOWN4, HIGH);
  sensor4.setTimeout(500);
  if (!sensor4.init())
  {
    Serial.println("Failed to detect and initialize sensor 4!");
    while (1);
  }
  sensor4.setAddress(4);
  
  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor1.setDistanceMode(VL53L1X::Long);
  sensor1.setMeasurementTimingBudget(50000);
  sensor2.setDistanceMode(VL53L1X::Long);
  sensor2.setMeasurementTimingBudget(50000);
  sensor3.setDistanceMode(VL53L1X::Long);
  sensor3.setMeasurementTimingBudget(50000);
  sensor4.setDistanceMode(VL53L1X::Long);
  sensor4.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor1.startContinuous(50);
  sensor2.startContinuous(50);
  sensor3.startContinuous(50);
  sensor4.startContinuous(50);
}

void loop()
{
  Serial.print(sensor1.read());
  Serial.print(" ");
  Serial.print(sensor2.read());
  Serial.print(" ");
  Serial.print(sensor3.read());
  Serial.print(" ");
  Serial.print(sensor4.read());
  Serial.println();
  if (sensor1.timeoutOccurred()) { Serial.println("TIMEOUT 1"); }
  if (sensor2.timeoutOccurred()) { Serial.println("TIMEOUT 2"); }
  if (sensor3.timeoutOccurred()) { Serial.println("TIMEOUT 3"); }
  if (sensor4.timeoutOccurred()) { Serial.println("TIMEOUT 4"); }
}
