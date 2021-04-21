#include "RoboClaw.h"
#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_LSM6DS33.h>

// some motor control constants
#define address1 0x80
#define address2 0x84

#define MOTOR1_1 4
#define MOTOR1_2 2
#define MOTOR2_1 1
#define MOTOR2_2 3

//Velocity PID coefficients.
#define Kp_1_1 6.3865
#define Ki_1_1 0.91062
#define Kd_1_1 0.00
#define qpps_1_1 3187

#define Kp_1_2 6.78877
#define Ki_1_2 1.00573
#define Kd_1_2 0.00
#define qpps_1_2 3000

#define Kp_2_1 6.03917
#define Ki_2_1 0.94777
#define Kd_2_1 0.00
#define qpps_2_1 3000

#define Kp_2_2 6.03917
#define Ki_2_2 0.88347
#define Kd_2_2 0.00
#define qpps_2_2 2625

double ppr = 1425.1;
double ppr_int = 1425;
long start = 0;
bool flag = false;
double qpps_max = 1800;
int qpps_max_int = 1800;

double WHEEL_R = 2; // inches? TODO check this
double ROBOT_R = 7; // distance from center of chassis to wheel

typedef struct vec3{
  double vx;
  double vy;
  double w;
} vector3;

typedef struct vec4{
  double w1; // 2.1 vx points to w1
  double w2; // 1.2 vy points to w2
  double w3; // 2.1
  double w4; // 1.1
} vector4;

RoboClaw roboclaw1(&Serial3,10000);
RoboClaw roboclaw2(&Serial2,10000);

// Sensor 0 same side as realsense, go counterclockwise
#define SHUTDOWN0 13
#define SHUTDOWN1 10
#define SHUTDOWN2 11
#define SHUTDOWN3 12

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

vector3 spe;

// Serial stuff
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

      // variables to hold the parsed data
char messageFromPC[numChars] = {0};

boolean newData = false;

//some motor control helper functions

// return absolute encoder reading in radians
float get_angle(uint8_t num, bool *valid){
  uint8_t enc_status;
  bool enc_valid;
  int32_t enc_val;
  float enc_val_res;
  if (num == MOTOR1_1){
    enc_val = roboclaw1.ReadEncM1(address1, &enc_status, &enc_valid);
  }
  else if (num == MOTOR1_2){
    enc_val = roboclaw1.ReadEncM2(address1, &enc_status, &enc_valid);
  }
  else if (num == MOTOR2_1){
    enc_val = roboclaw2.ReadEncM1(address2, &enc_status, &enc_valid);
  }
  else{
    enc_val = roboclaw2.ReadEncM2(address2, &enc_status, &enc_valid);
  }
  *valid = enc_valid;
  enc_val_res = (float)enc_val / ppr;
  return enc_val_res;
}

void displayspeed(void)
{
  bool enc1_1_valid, enc1_2_valid, enc2_1_valid, enc2_2_valid;
  bool speed1_1_valid, speed1_2_valid, speed2_1_valid, speed2_2_valid;
  uint8_t speed1_1_status, speed1_2_status, speed2_1_status, speed2_2_status;
  float enc1_1= get_angle(MOTOR1_1, &enc1_1_valid);
  float enc1_2= get_angle(MOTOR1_2, &enc1_2_valid);
  float enc2_1= get_angle(MOTOR2_1, &enc2_1_valid);
  float enc2_2= get_angle(MOTOR2_2, &enc2_2_valid);
  int32_t speed1_1 = roboclaw1.ReadSpeedM1(address1, &speed1_1_status, &speed1_1_valid);
  int32_t speed1_2 = roboclaw1.ReadSpeedM2(address1, &speed1_2_status, &speed1_2_valid);
  int32_t speed2_1 = roboclaw2.ReadSpeedM1(address2, &speed2_1_status, &speed2_1_valid);
  int32_t speed2_2 = roboclaw2.ReadSpeedM2(address2, &speed2_2_status, &speed2_2_valid);
  Serial.print("Encoder1_1:");
  if(enc1_1_valid){
    Serial.print(enc1_1);
    Serial.print(" ");
    Serial.print(enc1_1 * (2*PI), 3);
    Serial.print(" ");
    if (speed1_1_valid){
      Serial.print(speed1_1);
    }
    else{
      Serial.print("Invalid");
    }
  }
  else{
    Serial.print("invalid");
  }
  Serial.print("   ");
  Serial.print("Encoder1_2:");
  if(enc1_2_valid){
    Serial.print(enc1_2);
    Serial.print(" ");
    Serial.print(enc1_2 * (2*PI), 3);
    Serial.print(" ");
    if (speed1_2_valid){
      Serial.print(speed1_2);
    }
    else{
      Serial.print("Invalid");
    }
  }
  else{
    Serial.print("invalid");
  }
  Serial.print("   ");
  Serial.print("Encoder2_1:");
  if(enc2_1_valid){
    Serial.print(enc2_1);
    Serial.print(" ");
    Serial.print(enc2_1 * (2*PI), 3);
    Serial.print(" ");
    if (speed2_1_valid){
      Serial.print(speed2_1);
    }
    else{
      Serial.print("Invalid");
    }
  }
  else{
    Serial.print("invalid");
  }
  Serial.print("   ");

  
  Serial.print("Encoder2_2:");
  if(enc2_2_valid){
    Serial.print(enc2_2);
    Serial.print(" ");
    Serial.print(enc2_2 * (2*PI), 3);
    Serial.print(" ");
    if (speed2_2_valid){
      Serial.print(speed2_2);
    }
    else{
      Serial.print("Invalid");
    }
  }
  else{
    Serial.print("invalid");
  }
  Serial.println();
}

// takes in w in rad/s, outputs speed command
int w_to_speed_cmd(double w){
  int output = 0; // should be in -127 ~ 127
  double rps = w/(2*PI);
  double qpps_cmd = rps * ppr;
  int qpps_int = (int)qpps_cmd;
  if (qpps_int > qpps_max_int)
    qpps_int = qpps_max_int; 
  else if (qpps_int < -qpps_max_int)
    qpps_int = -qpps_max_int;
  if (qpps_int > 0)
    output = map(qpps_int, 0, qpps_max_int, 0, 127);
  else
    output = map(qpps_int, 0, -qpps_max_int, 0, -127);
  return output;
}

void drive_motor(int motor_num, int cmd){
  if (motor_num == MOTOR1_1){ // w4
    if (cmd > 0)
      roboclaw1.ForwardM1(address1, cmd);
    else
      roboclaw1.BackwardM1(address1, -cmd);
  }
  else if (motor_num == MOTOR1_2){ // w2
    if (cmd > 0)
      roboclaw1.ForwardM2(address1, cmd);
    else
      roboclaw1.BackwardM2(address1, -cmd);
  }
  else if (motor_num == MOTOR2_1){ // w1
    if (cmd > 0)
      roboclaw2.ForwardM1(address2, cmd);
    else
      roboclaw2.BackwardM1(address2, -cmd);
  }
  else { // w3
    if (cmd > 0)
      roboclaw2.ForwardM2(address2, cmd);
    else
      roboclaw2.BackwardM2(address2, -cmd);
  }
}

void command_speed(vector4 w_wheels){
  int w1_cmd = w_to_speed_cmd(w_wheels.w1);
  int w2_cmd = w_to_speed_cmd(w_wheels.w2);
  int w3_cmd = w_to_speed_cmd(w_wheels.w3);
  int w4_cmd = w_to_speed_cmd(w_wheels.w4);
  drive_motor(1, w1_cmd);
  drive_motor(2, w2_cmd);
  drive_motor(3, w3_cmd);
  drive_motor(4, w4_cmd);
}

void v_world_to_robot(vector3 &v_world, double theta, vector3 &v_robot){
  v_robot.vx = cos(theta)*v_world.vx + sin(theta)*v_world.vy;
  v_robot.vy = -sin(theta)*v_world.vx + cos(theta)*v_world.vy;
  v_robot.w = v_world.w;
}

void v_robot_to_wheels(vector3 &v_robot, double d, double r, vector4 &w_wheels){
  w_wheels.w1 = (v_robot.vy + d*v_robot.w)/r;
  w_wheels.w2 = (-v_robot.vx + d*v_robot.w)/r;
  w_wheels.w3 = (-v_robot.vy + d*v_robot.w)/r;
  w_wheels.w4 = (v_robot.vx + d*v_robot.w)/r;
}

double v_wheels_to_robot(vector4 &w_wheels, double d, double r, vector3 &v_robot){
  v_robot.vx = 0.5*(w_wheels.w4 - w_wheels.w2)*r;
  v_robot.vy = 0.5*(w_wheels.w1 - w_wheels.w3)*r;
  v_robot.w = (w_wheels.w1 + w_wheels.w2 + w_wheels.w3 + w_wheels.w4)*r/(4*d);
}



void setup() {
  // motor control setup
  roboclaw1.begin(38400);
  roboclaw2.begin(38400);
  //Set PID Coefficients
  roboclaw1.SetM1VelocityPID(address1,Kd_1_1,Kp_1_1,Ki_1_1,qpps_1_1);
  roboclaw1.SetM2VelocityPID(address1,Kd_1_2,Kp_1_2,Ki_1_2,qpps_1_2);
  roboclaw2.SetM1VelocityPID(address2,Kd_2_1,Kp_2_1,Ki_2_1,qpps_2_1);
  roboclaw2.SetM2VelocityPID(address2,Kd_2_2,Kp_2_2,Ki_2_2,qpps_2_2);
  spe.vx = 0;
  spe.vy = 0;
  spe.w = 0;

  // tof setup
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
    Serial.println("Failed to detect and initialize sensor 0!");
    while (1);
  }
  sensor0.setAddress(ADDR0);

  digitalWrite(SHUTDOWN1, HIGH);
  sensor1.setTimeout(500);
  if (!sensor1.init())
  {
    Serial.println("Failed to detect and initialize sensor 1!");
    while (1);
  }
  sensor1.setAddress(ADDR1);

  digitalWrite(SHUTDOWN2, HIGH);
  sensor2.setTimeout(500);
  if (!sensor2.init())
  {
    Serial.println("Failed to detect and initialize sensor 2!");
    while (1);
  }
  sensor2.setAddress(ADDR2);

  digitalWrite(SHUTDOWN3, HIGH);
  sensor3.setTimeout(500);
  if (!sensor3.init())
  {
    Serial.println("Failed to detect and initialize sensor 3!");
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

void loop() {
  // Send feedback to RPi
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);

  yaw += (gyro.gyro.z - bias)*((millis() - imu_time)/1000.0);
  Serial.print(-yaw);
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

  // Receive command from computer, then put into spe
  recvWithStartEndMarkers();
  if (newData == true) {
      strcpy(tempChars, receivedChars);
          // this temporary copy is necessary to protect the original data
          //   because strtok() used in parseData() replaces the commas with \0
      parseData();
      newData = false;
  }
  
  vector4 s_wheels;
  v_robot_to_wheels(spe, ROBOT_R, WHEEL_R, s_wheels);
  command_speed(s_wheels);
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//============

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars," ");      // get the first part - the string
    spe.vx = atof(strtokIndx); // copy it to messageFromPC

    strtokIndx = strtok(NULL, " "); // this continues where the previous call left off
    spe.vy = atof(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, " ");
    spe.w = atof(strtokIndx);     // convert this part to a float
}
