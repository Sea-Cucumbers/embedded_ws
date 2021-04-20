#include "RoboClaw.h"

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

void drive_motor_0(uint8_t num, float rps){
  int sp = (int)(rps * ppr);
  if (num == MOTOR1_1){
    roboclaw1.SpeedM1(address1, sp);
  }
  else if (num == MOTOR1_2){
    roboclaw1.SpeedM2(address1, sp);
  }
  else if (num == MOTOR2_1){
    roboclaw2.SpeedM1(address2, sp);
  }
  else{
    roboclaw2.SpeedM2(address2, sp);
  }
  //delay(10);
}

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

void setup() {
  
  // put your setup code here, to run once:
  Serial.begin(57600);
  roboclaw1.begin(38400);
  roboclaw2.begin(38400);
  //Set PID Coefficients
  roboclaw1.SetM1VelocityPID(address1,Kd_1_1,Kp_1_1,Ki_1_1,qpps_1_1);
  roboclaw1.SetM2VelocityPID(address1,Kd_1_2,Kp_1_2,Ki_1_2,qpps_1_2);
  roboclaw2.SetM1VelocityPID(address2,Kd_2_1,Kp_2_1,Ki_2_1,qpps_2_1);
  roboclaw2.SetM2VelocityPID(address2,Kd_2_2,Kp_2_2,Ki_2_2,qpps_2_2);
 start = millis();
  
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

void FB(int num){
  // when num is positive, robot goes right
  // i.e. 1_1 is forward, rotating ccw
  //      1_2 is backward, rotating cw
  if (num > 0){
    roboclaw1.ForwardM1(address1, num); 
    roboclaw1.BackwardM2(address1, num);
  }
  else {
    roboclaw1.BackwardM1(address1, -num);
    roboclaw1.ForwardM2(address1, -num);
  }
}

void LR(int num){
  if (num > 0){
    roboclaw2.ForwardM1(address2, num);
    roboclaw2.BackwardM2(address2, num);
  }
  else{
    roboclaw2.BackwardM1(address2, -num);
    roboclaw2.ForwardM2(address2, -num);
  }
}

void turn(int num){
  if (num > 0){
    roboclaw1.ForwardM1(address1, num);
    roboclaw1.ForwardM2(address1, num);
    roboclaw2.ForwardM1(address2, num);
    roboclaw2.ForwardM2(address2, num);
  }
  else{
    roboclaw1.BackwardM1(address1, -num);
    roboclaw1.BackwardM2(address1, -num);
    roboclaw2.BackwardM1(address2, -num);
    roboclaw2.BackwardM2(address2, -num);
  }
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

void loop() {
  // at the beginning, we need the robot angular velocity
  // as we let the motors turn, we get each motor's speed and convert to rad/s
  // we then input them to v_wheels_to_robot function
  
  // put your main code here, to run repeatedly:
  if (flag == true){
    roboclaw1.ForwardM1(address1, 0);
    roboclaw1.BackwardM2(address1, 0);
    roboclaw2.ForwardM1(address2, 0);
    roboclaw2.BackwardM2(address2, 0);
  }
  if (millis()-start < 4000){
    //FB(-64);
    vector3 spe;
    spe.vx = 0; // inch per second
    spe.vy = PI/16/0.0254;
    spe.w = PI/8;
    vector4 s_wheels;
    v_robot_to_wheels(spe, ROBOT_R, WHEEL_R, s_wheels);
    /*
    vector4 w_wheels;
    w_wheels.w1 = 0;
    w_wheels.w2 = PI/2;
    w_wheels.w3 = 0;
    w_wheels.w4 = -PI/2;
    */
    command_speed(s_wheels);
    displayspeed();
  }
  
  else {
    flag = true;
  }
  
}
