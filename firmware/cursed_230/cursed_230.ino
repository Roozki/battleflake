


//#include "GY521.h"
#include <ros.h>
#include <Wire.h>
#include <Servo.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <AccelStepper.h>

//ros stuff

ros::NodeHandle nh;

std_msgs::Int16 heart;
std_msgs::Int16 IR1;
std_msgs::Int16 IR2;
std_msgs::Int16 IR3;
std_msgs::Int16 IR4;
std_msgs::Int16 IR5;





//geometry_msgs::TwistStamped imu;
geometry_msgs::Twist imu;
ros::Publisher heartbeat("heartbeat", &heart);
ros::Publisher ir_1("ir_sensors/IR1", &IR1);
ros::Publisher ir_2("ir_sensors/IR2", &IR2);
 ros::Publisher ir_3("ir_sensors/IR3", &IR3);
 ros::Publisher ir_4("ir_sensors/IR4", &IR4);
 ros::Publisher ir_5("ir_sensors/IR5", &IR5);
ros::Publisher imu_pub("imu_data", &imu);

//ros::Subscriber<motorIn1 std_msgs::Int16>;


Servo servo1; 
int counter;
int sdrivepin = 7; //small drive m

int motor[] = {0, 1};
int motorDirPin[] = {3,4};
int motorPulPin[] = {8,9};

int throttlePin = 19;
int aileronPin = 20;



int throttle = 0;
int aileron = 0;

int throttlestart;
int throttleee;
const int imu_address = 0x69; //I2C address of imu
//accelerometer


//std_msgs::Int16 acc_x;
//std_msgs::Int16 acc_y;
//std_msgs::Int16 acc_z; //exactly 16 bits
// std_msgs::Int16 gyro_x;
// std_msgs::Int16 gyro_y;
// std_msgs::Int16 gyro_z;
std_msgs::Float64 temperature; //hot hot hot

float acc_x;
float acc_y;
float acc_z;
float gyro_x;
float gyro_y;
float gyro_z;





char tmp_str[7];
//int IR1;
char* convert_int16_to_str(int16_t i) {
 sprintf(tmp_str, "%6d", i);
   return tmp_str;
}

//motor control

AccelStepper mot1;
//
// void driveMotor(int motor, int dir, int speed){
// int tmp = speed / 2;
//   digitalWrite(motorDirPin[motor], dir);

// if(micros() % speed == 0){
//  if(mircos() % tmp == 0){
//   digitalWrite(motorPulPin[motor], HIGH);
//  }else{
//      digitalWrite(motorPulPin[motor], LOW);

//  }
// }
// }


void setup() {
  // put your setup code here, to run once:
 counter ++;
 nh.initNode();
 nh.advertise(heartbeat);

  nh.advertise(ir_1);
   nh.advertise(ir_2);
  nh.advertise(ir_3);
  nh.advertise(ir_4);
   nh.advertise(ir_5);

  nh.advertise(imu_pub);
    //pinMode(throttlePin, INPUT_PULLUP);

// attachInterrupt(throttlePin, throttleUp, HIGH);
// attachInterrupt(throttlePin, throttleDown, LOW);


  Serial.begin(57600);
  Wire.begin();
  Wire.beginTransmission(imu_address);
  Wire.write(0x6B); //PWR_MGMT_1 register
  Wire.write(0); //wakey wakey MPU-6050
  Wire.endTransmission(true);
//servo1.attach(7);
    nh.spinOnce();


                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
}

void loop() {   
  IR1.data = analogRead(A0);
  IR2.data = analogRead(A1);
  IR3.data = analogRead(A2);
  IR4.data = analogRead(A3);
  IR5.data = analogRead(A4);
  if(Serial.available() > 0){
  //  int tmp = Serial.read();
//analogWrite(8, tmp);
Serial.println(Serial.read());

  }
  //Serial.println("tmp");

  //throttle = digitalRead(throttlePin);
 //aileron = analogRead(aileronPin);


  //Serial.print("throttle: "); Serial.println(throttle);
   // Serial.print("aileron: "); Serial.println(aileron);
//counter ++;
//if (counter >= 100){
   // heart.data ++;

 // heartbeat.publish( &heart);

//counter = 0;
//}
ir_1.publish( &IR1);
 ir_2.publish( &IR1);
 //ir_3.publish( &throttle);
 //ir_4.publish( &aileron);
// ir_5.publish( &IR1);






delay(10);

//   Wire.beginTransmission(imu_address);
//   Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
//  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
//   Wire.requestFrom(imu_address, 7*2, true); // request a total of 7*2=14 registers
  
  nh.spinOnce();

//"Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  // acc_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  // acc_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  // acc_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  // temperature.data = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  // gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  // gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  // gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  
  // print out data
  // Serial.print("aX = "); Serial.print(convert_int16_to_str(acc_x));

  // Serial.print(" | aY = "); Serial.print(convert_int16_to_str(acc_y));
  // Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(acc_z));
  // // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  // //Serial.print(" | tmp = "); Serial.print(temperature/340.00+36.53);
  // Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x));
  // Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y));
  // Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z));
  //  Serial.println();
  //Serial.println(IR1.data);
  
//packing up the accelerometer data in ROS msgs

  // imu.twist.linear.x = acc_x;
  //  imu.twist.linear.x = acc_y;
  //   imu.twist.linear.x = acc_z;
  //    imu.twist.angular.x = gyro_x;
  //    imu.twist.angular.y = gyro_y;
  //    imu.twist.angular.z = gyro_z;
  imu.linear.x = acc_x;
   imu.linear.x = acc_y;
    imu.linear.x = acc_z;
     imu.angular.x = gyro_x;
     imu.angular.y = gyro_y;
     imu.angular.z = gyro_z;


imu_pub.publish( &imu);



  // delay
  delay(10);
  //counter +1;
//servo1.write(map(acc_x, -25000, 25000, 20, 150));


  nh.spinOnce();
}

// void throttleUp(){
// throttlestart = micros();


// }

// void throttleDown(){
//   throttle = micros() - throttlestart;


// }











//if (Serial.available() > 0){
//
//  Serial.print(char(Serial.read()));
//  
//  if(Serial.available() == 0){
//    Serial.println("");
//    Serial.println("STRING READ SUCCESSFULL");
//  }
//  
//}
//
//if(Serial2.available() > 0){
//  Serial.println(Serial2.read());
//   if(Serial2.available() == 0){
//    Serial.println("");
//    Serial.println("STRING READ SUCCESSFULL");
//  }
//}





