#include "stdint.h"
//#include "GY521.h"
#include <WiFi.h>
#include <Arduino.h>
#include <AsyncTCP.h>
#include <AccelStepper.h>
#include <sstream>
#include <ESP32Servo.h>
//left drive motor
#define MOT_PIN_L_1 5
#define MOT_PIN_L_2 18
#define MOT_PIN_L_PWM 17

#define HAMMER_PIN 16 

//right drive motor
#define MOT_PIN_R_1 23
#define MOT_PIN_R_2 22
#define MOT_PIN_R_PWM 19

//encoders for wheel odometry
#define MOT_ENC_A_PIN_L 32
#define MOT_ENC_B_PIN_L 33
#define MOT_ENC_A_PIN_R 26
#define MOT_ENC_B_PIN_R 27

#define ENC_STEPS_PER_SHAFT_ROTATION 1856 //64 per revolution, with 30:1 gear ratio

//PID values for velocity
#define kp 1.5
#define ki 0.1
#define kd 0.15

//speed stuff
#define SPEED_INTERVAL_MS 50
#define METRIC_SPEED_CONSTANT 1

long encoder_L_prev = 0;
long encoder_R_prev = 0;

long Lspeed = 0;
long Rspeed = 0;

volatile long encoder_L_position = 0;
volatile bool ENC_L_A_prev = false;
volatile bool ENC_L_B_prev = false;

volatile long encoder_R_position = 0;
volatile bool ENC_R_A_prev = false;
volatile bool ENC_R_B_prev = false;


//Interupt Service Routines
void IRAM_ATTR encoder_L_isr(); //IRAM_ATTR to isolate memory in the more rapidly accessable IRAM
void IRAM_ATTR encoder_R_isr();


Servo hammer;

int pwrL;
int pwrR;
int hammerPos; //hammer microsecond pulse width for PWM

void sendCmd();

void velocityCMD(int vel_lin, int vel_ang);
void CMD();
void attack(int hammerCMD);

bool connect_flag = false;


const char* ssid = "battle_network";
const char* pass = "guest123";
AsyncClient client;


int mode, drive1PWR, drive2PWR;

//int pwr1 = 0;
//int pwr2 = 0;
//time stuff
int speed_prev_time = 0;

// int beatTEMP = millis();
// int beatINTERVAL = 1000; //ms
// int posTEMP = millis();
// int posINTERVAL = 100; //ms


int counter;

//steppers

//Stepper();


// class Stepper{
//   public: Stepper(uint8_t PUL_pin, uint8_t DIR_pin, uint8_t gear_reduction, uint8_t steps_per_rev) : PUL_PIN(PUL_pin), DIR_PIN(DIR_pin), GEAR_RED(gear_reduction), STEPS_PER_REV(steps_per_rev){
//     pinMode(PUL_PIN, OUTPUT);
//     pinMode(DIR_PIN, OUTPUT);
//   }

//   void setSpeed(int rpm){
//     step_delay_us = rpm*GEAR_RED*STEPS_PER_REV/60/1000000; //revolutions -> steps, minutes -> us
//   }

//   void moveStepsRel(int steps){
//       dir = steps > 0;

//   }

//   // void moveVel(float scale_vel){
//   //   bool dir = scale_vel > 0;
//   //   step_delay_us
//   //   digitalWrite(DIR_PIN, dir);
//   //   long time = micros();
//   //   if(time > prev_time + vel){
//   //     steps_to_take ++;
//   //     prev_time = time;
//   //   }
//   // }
//   void getSpeed(){

//   }

//   void run(){ //must be called at least as fast as step_delay_us
//     digitalWrite(DIR_PIN, dir);
//     time = micros();
//     if(time > prev_time + step_delay_us){
//       pul_state = !pul_state;
//       prev_time = time;
//     }
//     digitalWrite(PUL_PIN, pul_state);

//   }

//   private:
//     uint8_t PUL_PIN;
//     uint8_t DIR_PIN;
//     int prev_time;
//     uint16_t GEAR_RED;
//     uint16_t STEPS_PER_REV;
//     int step_delay_us = 100;
//     long steps_to_take;
//     long steps_taken;
//     bool dir;
//     bool pul_state;
//     long time;

    

// };

