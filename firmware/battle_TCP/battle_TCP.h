//#include "GY521.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <AccelStepper.h>
#include <sstream>
#include <ESP32Servo.h>
//left drive motor
#define MOT_PIN_L_1 18
#define MOT_PIN_L_2 5
#define MOT_PIN_L_PWM 17

#define HAMMER_PIN 2

//right drive motor
#define MOT_PIN_R_1 23
#define MOT_PIN_R_2 22
#define MOT_PIN_R_PWM 19

//encoders for wheel odometry
#define MOT_ENC_A_PIN_L 32
#define MOT_ENC_B_PIN_L 33
#define MOT_ENC_A_PIN_R 27
#define MOT_ENC_B_PIN_R 26

#define ENC_STEPS_PER_SHAFT_ROTATION 1856 //64 per revolution, with 30:1 gear ratio

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

int pwr1;
int pwr2;
int hammerUs; //hammer microsecond pulse width for PWM

void sendCmd();
void CMD(int lin, int ang, int hammerPOS);

bool connect_flag = false;


const char* ssid = "battle_network";
const char* pass = "guest123";
AsyncClient client;


int mode, drive1PWR, drive2PWR;

//int pwr1 = 0;
//int pwr2 = 0;

int beatTEMP = millis();
int beatINTERVAL = 1000; //ms
int posTEMP = millis();
int posINTERVAL = 100; //ms


int counter;

