//#include "GY521.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <AccelStepper.h>
#include <sstream>
#include <ESP32Servo.h>
//left drive motor
#define MOT_PIN_L_1 18
#define MOT_PIN_L_2 5
#define MOT_PIN_L_PWM 1

#define HAMMER_PIN 17

//right drive motor
#define MOT_PIN_R_1 23
#define MOT_PIN_R_2 22
#define MOT_PIN_R_PWM 19

Servo hammer;



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

