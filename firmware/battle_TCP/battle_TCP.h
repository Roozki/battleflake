//#include "GY521.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <AccelStepper.h>

const char* ssid = "battle_network";
const char* pass = "guest123";
AsyncClient client;


int mode, drive1PWR, drive2PWR, hammerPWR;
int pin1 = 0;
int pin2 = 1;

int beatTEMP = millis();
int beatINTERVAL = 1000; //ms
int posTEMP = millis();
int posINTERVAL = 100; //ms


int counter;

