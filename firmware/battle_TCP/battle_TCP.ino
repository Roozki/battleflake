#include "battle_TCP.h"
//#include "std/string.h"



/*
TODO:
Get values for servo working
left right triggers for hammer



*/

int pos;

void setup() {
  Serial.begin(9600);

  pinMode(HAMMER_PIN, OUTPUT);
  hammer.attach(HAMMER_PIN, 500, 2500);
    hammer.setPeriodHertz(100);  // standard 50 hz servo


  pinMode(MOT_PIN_L_1, OUTPUT);
  pinMode(MOT_PIN_L_2, OUTPUT);
  //pinMode(MOT_PIN_L_PWM, OUTPUT);
  pinMode(MOT_PIN_R_1, OUTPUT);
  pinMode(MOT_PIN_R_2, OUTPUT);
  pinMode(MOT_PIN_R_PWM, OUTPUT);

  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }


  // Connect to the server
  client.onConnect([](void* arg, AsyncClient* c) {
    Serial.println("Connected to server");
    //Send data to the server
    c->write("Hello, server!");
  });

  client.onError([](void* arg, AsyncClient* c, int8_t error) {
    Serial.println("Connection error");
    Serial.println(error);
  });

  client.onDisconnect([](void* arg, AsyncClient* c) {
    Serial.println("Disconnected from server");
  });

  client.onData([](void* arg, AsyncClient* c, void* data, size_t len) {
    String sendBuffer_str = "R1(" + String(pwr1); + ",a" + String(pwr2) + ",b" + String(hammerUs) + ",c)\n";
    char sendBuffer[sendBuffer_str.length() + 1];
    sendBuffer_str.toCharArray(sendBuffer, sizeof(sendBuffer));
    c->write(sendBuffer);
    //Serial.print("Received data: ");
    // char* temp = data;
    //    Serial.println(temp);
    const char* bus = ((char*)data);

    // bus = ;
    String dat = bus;
    int indexLY = dat.indexOf("a");
    int linX = (dat.substring(7, indexLY)).toInt();
    int indexRZ = dat.indexOf("b");
    int angZ = (dat.substring(indexLY + 1, indexRZ)).toInt();
    int indexHAM = dat.indexOf("c");
    int hammerPOS = (dat.substring(indexRZ + 1, indexHAM).toInt());

    //Serial.println(angZ);
    //Serial.println(hammerPOS);
    //Serial.println(linX);
    CMD(linX, angZ, hammerPOS);
    //Serial.write((uint8_t*)data, len);
  });
}

void loop() {

  // Check for any errors or disconnections
  if (!connect_flag) {
    Serial.println("Lost connection to server");
    client.connect("192.168.1.100", 9000);
    delay(1000);
    connect_flag = true;
  }
}


void CMD(int lin, int ang, int hammerPOS) {

  pwr1 = lin + ang;
  pwr2 = lin - ang;
  hammerUs = map(hammerPOS, 0, 201, 5, 90);
  //delay(10);
  hammer.write(hammerUs);
  //Serial.println(hammerUs);


  if (pwr1 > 0){
    digitalWrite(MOT_PIN_L_1, HIGH);
    digitalWrite(MOT_PIN_L_2, LOW);
  }else{
    digitalWrite(MOT_PIN_L_1, LOW);
    digitalWrite(MOT_PIN_L_2, HIGH);
  }
  analogWrite(MOT_PIN_L_PWM, abs(pwr1*4));

  if (pwr2 > 0) {
      digitalWrite(MOT_PIN_R_1, HIGH);
      digitalWrite(MOT_PIN_R_2, LOW);
    }else{
      digitalWrite(MOT_PIN_R_1, LOW);
      digitalWrite(MOT_PIN_R_2, HIGH);
  }
   analogWrite(MOT_PIN_R_PWM, abs(pwr2*4));
}