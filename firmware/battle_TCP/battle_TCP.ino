#include "battle_TCP.h"



void setup() {
  Serial.begin(9600);

  pinMode(MOT_PIN_L_1, OUTPUT);
  pinMode(MOT_PIN_L_2, OUTPUT);
  pinMode(MOT_PIN_L_PWM, OUTPUT);
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
    client.write("init_robot_1\n");
    //Serial.print("Received data: ");
   // char* temp = data;
//    Serial.println(temp);
    const char* bus = ((char*) data);

    // bus = ;
     String dat = bus;
    int indexTEMP = dat.lastIndexOf(", ");
   int linX = (dat.substring(7, indexTEMP)).toInt();
    int indexTEMP2 = dat.indexOf(")");
   int angZ = (dat.substring(indexTEMP + 1, indexTEMP2 + 1)).toInt();
  
   // CMD((uint8_t*)data, len);
   Serial.print(dat);
  //  Serial.write(pwr1);
     //Serial.print(" we  ");

    //Serial.write((uint8_t*)pwr2, len);
   // Serial.println(pwr1);
    //Serial.print(angZ);
    //Serial.println(linX);
    CMD(linX, angZ);
    //Serial.write((uint8_t*)data, len);
  });
  //client.connect("192.168.1.100", 9000);


}

void loop() {
  // Process any incoming data from the server
    //client.write("helloo");
   

    // // if(!connect_flag){
    //      if (client.connected()) {

    // //   
    //    connect_flag = true;
      
    //  }else if(!client.connected() && !client.connecting()){
    //    connect_flag = false;
    //  }



   // Serial.println("Still Connected");
     //String data = client.read();
     //delay(10);
  
  // Check for any errors or disconnections
  if (!connect_flag) {
    Serial.println("Lost connection to server");
  client.connect("192.168.1.100", 9000);
      delay(1000);
            connect_flag = true;


  }
      //delay(10);

}


void CMD(int lin, int ang){

  int pwr1 = lin + ang;
  int pwr2 = lin - ang;

  if (pwr1 > 0){
    digitalWrite(MOT_PIN_L_1, HIGH);
    digitalWrite(MOT_PIN_L_2, LOW);
  }else{
    digitalWrite(MOT_PIN_L_1, LOW);
    digitalWrite(MOT_PIN_L_2, HIGH);
  }
  analogWrite(MOT_PIN_L_PWM, abs(pwr1*4));

    if (pwr2 > 0){
    digitalWrite(MOT_PIN_R_1, HIGH);
    digitalWrite(MOT_PIN_R_2, LOW);
  }else{
    digitalWrite(MOT_PIN_R_1, LOW);
    digitalWrite(MOT_PIN_R_2, HIGH);
  }
  analogWrite(MOT_PIN_R_PWM, abs(pwr2*4));

}