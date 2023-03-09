#include "battle_TCP.h"



void setup() {
  Serial.begin(9600);

 pinMode(pin1, OUTPUT);
 pinMode(pin2, OUTPUT);

  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  } 


  // Connect to the server
  client.onConnect([](void* arg, AsyncClient* c) {
    Serial.println("Connected to server");
    // Send data to the server
    // c->w;hrite("Hello, server!");
  });

  client.onError([](void* arg, AsyncClient* c, int8_t error) {
    Serial.println("Connection error");
    Serial.println(error);
  });

  client.onDisconnect([](void* arg, AsyncClient* c) {
    Serial.println("Disconnected from server");
  });

    client.onData([](void* arg, AsyncClient* c, void* data, size_t len) {

    //Serial.print("Received data: ");
   // char* temp = data;
//    Serial.println(temp);
    String dat = "";

     dat = ((String *) data)->c_str();
    int indexTEMP = dat.lastIndexOf(", ");
    pwr1 = (dat.substring(7, indexTEMP)).toInt();
    int indexTEMP2 = dat.indexOf(")");
    pwr2 = (dat.substring(indexTEMP, indexTEMP2)).toInt();

   // CMD((uint8_t*)data, len);
   //Serial.println(dat);
    Serial.write(pwr1);
     //Serial.print(" we  ");

    //Serial.println(pwr2);


    Serial.write((uint8_t*)data, len);
  });
  client.connect("192.168.1.100", 9000);


}

void loop() {
  // Process any incoming data from the server
    //client.write("helloo");

  if (client.connected()) {

   // Serial.println("Still Connected");
    client.write("helloo");
    delay(100);


    
     //String data = client.read();
  }
  // Check for any errors or disconnections
  if (!client.connected()) {
    Serial.println("Lost connection to server");
  client.connect("192.168.1.100", 9000);
      delay(100);

  }
      delay(100);

}


void CMD(int pwr1, int pwr2){


}