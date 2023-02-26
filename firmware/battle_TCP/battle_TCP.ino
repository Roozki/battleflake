#include "battle_TCP.h"


// void setup() {
//   // put your setup code here, to run once:
//  counter ++;

// pinMode(pin1, OUTPUT);
// pinMode(pin2, OUTPUT);
// fakeCMD.drive1PWR = 70;

// }

// void loop() {
// counter ++;
// if (millis() - beatTEMP > beatINTERVAL){

// beatTEMP = millis();
// counter = 0;
// }

// // if(x == 1){

// //     digitalWrite(pin1, HIGH);
// //     digitalWrite(pin2, LOW);


// // }else if(b == 1){
// //   digitalWrite(pin1, LOW);
// //   digitalWrite(pin2, HIGH);

// // } else{
// //   digitalWrite(pin1, LOW);
// //   digitalWrite(pin2, LOW);


// // }

// delay(10);



// }
void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");

  // Connect to the server
  client.onConnect([](void* arg, AsyncClient* c) {
    Serial.println("Connected to server");
    // Send data to the server
    // c->write("Hello, server!");
  });

  client.onError([](void* arg, AsyncClient* c, int8_t error) {
    Serial.println("Connection error");
    Serial.println(error);
  });

  client.onDisconnect([](void* arg, AsyncClient* c) {
    Serial.println("Disconnected from server");
  });

  client.connect("192.168.1.100", 9000);
}

void loop() {
  // Process any incoming data from the server
  while (client.connected()) {

    Serial.println("Still Connected");
    client.write("helloo");
    delay(1000);

    // String data = client.read();
  }
  // Check for any errors or disconnections
  if (!client.connected()) {
    Serial.println("Lost connection to server");
  }
}