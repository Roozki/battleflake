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

    client.onData([](void* arg, AsyncClient* c, void* data, size_t len) {
    Serial.print("Received data: ");
    Serial.write((uint8_t*)data, len);
  });

  client.connect("192.168.1.100", 9000);
}

void loop() {
  // Process any incoming data from the server
  while (client.connected()) {

    Serial.println("Still Connected");
    client.write("helloo");
    delay(100);

    
     //String data = client.read();
  }
  // Check for any errors or disconnections
  if (!client.connected()) {
    Serial.println("Lost connection to server");
  }
      delay(100);

}