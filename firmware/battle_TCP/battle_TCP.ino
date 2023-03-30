//author: rowan zawadzki

#include "battle_TCP.h"
//#include "std/string.h"

void IRAM_ATTR encoder_L_isr() {
  int A_val = digitalRead(MOT_ENC_A_PIN_L);
  int B_val = digitalRead(MOT_ENC_B_PIN_L);

  if (A_val == ENC_L_A_prev && B_val == ENC_L_B_prev) {
    return;
  }

  if (A_val != B_val) {
    ENC_L_A_prev = A_val;
    ENC_L_B_prev = B_val;
  } else {
    if (A_val == ENC_L_A_prev) {
      encoder_L_position++;
    } else {
      encoder_L_position--;
    }
  }
}

void IRAM_ATTR encoder_R_isr() {
  int A_val = digitalRead(MOT_ENC_A_PIN_R);
  int B_val = digitalRead(MOT_ENC_B_PIN_R);

  if (A_val == ENC_R_A_prev && B_val == ENC_R_B_prev) {
    return;
  }

  if (A_val != B_val) {
    ENC_R_A_prev = A_val;
    ENC_R_B_prev = B_val;
  } else {
    if (A_val == ENC_R_A_prev) {
      encoder_R_position++;
    } else {
      encoder_R_position--;
    }
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(HAMMER_PIN, OUTPUT);
  hammer.attach(HAMMER_PIN, 500, 2500);
  hammer.setPeriodHertz(100);  // standard 50 hz servo

  //encoder pin modes
  pinMode(MOT_ENC_A_PIN_L, INPUT_PULLUP);
  pinMode(MOT_ENC_B_PIN_L, INPUT_PULLUP);
  pinMode(MOT_ENC_A_PIN_R, INPUT_PULLUP);
  pinMode(MOT_ENC_B_PIN_R, INPUT_PULLUP);

  //motor pin modes
  pinMode(MOT_PIN_L_1, OUTPUT);
  pinMode(MOT_PIN_L_2, OUTPUT);
  pinMode(MOT_PIN_R_PWM, OUTPUT);
  pinMode(MOT_PIN_R_1, OUTPUT);
  pinMode(MOT_PIN_R_2, OUTPUT);
  pinMode(MOT_PIN_R_PWM, OUTPUT);

  //encoder ISRs
  attachInterrupt(digitalPinToInterrupt(MOT_ENC_A_PIN_L), encoder_L_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOT_ENC_B_PIN_L), encoder_L_isr, CHANGE);

  attachInterrupt(digitalPinToInterrupt(MOT_ENC_A_PIN_R), encoder_R_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOT_ENC_B_PIN_R), encoder_R_isr, CHANGE);


  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }


  // Connect to the server
  client.onConnect([](void* arg, AsyncClient* c) {
    Serial.println("Connected to server");
    //Send data to the server
    c->write("I'M ROBOT_1, HELLO!");
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

   String sendBuffer_str = "R1(" + String(pwr1) + ",a" + String(pwr2) + ",b" + String(hammerUs) + ",c" + String(encoder_L_position) + ",d" + String(encoder_R_position) + ",e)\n";
   char sendBuffer[sendBuffer_str.length() + 1];
   sendBuffer_str.toCharArray(sendBuffer, sendBuffer_str.length() + 1);
   c->write(sendBuffer);
    //Serial.write((uint8_t*)data, len);
  });
}

void loop() {

  //Serial.println(encoder_L_position);
  //Serial.println(digitalRead(MOT_ENC_B_PIN_L));
  // Check for any errors or disconnections

  if (!connect_flag) {
    Serial.println("Lost connection to server");
    client.connect("192.168.1.100", 9000);

    //delay(1000);
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


  if (pwr1 > 0) {
    digitalWrite(MOT_PIN_L_1, HIGH);
    digitalWrite(MOT_PIN_L_2, LOW);
  } else {
    digitalWrite(MOT_PIN_L_1, LOW);
    digitalWrite(MOT_PIN_L_2, HIGH);
  }
  analogWrite(MOT_PIN_L_PWM, abs(pwr1 * 4));

  if (pwr2 > 0) {
    digitalWrite(MOT_PIN_R_1, HIGH);
    digitalWrite(MOT_PIN_R_2, LOW);
  } else {
    digitalWrite(MOT_PIN_R_1, LOW);
    digitalWrite(MOT_PIN_R_2, HIGH);
  }
  analogWrite(MOT_PIN_R_PWM, abs(pwr2 * 4));
}