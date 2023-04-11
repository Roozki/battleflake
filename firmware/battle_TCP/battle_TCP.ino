//author: rowan zawadzki

#include "battle_TCP.h"
//#include "std/string.h"

// void IRAM_ATTR encoder_L_isr() {
//   int A_val = digitalRead(MOT_ENC_A_PIN_L);
//   int B_val = digitalRead(MOT_ENC_B_PIN_L);

//   if (A_val == ENC_L_A_prev && B_val == ENC_L_B_prev) {
//     return;
//   }

//   if (A_val != B_val) {
//     ENC_L_A_prev = A_val;
//     ENC_L_B_prev = B_val;
//   } else {
//     if (A_val == ENC_L_A_prev) {
//       encoder_L_position++;
//     } else {
//       encoder_L_position--;
//     }
//   }
// }

// void IRAM_ATTR encoder_R_isr() {
//   int A_val = digitalRead(MOT_ENC_A_PIN_R);
//   int B_val = digitalRead(MOT_ENC_B_PIN_R);

//   if (A_val == ENC_R_A_prev && B_val == ENC_R_B_prev) {
//     return;
//   }

//   if (A_val != B_val) {
//     ENC_R_A_prev = A_val;
//     ENC_R_B_prev = B_val;
//   } else {
//     if (A_val == ENC_R_A_prev) {
//       encoder_R_position++;
//     } else {
//       encoder_R_position--;
//     }
//   }
// }


void setup() {
  Serial.begin(9600);
  //LED system
   //xTaskCreatePinnedToCore(ledTask, "ledTask", 150000, NULL, 1, &ledTaskHandle, 1);
  
  FastLED.addLeds<WS2812B, DATA_LEDS_B, GRB>(ledsB, NUM_LEDS_B);
  FastLED.addLeds<WS2812B, DATA_LEDS_BL, GRB>(ledsBL, NUM_LEDS_BL);
  FastLED.addLeds<WS2812B, DATA_LEDS_BR, GRB>(ledsBR, NUM_LEDS_BR);
  FastLED.addLeds<WS2812B, DATA_LEDS_L, GRB>(ledsL, NUM_LEDS_L);
  FastLED.addLeds<WS2812B, DATA_LEDS_R, GRB>(ledsR, NUM_LEDS_R);
  FastLED.addLeds<WS2812B, DATA_LEDS_F, GRB>(ledsF, NUM_LEDS_F);
  FastLED.setBrightness(150); // Adjust the brightness, value between 0 and 255
 



  pinMode(HAMMER_PIN, OUTPUT);
  hammer.setPeriodHertz(200);  // standard 50 hz servo
  hammer.attach(HAMMER_PIN, 500, 2500);
 
  //encoder pin modes
  // pinMode(MOT_ENC_A_PIN_L, INPUT_PULLUP);
  // pinMode(MOT_ENC_B_PIN_L, INPUT_PULLUP);
  // pinMode(MOT_ENC_A_PIN_R, INPUT_PULLUP);
  // pinMode(MOT_ENC_B_PIN_R, INPUT_PULLUP);


  //motor pin modes
  pinMode(MOT_PIN_L_1, OUTPUT);
  pinMode(MOT_PIN_L_2, OUTPUT);
  pinMode(MOT_PIN_R_PWM, OUTPUT);
  pinMode(MOT_PIN_R_1, OUTPUT);
  pinMode(MOT_PIN_R_2, OUTPUT);
  pinMode(MOT_PIN_R_PWM, OUTPUT);

  //hammer relay
  pinMode(HAMMER_PWR_PIN, OUTPUT);
  

  //encoder ISRs
  // attachInterrupt(digitalPinToInterrupt(MOT_ENC_A_PIN_L), encoder_L_isr, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(MOT_ENC_B_PIN_L), encoder_L_isr, CHANGE);

  // attachInterrupt(digitalPinToInterrupt(MOT_ENC_A_PIN_R), encoder_R_isr, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(MOT_ENC_B_PIN_R), encoder_R_isr, CHANGE);


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
    int hammerCommand = (dat.substring(indexRZ + 1, indexHAM).toInt());
  

    //Serial.println(angZ);
    //Serial.println(hammerPOS);
    //Serial.println(linX);
   velocityCMD(linX, angZ);
   attack(hammerCommand);

   String sendBuffer_str = "R1(" + String(pwrL) + "a" + String(pwrR) + "b" + String(hammerPos) + "c" + String(encoder_L_position) + "d" + String(encoder_R_position) + "e)\n";
   char sendBuffer[sendBuffer_str.length() + 1];
   sendBuffer_str.toCharArray(sendBuffer, sendBuffer_str.length() + 1);
   c->write(sendBuffer);
   checktLights();
    //Serial.write((uint8_t*)data, len);
  });
}

void checktLights(){
if (nonBlockingDelay(previousMillis, 50)) {
      updateRainbow(ledsB, NUM_LEDS_PER_STRIP, 10);
      updateRainbow(ledsBL, NUM_LEDS_PER_STRIP, 10);
      updateRainbow(ledsBR, NUM_LEDS_PER_STRIP, 10);
      updateRainbow(ledsL, NUM_LEDS_PER_STRIP, 10);
      updateRainbow(ledsR, NUM_LEDS_PER_STRIP, 10);
      updateRainbow(ledsF, NUM_LEDS_PER_STRIP, 10);
    
    FastLED.show();
  }
}
void loop() {
  

  if(hammerPos == 30){
    hammerOffCount ++;
    if(hammerOffCount > 4000 ){
       digitalWrite(HAMMER_PWR_PIN, LOW);
       hammerOffCount = 0;
    }
  }
  //Serial.println(encoder_L_position);
  //Serial.println(digitalRead(MOT_ENC_B_PIN_L));
  // Check for any errors or disconnections

  if (!connect_flag) {
    Serial.println("Lost connection to server");
    client.connect("192.168.1.100", 9000);

    //delay(1000);
    connect_flag = true;
  }
//poll speed
  // if(millis() - speed_prev_time > SPEED_INTERVAL_MS){

  //   Lspeed = (encoder_L_position - encoder_L_prev)*METRIC_SPEED_CONSTANT;
  //   Rspeed = (encoder_R_position - encoder_R_prev)*METRIC_SPEED_CONSTANT;
  //   speed_prev_time = millis();
  //   encoder_L_prev = encoder_L_position;
  //   encoder_R_prev = encoder_R_position;
  // }

  // Serial.print(Lspeed);
  // Serial.print("  ");
  // Serial.println(Rspeed);


}

void velocityCMD(int vel_lin, int vel_ang) {

  if(vel_lin == 0 && vel_ang == 0){ //quickly stops, no need to calculate anything
    pwrR = 0;
    pwrL = 0;
    CMD();
// #define ki 0.1
// #define kd 0.15
    return;
  }

  pwrR = vel_lin + vel_ang;
  pwrL = vel_lin - vel_ang;

  // if(vel_lin < Rspeed){
  //   pwrR-=5;

  // }
  // if(vel_lin < Lspeed){
  //   pwrL-=5;
  // }
  //  if(vel_lin > Rspeed){
  //   pwrR+= 5;

  // }
  // if(vel_lin > Lspeed){
  //   pwrL+=5;
  // }

  CMD();
 return;
}

void attack(int hammerCMD){
 //hammerUs = map(hammerCMD, 0, 201, 5, 90);
  hammerPos = 70;
if(hammerCMD > 30){
  hammerPos = 70;
  digitalWrite(HAMMER_PWR_PIN, HIGH);
}
if(hammerCMD <= 30){
  hammerPos = 40;
  digitalWrite(HAMMER_PWR_PIN, HIGH);

}
if(hammerCMD <= 10){
  hammerPos = 30;

  
}
  //delay(10);
      hammer.write(hammerPos);

  //Serial.println(hammerPos);
}


void CMD() {
 
  //Serial.println(hammerUs);
  

  if (pwrL > 0) {
    digitalWrite(MOT_PIN_L_1, HIGH);
    digitalWrite(MOT_PIN_L_2, LOW);
  } else {
    digitalWrite(MOT_PIN_L_1, LOW);
    digitalWrite(MOT_PIN_L_2, HIGH);
  }
  analogWrite(MOT_PIN_L_PWM, abs(pwrL));

  if (pwrR > 0) {
    digitalWrite(MOT_PIN_R_1, HIGH); 
    digitalWrite(MOT_PIN_R_2, LOW);
  } else {
    digitalWrite(MOT_PIN_R_1, LOW);
    digitalWrite (MOT_PIN_R_2, HIGH);
  }
  analogWrite(MOT_PIN_R_PWM, abs(pwrR));
  return;
}
void updateRainbow(CRGB* leds, int numLeds, uint8_t phaseOffset) {
  static uint8_t initialHue = 0;
  initialHue++;
  for (int i = 0; i < numLeds; i++) {
    leds[i] = CHSV(initialHue + i * 256 / numLeds + phaseOffset, 255, 255);
  }
}


