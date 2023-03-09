#include <ESP32Servo.h>
#define MOT_PIN_1 18
#define MOT_PIN_2 5
#define MOT_PIN_PWM 17


Servo mot();
void setup() {
  // put your setup code here, to run once:
  pinMode(MOT_PIN_1, OUTPUT);
  pinMode(MOT_PIN_2, OUTPUT);
  pinMode(MOT_PIN_PWM, OUTPUT);

}

void loop() {
  digitalWrite(MOT_PIN_1, HIGH);
  digitalWrite(MOT_PIN_2, LOW);
  // put your main code here, to run repeatedly:
delay(10);
for( int i = 50; i < 350; i += 10){
analogWrite(MOT_PIN_PWM, i);
delay(10);

}
for( int i = 350; i > 49; i -= 10){
analogWrite(MOT_PIN_PWM, i);
delay(10);

}
 digitalWrite(MOT_PIN_1, LOW);
 digitalWrite(MOT_PIN_2, HIGH);
 for( int i = 50; i < 350; i += 10){
analogWrite(MOT_PIN_PWM, i);
delay(10);

}

for( int i = 350; i > 49; i -= 10){
analogWrite(MOT_PIN_PWM, i);
delay(10);

}

}
