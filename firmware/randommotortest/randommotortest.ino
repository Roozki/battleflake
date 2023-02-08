#include <Servo.h>
#define MOT_PIN_1 2
#define MOT_PIN_2 4
#define MOT_PIN_PWM 3


Servo mot();
void setup() {
  // put your setup code here, to run once:
  pinMode(MOT_PIN_1, OUTPUT);

}

void loop() {
  digitalWrite(MOT_PIN_1, HIGH);
  digitalWrite(MOT_PIN_2, LOW);
  // put your main code here, to run repeatedly:
analogWrite(MOT_PIN_PWM, 100);

//delay(100);
}
