

#include "battle_firmware.h"


void setup() {
  // put your setup code here, to run once:
 counter ++;
 nh.initNode();
 nh.advertise(heartbeat);
 nh.advertise(feedback_pub);
 nh.subscribe(CMDsub);
    nh.spinOnce();

pinMode(pin1, OUTPUT);
pinMode(pin2, OUTPUT);
fakeCMD.drive1PWR = 70;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
}

void loop() {   
counter ++;
if (millis() - beatTEMP > beatINTERVAL){
    heart.data ++;

  heartbeat.publish( &heart);
  feedback_pub.publish(&fakeCMD);
 
beatTEMP = millis();
counter = 0;
}


// if(x == 1){

//     digitalWrite(pin1, HIGH);
//     digitalWrite(pin2, LOW);
 

// }else if(b == 1){
//   digitalWrite(pin1, LOW);
//   digitalWrite(pin2, HIGH);

// } else{
//   digitalWrite(pin1, LOW);
//   digitalWrite(pin2, LOW);


// }

delay(10);

  nh.spinOnce();


}

