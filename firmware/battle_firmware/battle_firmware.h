


//#include "GY521.h"
#include <Encoder.h>

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>
#include <bb_msgs/battleCmd.h>
#include <AccelStepper.h>

//ros stuff

ros::NodeHandle nh;

std_msgs::Int16 heart;
std_msgs::Int16 IR1;
bb_msgs::battleCmd fakeCMD; 

int mode, drive1PWR, drive2PWR, hammerPWR;
int pin1 = 0;
int pin2 = 1;
//geometry_msgs::TwistStamped imu;
geometry_msgs::Twist info;
ros::Publisher heartbeat("heartbeat", &heart);


ros::Publisher feedback_pub("feedback", &fakeCMD);

int beatTEMP = millis();
int beatINTERVAL = 1000; //ms
int posTEMP = millis();
int posINTERVAL = 100; //ms


int counter;

void cmdCallback( const bb_msgs::battleCmd& msg){
  fakeCMD.mode = 1;
  feedback_pub.publish(&fakeCMD);
 }

ros::Subscriber<bb_msgs::battleCmd> CMDsub("battleCmd", &cmdCallback, 2);