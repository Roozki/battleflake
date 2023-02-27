/*
 * author: Rowan Zawadzki
*/

#ifndef SAMPLE_PACKAGE_MYNODE_H
#define SAMPLE_PACKAGE_MYNODE_H

// STD Includes
#include <iostream>

// ROS Includes
//#include <std_msgs/String.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <bb_msgs/battleCmd.h>

//network TCP stuff
#define _BSD_SOURCE
#include <stdio.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h> 
#include <arpa/inet.h>
#include <unistd.h>
#define PORT1 8080


class BattleServer {
public:
    BattleServer(int argc, char **argv, std::string node_name);
 

private:
    
    float mapFloat(float input, float fromMin, float fromMax, float toMin, float toMax);

    void JoyCallBack(const sensor_msgs::Joy::ConstPtr& msg);
    void sendCmds();
   


    ros::Subscriber joy_sub;
    ros::Publisher cmd_pub;
    sensor_msgs::Joy joy;
    bb_msgs::battleCmd cmd;
};
#endif //SAMPLE_PACKAGE_MYNODE_H
