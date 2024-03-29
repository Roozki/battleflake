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
#include <geometry_msgs/Twist.h>
//#include <keyboard_publisher/KeyEvent.h>
#include<bb_msgs/robotStatus.h>
#include<bb_msgs/networkStatus.h>
#include <cstring>

//network TCP stuff
#define _BSD_SOURCE
#include <stdio.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h> 
#include <arpa/inet.h>
#include <unistd.h>
#define PORT1 9000//8080 //9000 for battle firmwares

//modess
#define joy_mode 0
#define vision_mode 1


class BattleServer {
public:
    BattleServer(int argc, char **argv, std::string node_name);


private: 


    float mapFloat(float input, float fromMin, float fromMax, float toMin, float toMax);

    void JoyCallback(const sensor_msgs::Joy::ConstPtr& msg);
     void VisionCMDCallback(const geometry_msgs::Twist::ConstPtr& msg);


    void sendCmds(int robot1_Ly, int robot1_Rz, int hammerPOS);
    void MecaCmds(float lx, float ly, float lz, float rx, float ry, float rz, float cartacc, int activate, int home);
    void setup(int mode);
    void readRobot();
    float dependentAxis(float MasterAxis, float SlaveAxis, int mode); //i think the naming convention 'master' and 'slave' should change as its potentially harmful language, but it be what it be for now


    //void 

    int sockfd, newsockfd;
    socklen_t clilen;
    char buffer[256];
    char readBuffer[256];
    struct sockaddr_in serv_addr, cli_addr;
    int n; 
    float cartacc = 0.0;

    //temp variables
    float tempacc = 0;
    int tempActivate = 0;
    int tempHome = 0;




    ros::Subscriber joy_sub;
    ros::Subscriber vision_sub;

    ros::Publisher network_status_pub;
    ros::Publisher robot_status_pub;

    sensor_msgs::Joy joy;
};
#endif //SAMPLE_PACKAGE_MYNODE_H
