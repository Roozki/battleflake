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

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/opencv.hpp>

// Image Conversion
#include <cv_bridge/cv_bridge.h>
#include <image_transport/subscriber.h>
#include <sensor_msgs/image_encodings.h>


class BattleVision {
public:
    BattleVision(int argc, char **argv, std::string node_name);
 

private:
    void CallBackk(int event, int x, int y, int flags, void* userdata);

   // float mapFloat(float input, float fromMin, float fromMax, float toMin, float toMax);

   // void JoyCallBack(const sensor_msgs::Joy::ConstPtr& msg);
  //  void sendCmd();
   


    // ros::Subscriber joy_sub;
    // ros::Publisher cmd_pub;
    // sensor_msgs::Joy joy;
    // bb_msgs::battleCmd cmd;
};
#endif //SAMPLE_PACKAGE_MYNODE_H
