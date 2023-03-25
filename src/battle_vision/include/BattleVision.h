/*
* author: Rowan Zawadzki
*/

#ifndef SAMPLE_PACKAGE_MYNODE_H
#define SAMPLE_PACKAGE_MYNODE_H

// STD Includes
#include <iostream>
#include <chrono>
#include <unordered_set>

// ROS Includes
//#include <std_msgs/String.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

// Image Conversion
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <sensor_msgs/image_encodings.h>

//bb includes
#include<bb_msgs/bbVision2point.h>
#include<bb_msgs/battleCmd.h>

//definitions
#define ROBOT_ID 0
#define ENEMY_ID 1 2 3 4
#define ROBOT_LONG_SCALE 4
#define ROBOT_LAT_SCALE 3

using namespace cv;
class BattleVision {
public:

    BattleVision(int argc, char **argv, std::string node_name);

     float fps = 30.0;
    void clickCallback(int event, int x, int y, int flags){
         if  ( event == EVENT_LBUTTONDOWN )
     {
        //  cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
               
       // Battpt.x = x;
       // BattleVision::pt.y = y;
       BattleVision::processClick(x, y);
     }
     else if  ( event == EVENT_RBUTTONDOWN )
     {
          //cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if  ( event == EVENT_MBUTTONDOWN )
     {
          //cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if ( event == EVENT_MOUSEMOVE )
     {
        
          //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;

     }
    }

   static void clickCallbackHandler(int event, int x, int y, int flags, void* userdata){
    BattleVision* self = static_cast<BattleVision*>(userdata);
    self->clickCallback(event, x, y, flags);

   }


private:
    void frameCallback(const sensor_msgs::Image::ConstPtr& msg);
     bool robotTracked = false;
     cv::Point2f m1;
     cv::Point2f m2;
     cv::Point2f click;

     void flashWarning(std::string msg, int x, int y, double size, int thick);


    std::vector<int> processMarkers(const cv::Mat& image);

    cv::Mat rosToMat(const sensor_msgs::Image::ConstPtr& image);

    image_transport::Subscriber shutter;
    ros::Publisher cmd_pubber;
    image_transport::Publisher bounder;
    ros::Publisher point_pub;

    void processClick(int x, int y);
    void sendCmd();


     cv::Mat outputImage;

    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    bool draw_markers = true;
    int camera        = 1;

    //opencv text params
    int font1 = cv::FONT_HERSHEY_SIMPLEX;
    int font2 = cv::FONT_HERSHEY_DUPLEX;

    double fontScale = 1.0;

    int thickness = 2;
    int lineType = cv::LINE_AA; //anti-aliased line
     cv::Point2f robot_locked_point = cv::Point2f(70, 140);
     cv::Point2f angle_to_go_point = cv::Point2f(50, 250);

     cv::Scalar textColour = cv::Scalar(100, 0, 0); //bgr

     //time stuff
     int frameTimer = 0;
     //int blink_interval_ms = 500; // Blink interval in milliseconds

     int blink_interval = 5; //blink_interval_ms/1000/fps;
};
#endif //SAMPLE_PACKAGE_MYNODE_H
