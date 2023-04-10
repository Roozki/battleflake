/*
* author: Rowan Zawadzki
*/

#ifndef SAMPLE_PACKAGE_MYNODE_H
#define SAMPLE_PACKAGE_MYNODE_H

// STD Includes
#include <iostream>
#include <chrono>
#include <thread>
#include <unordered_set>
#include <random>

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
//#include <opencv2/core/cuda.hpp>
//#include <opencv2/core/cuda/arithm.hpp>

// Image Conversion
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <sensor_msgs/image_encodings.h>

//bb includes
#include<bb_msgs/robotStatus.h>
#include<bb_msgs/networkStatus.h>

//PID controller
// #define Kp 1.0
// #define Ki 0.1
// #define kd 0.1

//definitions
#define ROBOT_ID 3
#define ENEMY_ID 4
#define ROBOT_LONG_SCALE 4
#define ROBOT_LAT_SCALE 3
#define WEAPON_SCALE 9

#define MAX_PWM_ANG 240
#define MAX_PWM_LIN 230

#define LIN_ANG_OFFSET_TRADEOFF 30
#define MAX_OFFSET 140.0



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
       //BattleVision::processClick(x, y);
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
       BattleVision::processClick(x, y);

     }
    }

   static void clickCallbackHandler(int event, int x, int y, int flags, void* userdata){
    BattleVision* self = static_cast<BattleVision*>(userdata);
    self->clickCallback(event, x, y, flags);

   }


private:
     void frameCallback(const sensor_msgs::Image::ConstPtr& msg);
     void robot_1_callBack(const bb_msgs::robotStatus::ConstPtr& msg);
     void network_callBack(const bb_msgs::networkStatus::ConstPtr& msg);
     bool areCVPointsClose(const cv::Point2f &point1, const cv::Point2f &point2, float threshold);
     float mapFloat(float input, float fromMin, float fromMax, float toMin, float toMax);

     bool robotTracked = false;
     cv::Point2f m1;
     cv::Point2f m2;
     cv::Point2f enemy_position;
     cv::Point2f hammerHitPoint;
     cv::Point2f pivot;

     //interval 0 for constant msg
     void flashWarning(std::string msg, int x, int y, double size, int thick, cv::Scalar colour, int blink_interval, int cycle,  int* frameCLK); //colour is processed as bgr
     void draw_loading_bar(cv::Mat &output, double progress);

    std::vector<int> processMarkers(const cv::Mat& image);

    cv::Mat rosToMat(const sensor_msgs::Image::ConstPtr& image);

    image_transport::Subscriber shutter;
    ros::Publisher cmd_pubber;
    image_transport::Publisher bounder;
    ros::Publisher point_pub;
    ros::Subscriber server_status;
    ros::Subscriber robot_1_status;

    void processClick(int x, int y);
    void sendCmd();
    void dramaticSetup(); //fancy startup just for fun

     std::string wee; //waa (in other words, a temp string used to print values with cv::puttext)

     cv::Mat outputImage;//(window_height, window_width, CV_8UC3, cv::Scalar(0, 0, 0));


     cv::Ptr<cv::aruco::Dictionary> dictionary;
     cv::Ptr<cv::aruco::DetectorParameters> detectorParams;

    bool draw_markers = true;
    int camera        = 1;

     const int window_width = 1920;
     const int window_height = 1080;

    //opencv text params
    int font1 = cv::FONT_HERSHEY_SIMPLEX;
    int font2 = cv::FONT_HERSHEY_DUPLEX;

    double fontScale = 1.0;

    int thickness = 2;
    int lineType = cv::LINE_AA; //anti-aliased line
     cv::Point2f robot_locked_point = cv::Point2f(70, 140);
     cv::Point2f angle_to_go_point = cv::Point2f(50, 250);
     cv::Point2f proportional_adj_point = cv::Point2f(50, 350);
     cv::Point2f intergral_adj_point = cv::Point2f(50, 400);
     cv::Point2f derrivative_adj_point = cv::Point2f(50, 450);

     cv::Scalar textColour = cv::Scalar(100, 0, 0); //bgr

     //time stuff, should be a vector or hash map
     int frameCLK_1 = 0;
     int frameCLK_2 = 0;
     int frameCLK_3 = 0;
     int frameCLK_4 = 0;
     int frameCLK_6 = 0;
     int frameCLK_7 = 0;
     int frameCLK_8 = 0;
     int frameCLK_9 = 0;
     int frameCLK_10 = 0;


     //network status
     int networkSpeed = -1;
     bool networkStatus = 0;
     //int blink_interval_ms = 500; // Blink interval in milliseconds

     //int blink_interval = 5; //blink_interval_ms/1000/fps;

     //robot feedback
          //weapon systems
          int hammer_STATUS = -1;
          int hammer_counter = 0;
          //drive systems
          int slip;
     
     //flags
     bool started = true;
     


     struct Robot
     {
          char status = '0';
          int L_speed = 0;
          int R_speed = 0;
          int L_PWM = 0;
          int R_PWM = 0;
     };
     Robot robot1;

     struct frame_clock
     {
          uint8_t clk = 0;
          uint8_t threshold = 250;
          void resetCheck(){
               if (clk > threshold){
                    clk = 0;
               }
          }
     };
     frame_clock frameCLK_5;
     geometry_msgs::Twist cmd;


     // PID angular gains

    double angKp = 2.2 * 100;
    double angKi = 0.3 * 100;
    double angKd = 1.9 * 100;
    //kd = 1.77 was a good one, kp = 0.6-7

    double angsetpoint = 0.0; // Desired trajectory angle
    double angerror = 0.0;
    double angprevious_error = 0.0;
    double angintegral = 0.0;
    double angderivative = 0.0;
    double output = 0.0;
    int offset = 140; //depends on battery level
     int angoffset;
     // PID linear gains
    double linKp = 0.9 * 1;
    double linKi = 0 * 1;
    double linKd = 0.2 * 1;

    int LIN_THRESHOLD_ANGLE = 5;

    double linsetpoint = 0.0; // Desired distance from point
    double linerror = 0.0;
    double linprevious_error = 0.0;
    double linintegral = 0.0;
    double linderivative = 0.0;
    double linOutput = 0.0;
    //int offset = 150; //depends on battery level
    
    bool angresetflag = false;
    bool tempangresetflag = false;
    bool linresetflag = false;
    bool templinresetflag = false;
    int linoffset; //depends on battery level


     

};
#endif //SAMPLE_PACKAGE_MYNODE_H
