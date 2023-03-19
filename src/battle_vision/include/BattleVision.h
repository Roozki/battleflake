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
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

// Image Conversion
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <sensor_msgs/image_encodings.h>

//bb includes
#include<bb_msgs/bbVision2point.h>


using namespace cv;
class BattleVision {
public:

    BattleVision(int argc, char **argv, std::string node_name);

     float fps = 30.0;
    void clickCallback(int event, int x, int y, int flags){
         if  ( event == EVENT_LBUTTONDOWN )
     {
        //  cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        ROS_INFO("Left Button clicked at: %d, %d", x, y);
          if(x > 500){
               fps--;
          }
          else{
               fps++;
          }
       // Battpt.x = x;
       // BattleVision::pt.y = y;
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

     
    std::vector<int> processMarkers(const cv::Mat& image);

    cv::Mat rosToMat(const sensor_msgs::Image::ConstPtr& image);

    image_transport::Subscriber shutter;
    ros::Publisher my_publisher;
    image_transport::Publisher bounder;
    ros::Publisher point_pub;


    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    bool draw_markers = true;
    int camera        = 1;
};
#endif //SAMPLE_PACKAGE_MYNODE_H
