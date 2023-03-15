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
//#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/opencv.hpp>

// Image Conversion
#include <cv_bridge/cv_bridge.h>
#include <image_transport/subscriber.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
class BattleVision {
public:

    BattleVision(int argc, char **argv, std::string node_name);

     float fps = 20.0;
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
     //Point pt(1,1);

   
   // float mapFloat(float input, float fromMin, float fromMax, float toMin, float toMax);

   // void JoyCallBack(const sensor_msgs::Joy::ConstPtr& msg);
  //  void sendCmd();
   


    // ros::Subscriber joy_sub;
    // ros::Publisher cmd_pub;
    // sensor_msgs::Joy joy;
    // bb_msgs::battleCmd cmd;
};
#endif //SAMPLE_PACKAGE_MYNODE_H
