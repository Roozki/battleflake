#include <BattleVision.h>
/*
author: rowan zawadzki

*/




using namespace std;
 using namespace cv;
BattleVision::BattleVision(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Obtains character from the parameter server (or launch file), sets '!' as default
    //std::string parameter_name    = "character";
    //std::string default_character = "!";
//    SB_getParam(private_nh, parameter_name, suffix, default_character);

    // Setup Subscriber(s)
    // std::string topic_to_subscribe_to = "joy";
    // int queue_size                    = 1;
    // joy_sub                    = nh.subscribe(
    // topic_to_subscribe_to, queue_size, &MicroComm::JoyCallBack, this);

    // // Setup Publisher(s)
    // std::string topic = private_nh.resolveName("robot_1_CMD");
    // queue_size        = 1;
    // cmd_pub = private_nh.advertise<bb_msgs::battleCmd>(topic, queue_size);

    cv::Mat img = cv::imread("testPic.JPG");
       //if fail to read the image
     if ( img.empty() ) 
     { 
        ROS_ERROR("failed loading image");

     }
       //Create a window
     cv::namedWindow("My Window", 1);

     //set the callback function for any mouse event
     cv::setMouseCallback("My Window", BattleVision::CallBackk, NULL);

     //show the image
     cv::imshow("My Window", img);

     // Wait until user press some key
     cv::waitKey(0);

     //return 0;
}
 void BattleVision::CallBackk(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == cv::EVENT_LBUTTONDOWN )
     {
        //  cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        ROS_INFO("Left Button clicked at: %d, %d", x, y);
     }
     else if  ( event == cv::EVENT_RBUTTONDOWN )
     {
          //cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if  ( event == cv::EVENT_MBUTTONDOWN )
     {
          //cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if ( event == cv::EVENT_MOUSEMOVE )
     {
        
          //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;

     }
}

// float MicroComm::mapFloat(float input, float fromMin, float fromMax, float toMin, float toMax){
//     float m = (toMax - toMin) / (fromMax - fromMin); //slope, should be a constant?
//     float output = fromMin + (m*(input - fromMin));

//     return output;

//     //time complexity: O(n)
// }
 