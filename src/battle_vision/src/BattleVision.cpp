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
     ros::Rate loop_rate(3);
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

//     cv::Mat img = cv::imread("/home/rowan/battleflake/src/battle_vision/src/testPic.png");
//        //if fail to read the image
//      if ( img.empty() ) 
//      { 
//         ROS_ERROR("failed loading image");

//      }
//        //Create a window
     VideoCapture cap(0);
     if (!cap.isOpened())
    {
        ROS_ERROR("Failed to open camera");
    }
     cv::namedWindow("Battle Eye", 0);
     cv::setMouseCallback("Battle Eye", &BattleVision::clickCallbackHandler, this);

      Mat frame;
     Size targetResolution(800, 600);

    

    while (ros::ok())
    {
//        cap.grab(frame);
 //cv::setMouseCallback("Battle Eye", &BattleVision::clickCallbackHandler, this);

cap >> frame;
   // resize(frame, frame, targetResolution);
  //  Mat img = frame;//imread(frame);
     //    if (frame.empty())
     //    {
     //        ROS_ERROR("Failed to read frame from camera");
     //        break;
     //    }
        ros::spinOnce();

        imshow("Battle Eye", frame);
            cv::waitKey(1);
            loop_rate.sleep();

            ROS_INFO("wee");

    }

       // Resize the image to a specific resolution
    


     //set the callback function for any mouse event
       //  cv::imshow("My Window", frame);


     //show the image at a rate of 10hz
    // while(ros::ok()){

     // Wait until user press some key
    // cv::waitKey(0);
     //loop_rate.sleep();

     //}

     //return 0;
}

    
//      void BattleVision::clickCallbackHandler(int event, int x, int y, int flags, void *clickCallback){
//           BattleVision* ptr = static_cast<BattleVision*>(clickCallback);
//           ptr->BattleVision::clickCallback(event, x, y);
//       // static_cast<BattleVision*>(this)clickCallback(event, x, y);
//     }



// float MicroComm::mapFloat(float input, float fromMin, float fromMax, float toMin, float toMax){
//     float m = (toMax - toMin) / (fromMax - fromMin); //slope, should be a constant?
//     float output = fromMin + (m*(input - fromMin));

//     return output;

//     //time complexity: O(n)
// }
 