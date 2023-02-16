#include <MicroComm.h>
/*
author: rowan zawadzki

*/
MicroComm::MicroComm(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Obtains character from the parameter server (or launch file), sets '!' as default
    //std::string parameter_name    = "character";
    //std::string default_character = "!";
//    SB_getParam(private_nh, parameter_name, suffix, default_character);

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "joy";
    int queue_size                    = 1;
    joy_sub                    = nh.subscribe(
    topic_to_subscribe_to, queue_size, &MicroComm::JoyCallBack, this);

    // Setup Publisher(s)
    std::string topic = private_nh.resolveName("robot_1_CMD");
    queue_size        = 1;
    cmd_pub = private_nh.advertise<bb_msgs::battleCmd>(topic, queue_size);
}

void MicroComm::JoyCallBack(const sensor_msgs::Joy::ConstPtr& msg) {
    ROS_INFO("Received message");
float inputPWR = msg->axes[5];
    float PWR = mapFloat(inputPWR, -1.0, 1.0, -128.0, 128.0);

    //int PWR1 = PWR1 + (msg->axes[3] * 50);
    //int PWR2 =
   // cmd.drive1PWR = PWR + ((msg->axes[3] * 50) );

   //ROS_INFO("%i", cmd.mode[1]);
   sendCmd();
}

 void MicroComm::sendCmd(){

    

     cmd_pub.publish(cmd);

 }

float MicroComm::mapFloat(float input, float fromMin, float fromMax, float toMin, float toMax){
    float m = (toMax - toMin) / (fromMax - fromMin); //slope, should be a constant?
    float output = fromMin + (m*(input - fromMin));

    return output;
}
 