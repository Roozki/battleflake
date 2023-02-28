#include <BattleServer.h>
/*
author: rowan zawadzki

*/
BattleServer::BattleServer(int argc, char **argv, std::string node_name) {
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
    topic_to_subscribe_to, queue_size, &BattleServer::JoyCallBack, this);

    // Setup Publisher(s)
    // std::string topic = private_nh.resolveName("robot_1_CMD");
    // queue_size        = 1;
    // cmd_pub = private_nh.advertise<bb_msgs::battleCmd>(topic, queue_size);


 
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if(sockfd < 0){
    ROS_ERROR("error opening socket");
  }
  bzero((char *) &serv_addr, sizeof(serv_addr));

  serv_addr.sin_family = AF_INET; //sets tcp instead of udp i think

  serv_addr.sin_addr.s_addr = INADDR_ANY; //auto IP

  serv_addr.sin_port = htons(PORT1);

  if (bind(sockfd, (struct sockaddr *) &serv_addr,
    sizeof(serv_addr)) < 0){
   ROS_ERROR("ERROR on binding, killall might help?");
      }

  listen(sockfd,5);

  clilen = sizeof(cli_addr);

   newsockfd = accept(sockfd, 
    (struct sockaddr *) &cli_addr, &clilen);
     if (newsockfd < 0) 
          ROS_ERROR("ERROR on accept");

     ROS_INFO("server: got connection from %s port %d\n",
            inet_ntoa(cli_addr.sin_addr), ntohs(cli_addr.sin_port));


   

    //  n = read(newsockfd,buffer,255);
    //  if (n < 0) ROS_ERROR("ERROR reading from socket");
    //  ROS_INFO("Here is the message: %s\n",buffer);

    //  close(newsockfd);
    //  close(sockfd);
}

void BattleServer::JoyCallBack(const sensor_msgs::Joy::ConstPtr& msg) {
  //  ROS_INFO("Received joy message");
    float rstickx = msg->axes[3];
    float rsticky = msg->axes[4];
      if (rsticky < 0.1 && rsticky > -0.1){ //better to handle in joy_node run params
        rsticky = 0.0;
    }
    if (rstickx < 0.1 && rstickx > -0.1){
        rstickx = 0.0;
    }
    float rtrigger = msg->axes[5];
    float ltrigger = msg->axes[6];

    float PWR = mapFloat(rtrigger, 1.0, -1.0, 0.0, 100.0);
    float REVERSE = mapFloat(ltrigger, 1.0, -1.0, 0.0, 100.0);



    int PWR1yL = PWR - REVERSE;
    int PWR1zR = rstickx * 100;
    
  if(PWR1yL > 100){
    PWR1yL = 100;
  } 
  if(PWR1yL < -100){
    PWR1yL = -100;
  }
  if(PWR1zR > 100){
    PWR1zR = 100;
  } 
  if(PWR1zR < -100){
    PWR1zR = -100;
  }   
   

   //ROS_INFO("%i", cmd.mode[S1]);
   sendCmds(PWR1yL, PWR1zR);
    ros::Rate loop_rate(100); //send rate in hz
    loop_rate.sleep();
    ros::spinOnce();
}

 void BattleServer::sendCmds(int robot1_Ly, int robot1_Rz){

 //int Ly_length = std::strlen(std::to_string(robot1_Ly));
 //int Rz_length = std::strlen(std::to_string(robot1_Rz));

  // char Ly[Ly_length] = std::to_string(robot1_Ly);
  // char Rz[Rz_length] = std::to_string(robot1_Rz);
    
    // int outmsgSize = //Ly_length + Rz_length;

   const char* outmsg = "hey\n"; //new char[outmsgSize + 1];
     int outmsgSize = std::strlen(outmsg);//Ly_length + Rz_length;

    // This send function sends the correct number bytes of the string to the socket
     
     send(newsockfd, outmsg, outmsgSize, 0);

     bzero(buffer,256);

   // cmd_pub.publish(cmd);
   //delete[] outmsg;

 }

float BattleServer::mapFloat(float input, float fromMin, float fromMax, float toMin, float toMax){
    float m = (toMax - toMin) / (fromMax - fromMin); //slope, should be a constant?
    float output = fromMin + (m*(input - fromMin));

    return output;

    //time complexity: O(n)
}
 