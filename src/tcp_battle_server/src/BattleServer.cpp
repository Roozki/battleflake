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


  int sockfd, newsockfd;
  socklen_t clilen;
  char buffer[256];
  struct sockaddr_in serv_addr, cli_addr;
  int n; 
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
   ROS_ERROR("ERROR on binding");
      }

  listen(sockfd,5);

  clilen = sizeof(cli_addr);

   newsockfd = accept(sockfd, 
                 (struct sockaddr *) &cli_addr, &clilen);
     if (newsockfd < 0) 
          ROS_ERROR("ERROR on accept");

     ROS_INFO("server: got connection from %s port %d\n",
            inet_ntoa(cli_addr.sin_addr), ntohs(cli_addr.sin_port));


     // This send() function sends the 13 bytes of the string to the new socket
     send(newsockfd, "Hello, world!\n", 13, 0);

     bzero(buffer,256);

     n = read(newsockfd,buffer,255);
     if (n < 0) ROS_ERROR("ERROR reading from socket");
     ROS_INFO("Here is the message: %s\n",buffer);

     close(newsockfd);
     close(sockfd);
}

void BattleServer::JoyCallBack(const sensor_msgs::Joy::ConstPtr& msg) {
  //  ROS_INFO("Received joy message");
    float rstickx = msg->axes[3];
    if (rstickx < 0.1 && rstickx > -0.1){
        rstickx = 0.0;
    }
    float rtrigger = msg->axes[5];
    float PWR = mapFloat(rtrigger, 1.0, -1.0, -50.0, 50.0);

    int PWR1 = PWR - (rstickx * 100) - 1;
    int PWR2 = PWR + (rstickx * 100) - 1;
    
  if(PWR1 > 100){
    PWR1 = 100;
  } 
  if(PWR1 < -100){
    PWR1 = -100;
  }
  if(PWR2 > 100){
    PWR2 = 100;
  } 
  if(PWR2 < -100){
    PWR2 = -100;
  }   
   
    cmd.drive1PWR = PWR1;
    cmd.drive2PWR = PWR2;

   //ROS_INFO("%i", cmd.mode[S1]);
   sendCmds();
    ros::Rate loop_rate(100);
    loop_rate.sleep();
    ros::spinOnce();

}

 void BattleServer::sendCmds(){

    

   // cmd_pub.publish(cmd);

 }

float BattleServer::mapFloat(float input, float fromMin, float fromMax, float toMin, float toMax){
    float m = (toMax - toMin) / (fromMax - fromMin); //slope, should be a constant?
    float output = fromMin + (m*(input - fromMin));

    return output;

    //time complexity: O(n)
}
 