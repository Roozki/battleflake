#include <BattleServer.h>
/*
author: rowan zawadzki

*/
BattleServer::BattleServer(int argc, char **argv, std::string node_name) {

    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    //ros::NodeHandle private_nh("~");

    setup(joy_mode);
    // Obtains character from the parameter server (or launch file), sets '!' as default
    //std::string parameter_name    = "character";
    //std::string default_character = "!";
//    SB_getParam(private_nh, parameter_name, suffix, default_character);

    // Setup Subscriber(s)
   // std::string topic_to_subscribe_to = "joy";
    //int queue_size                    = 1;
    joy_sub = 
    nh.subscribe("joy", 1, &BattleServer::JoyCallback, this);

    vision_sub =
    nh.subscribe("robot1_cmd_vel", 1, &BattleServer::VisionCMDCallback, this);
    

    // Setup Publisher(s)
    std::string status_topic = "network_STATUS";
    network_status_pub = nh.advertise<bb_msgs::networkStatus>(status_topic, 1);

    status_topic = "robot_1_status";
    robot_status_pub = nh.advertise<bb_msgs::robotStatus>(status_topic, 1);

 //int status, client_fd;
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if(sockfd < 0){
    ROS_ERROR("error opening socket");
  }

    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        ROS_ERROR("\n Socket creation error \n");
            }
  //bzero((char *) &serv_addr, sizeof(serv_addr));

  serv_addr.sin_family = AF_INET; //sets tcp instead of udp i think

  serv_addr.sin_addr.s_addr = inet_addr("192.168.1.100");//INADDR_ANY; //auto IP, also //

  serv_addr.sin_port = htons(PORT1);



//server stuff
//  if (inet_pton(AF_INET, "192.168.0.100", &serv_addr.sin_addr)
//         <= 0) {
//         ROS_INFO(
//             "\nInvalid address/ Address not supported \n");
        
//     }

  // if ((connect(sockfd, (struct sockaddr*)&serv_addr,
  //                  sizeof(serv_addr)))
  //       < 0) {
  //       ROS_INFO("\nConnection Failed \n");
        
  //   }
bb_msgs::networkStatus net_msg;


  if (bind(sockfd, (struct sockaddr *) &serv_addr,
    sizeof(serv_addr)) < 0){
   ROS_ERROR("ERROR on binding, killall -9/-15 might help?");
    net_msg.status = false;
      }else{
   ROS_INFO("Socket was free, bind successful");
    net_msg.status = true;
      }
        listen(sockfd,5);


  clilen = sizeof(serv_addr);

   newsockfd = accept(sockfd, 
    (struct sockaddr *) &serv_addr, &clilen);
     if (newsockfd < 0) 
          ROS_ERROR("ERROR on accept");

     ROS_INFO("server: got connection from %s port %d\n",
            inet_ntoa(serv_addr.sin_addr), ntohs(serv_addr.sin_port));


  network_status_pub.publish(net_msg);

     n = read(newsockfd,buffer,255);
     if (n < 0) ROS_ERROR("ERROR reading from socket");
     ROS_INFO("Client has connected: %s\n",buffer);

    //  close(newsockfd);
    //  close(sockfd);
}

void BattleServer::setup(int mode) {
    switch (mode){
      case joy_mode:
         system(
        "gnome-terminal --tab -- bash -c 'rosrun joy joy_node _deadzone:=0.05 _autorepeat_rate:=20 _coalesce_interval:=0.05'");
        ROS_WARN(
        "JOYSTICK MODE DETECTED\n");
        ROS_INFO(
        "ALLCONTROLLER INITIATED, CURRENTLY PARSING FOR XBOX360");
        break;

      case vision_mode:
        ROS_WARN("VISION MODE DETECTED, JOYSTICK WILL NOT INITIATE");
        break;
      default:
          ROS_ERROR("No setup mode set, something went wrong");

        break;
    }

    ROS_INFO("Battle Main Control Node has started, attempting network initiation.");

     
}


void BattleServer::JoyCallback(const sensor_msgs::Joy::ConstPtr& msg) {//this callback is way too long i think
  //  ROS_INFO("Received joy message");
  //linear speed
  int lspeed = 1;
  int rspeed = 1;
  //axes
 const float lstickx = msg->axes[0];// * lspeed; 
 const float lsticky = msg->axes[1];// * lspeed;
 const float ltrig = msg->axes[2];
 const float rstickx = msg->axes[3];// * lspeed;
 const float rsticky = msg->axes[4];// * rspeed;
 const float rtrig = msg->axes[5];
 const float dpadx = msg->axes[6];
 const float dpady = msg->axes[7];
  //buttons
 const int A = msg->buttons[0];
 const int B = msg->buttons[1];
 const int X = msg->buttons[2];
 const int Y = msg->buttons[3];
 const int LB = msg->buttons[4];
 const int RB = msg->buttons[5];
 const int back = msg->buttons[6];
 const int start = msg->buttons[7];
 const int thebigone = msg->buttons[8];
 const int lstickB = msg->buttons[9];
 const int rstickB = msg->buttons[9];
  // switch (A - B){
  //   case(1):
  //     cartacc = cartacc + 0.1;
  //     break;
  //   case(-1):
  //     cartacc -= 0.1;
  //     break;
  //   case(0):
  //     //do nothing
  //     break;
  // }
//int mode = A;


 // float axisbalanceR = mapFloat(rtig, 1.0, -1.0, );//dependentAxis(rtrig, 0.0, mode);
 // float axisbalanceL = //dependentAxis(ltrig, 0.0, mode);


    // if (rsticky < 0.1 && rsticky > -0.1){ //better to handle deadzone in joy_node run params
    //     rsticky = 0.0;
    // }
    // if (rstickx < 0.1 && rstisckx > -0.1){
    //     rstickx = 0.0;
    // }
   

    //float PWR = mapFloat(rsticky, 1.0, -1.0, 0.0, 100.0);
   // float REVERSE = mapFloat(ltrig, 1.0, -1.0, 0.0, 100.0);

    float maphammer = mapFloat(rtrig, 1.0, -1.0, 0.0, 50.0);
    int hammer = maphammer;


    int PWR1yL = (rsticky + lsticky) * 300;
    int PWR1zR = (rstickx + lstickx) * 300;
    
  if(PWR1yL > 300){
    PWR1yL = 300;
  } 
  if(PWR1yL < -300){
    PWR1yL = -300;
  }
  if(PWR1zR > 300){
    PWR1zR = 300;
  } 
  if(PWR1zR < -300){
    PWR1zR = -300;
  }   
   

   //ROS_INFO("%i", cmd.mode[S1]);
   sendCmds(PWR1yL, PWR1zR, hammer);
    //ros::Rate loop_rate(5); //send rate in hz, slows down sending cmds even if joy node is rate set

     //loop_rate.sleep();
     ros::spinOnce();
}

float BattleServer::dependentAxis(float MasterAxis, float SlaveAxis, int mode){ //i think the naming convention 'master' and 'slave' should change as its potentially harmful language, but it be what it be for now
  float factor;
  switch (mode)
  {
  case 1:
  //factor = 
  default:
  ROS_INFO("tried to balance axis without mode set");
    break;
  }



  return factor;
}

void BattleServer::sendCmds(int robot1_Ly, int robot1_Rz, int hammer){


 int Ly_length = std::strlen(std::to_string(robot1_Ly).c_str());
 int Rz_length = std::strlen(std::to_string(robot1_Rz).c_str());
 int h_length  = std::strlen(std::to_string(hammer).c_str());

   const int bufferSize = Ly_length + Rz_length + h_length + 14; //14 bytes is size of surrounding characters, including spaces
   char buffer[bufferSize];
  std::sprintf(buffer, "moveR1(%d,a%d,b%d,c)\n", robot1_Ly, robot1_Rz, hammer);
  
    // This send function sends the correct number bytes of the string to the socket
     
     send(newsockfd, buffer, bufferSize, 0);

     bzero(buffer, bufferSize);
     readRobot();

 }

void BattleServer::readRobot(){

  int datasize;
  datasize = read(newsockfd,readBuffer,256);
  if (datasize < 0){
    ROS_ERROR("ERROR reading from socket"); 
    return;
  
  }

  std::string dataIn = std::string(readBuffer);

  bb_msgs::robotStatus robot_msg;
  if(dataIn.substr(0, 2) == "R1"){ //possibly make a switch case
    //ROS_WARN("Robot 1 Online");
    robot_msg.status = '1';
  }

  int Rspeed_index = dataIn.find('(');
  int Lspeed_index = dataIn.find('a');
  int hammer_index = dataIn.find('b');

   if (Rspeed_index != std::string::npos && Lspeed_index != std::string::npos && hammer_index != std::string::npos) {
    int rspeed_length = Lspeed_index - Rspeed_index - 1;
    int lspeed_length = hammer_index - Lspeed_index - 1;

    int rspeed = std::stoi(dataIn.substr(Rspeed_index + 1, rspeed_length));
    int lspeed = std::stoi(dataIn.substr(Lspeed_index + 1, lspeed_length));
 robot_msg.R_speed = rspeed;
  robot_msg.L_speed = lspeed;
    
  }else{
  robot_msg.R_speed = -1;
  robot_msg.L_speed = -1;
  }
  
 

  
  robot_status_pub.publish(robot_msg);

  //ROS_WARN("RSPEED : %i\n", rspeed);
  //ROS_WARN("LSPEED : %i\n", lspeed);

    
  ROS_INFO("Client says: %s\n",readBuffer);

 }

 void BattleServer::VisionCMDCallback(const geometry_msgs::Twist::ConstPtr& msg){ //should be custom msg
   

  int x = msg->linear.x;
  int z = msg->angular.z;
  int hammer = msg->angular.y;
   
  sendCmds(x, z, hammer);


 }



//not for capstone, part of other project
void BattleServer::MecaCmds(float lx, float ly, float lz, float rx, float ry, float rz, float cartacc, int activate, int home){

if(tempacc != cartacc){
  int cartacc_length = std::strlen(std::to_string(cartacc).c_str());

  tempacc = cartacc;
  int accbufferSize = cartacc_length + 14; 

  char accbuffer[accbufferSize];
  std::sprintf(accbuffer, "SetCartAcc(%f)\n", cartacc);
  send(newsockfd, accbuffer, accbufferSize, 0);

}

if(activate != tempActivate){
  //int _length = std::strlen(std::to_string(cartacc).c_str());

  tempActivate = activate;
  int accbufferSize = 15; 

  char accbuffer[accbufferSize];
  std::sprintf(accbuffer, "ActivateRobot\n");
  send(newsockfd, accbuffer, accbufferSize, 0);
}
if(home != tempHome){
  //int _length = std::strlen(std::to_string(cartacc).c_str());

  tempHome = home;
  int accbufferSize = 6; 

  char accbuffer[accbufferSize];
  std::sprintf(accbuffer, "Home\n");
  send(newsockfd, accbuffer, accbufferSize, 0);

}

// this can probably be more efficient
 int Lx_length = std::strlen(std::to_string(lx).c_str()); 
 int Ly_length = std::strlen(std::to_string(ly).c_str());
 int Lz_length = std::strlen(std::to_string(lz).c_str());
 int Rx_length = std::strlen(std::to_string(rx).c_str());
 int Ry_length = std::strlen(std::to_string(ry).c_str());
 int Rz_length = std::strlen(std::to_string(rz).c_str());



  // char Ly[Ly_length] = std::to_string(robot1_Ly);
  // char Rz[Rz_length] = std::to_string(robot1_Rz);
    
    // int outmsgSize = //Ly_length + Rz_length;
    //std::string command = "yoyo";
   //const char* 

   
  

   const int bufferSize = Lx_length + Ly_length + Lz_length + Rx_length + Ry_length + Rz_length + 27; //27 is the # of bytes of surrounding characters
   char buffer[bufferSize];
  std::sprintf(buffer, "MoveLinVelTrf(%f, %f, %f, %f, %f, %f)\n", lx, ly, lz, rx, ry, rz);
   //outmsg << command[0]; //new char[outmsgSize + 1];

     //int outmsgSize = std::strlen(outmsg);//Ly_length + Rz_length;

    // This send function sends the correct number bytes of the string to the socket
     
     send(newsockfd, buffer, bufferSize, 0);

     //bzero(buffer,256);

   // cmd_pub.publish(cmd);
   //delete[] outmsg;

 }


float BattleServer::mapFloat(float input, float fromMin, float fromMax, float toMin, float toMax){
    float m = (toMax - toMin) / (fromMax - fromMin); //slope, should be a constant?
    float output = fromMin + (m*(input - fromMin));

    return output;

    //time complexity: O(n)
}
 