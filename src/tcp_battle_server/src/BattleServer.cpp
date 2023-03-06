#include <BattleServer.h>
/*
author: rowan zawadzki

*/
BattleServer::BattleServer(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    setup();
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


    int status, valread;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
  // if(sockfd < 0){
  //   ROS_ERROR("error opening socket");
  // }

    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        ROS_ERROR("\n Socket creation error \n");
            }
  //bzero((char *) &serv_addr, sizeof(serv_addr));

  serv_addr.sin_family = AF_INET; //sets tcp instead of udp i think

  //serv_addr.sin_addr.s_addr = inet_addr("192.168.0.100");//INADDR_ANY; //auto IP

  serv_addr.sin_port = htons(PORT1);



//server stuff
 if (inet_pton(AF_INET, "192.168.0.100", &serv_addr.sin_addr)
        <= 0) {
        ROS_INFO(
            "\nInvalid address/ Address not supported \n");
        
    }

  if ((connect(sockfd, (struct sockaddr*)&serv_addr,
                   sizeof(serv_addr)))
        < 0) {
        ROS_INFO("\nConnection Failed \n");
        
    }
   char* home = "Home";

 //send(sockfd, home, strlen(home), 0);
    //ROS_INFO("Hello message sent\n");
   // valread = read(sockfd, buffer, 1024);
    //ROS_INFO("%s\n", buffer);
  
    // closing the connected socket
    //close(sockfd);

  // if (bind(sockfd, (struct sockaddr *) &serv_addr,
  //   sizeof(serv_addr)) < 0){
  //  ROS_ERROR("ERROR on binding, killall -9/-15 might help?");
  //     }

  //listen(sockfd,5);

  // clilen = sizeof(cli_addr);

  //  newsockfd = accept(sockfd, 
  //   (struct sockaddr *) &cli_addr, &clilen);
  //    if (newsockfd < 0) 
  //         ROS_ERROR("ERROR on accept");

  //    ROS_INFO("server: got connection from %s port %d\n",
  //           inet_ntoa(cli_addr.sin_addr), ntohs(cli_addr.sin_port));


   

      

    //  close(newsockfd);
    //  close(sockfd);
}
  void BattleServer::setup() {
     system(
     "gnome-terminal --tab -- bash -c 'rosrun joy joy_node _deadzone:=0.3 _autorepeat_rate:=0 _coalesce_interval:=0.01'");

     ROS_INFO(
     "ALLCONTROLLER INITIATED, CURRENTLY PARSING FOR XBOX360");
}


void BattleServer::JoyCallBack(const sensor_msgs::Joy::ConstPtr& msg) {//this callback is way too long i think
  //  ROS_INFO("Received joy message");
  //linear speed
  int lspeed = 1;
  int rspeed = 1;
  //axes
 const float lstickx = msg->axes[0] * lspeed; 
 const float lsticky = msg->axes[1] * lspeed;
 const float ltrig = msg->axes[2];
 const float rstickx = msg->axes[3] * lspeed;
 const float rsticky = msg->axes[4] * rspeed;
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
  switch (A - B){
    case(1):
      cartacc = cartacc + 0.1;
      break;
    case(-1):
      cartacc -= 0.1;
      break;
    case(0):
      //do nothing
      break;
  }
//int mode = A;
  
  //MecaCmds(lstickx, lsticky, rsticky, 0.0, 0.0, rstickx, cartacc);

 // float axisbalanceR = mapFloat(rtig, 1.0, -1.0, );//dependentAxis(rtrig, 0.0, mode);
 // float axisbalanceL = //dependentAxis(ltrig, 0.0, mode);




  float lx = lstickx;
  float ly = lsticky;// + rtrig;
  float lz = rsticky;// * ltrig;

  float rx = 0.0;//rstickx;  
  float ry = 0.0; //lstickx;// * rtrig;
  float rz = lstickx;// * ltrig;



int activate = X;
int home = Y;
int reseterror = LB;


  MecaCmds(lx, ly, lz, rx, ry, rz, cartacc, activate, home, reseterror);


    // if (rsticky < 0.1 && rsticky > -0.1){ //better to handle deadzone in joy_node run params
    //     rsticky = 0.0;
    // }
    // if (rstickx < 0.1 && rstickx > -0.1){
    //     rstickx = 0.0;
    // }
   

  //   float PWR = mapFloat(rtrig, 1.0, -1.0, 0.0, 100.0);
  //   float REVERSE = mapFloat(ltrig, 1.0, -1.0, 0.0, 100.0);



  //   int PWR1yL = PWR - REVERSE;
  //   int PWR1zR = rstickx * 100;
    
  // if(PWR1yL > 100){
  //   PWR1yL = 100;
  // } 
  // if(PWR1yL < -100){
  //   PWR1yL = -100;
  // }
  // if(PWR1zR > 100){
  //   PWR1zR = 100;
  // } 
  // if(PWR1zR < -100){
  //   PWR1zR = -100;
  // }   
   

   //ROS_INFO("%i", cmd.mode[S1]);
   //sendCmds(PWR1yL, PWR1zR);
    // ros::Rate loop_rate(10); //send rate in hz, slows down sending cmds even if joy node is rate set
    // loop_rate.sleep();
    // ros::spinOnce();
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

void BattleServer::sendCmds(int robot1_Ly, int robot1_Rz){


 int Ly_length = std::strlen(std::to_string(robot1_Ly).c_str());
 int Rz_length = std::strlen(std::to_string(robot1_Rz).c_str());

  // char Ly[Ly_length] = std::to_string(robot1_Ly);
  // char Rz[Rz_length] = std::to_string(robot1_Rz);
    
    // int outmsgSize = //Ly_length + Rz_length;
    //std::string command = "yoyo";
   //const char* 

   const int bufferSize = Ly_length + Rz_length + 12; //12 is size of surrounding characters
   char buffer[bufferSize];
  std::sprintf(buffer, "moveR1(%d, %d)\n", robot1_Ly, robot1_Rz);
   //outmsg << command[0]; //new char[outmsgSize + 1];

     //int outmsgSize = std::strlen(outmsg);//Ly_length + Rz_length;

    // This send function sends the correct number bytes of the string to the socket
     
     send(newsockfd, buffer, bufferSize, 0);

     //bzero(buffer,256);

   // cmd_pub.publish(cmd);
   //delete[] outmsg;

 }



//not for capstone, part of other project
void BattleServer::MecaCmds(float lx, float ly, float lz, float rx, float ry, float rz, float cartacc, int activate, int home, int error){

int valread;
if(tempacc != cartacc){
  int cartacc_length = std::strlen(std::to_string(cartacc).c_str());

  tempacc = cartacc;
  cartacc = 5;
  int cartbufferSize = cartacc_length + 14; 
  char cartbuffer[cartbufferSize];

  std::sprintf(cartbuffer, "SetCartAcc(%f)\n", cartacc);
  send(sockfd, cartbuffer, cartbufferSize, 0);
  ROS_INFO("Cartacc message sent\n");
    //valread = read(sockfd, buffer, 1024);
    //ROS_INFO("%s\n", buffer);

} else if(activate != tempActivate){
  //int _length = std::strlen(std::to_string(cartacc).c_str());

  tempActivate = activate;
  int accbufferSize = 15; 

  char accbuffer[accbufferSize];
  std::sprintf(accbuffer, "ActivateRobot\n");
  send(sockfd, accbuffer, accbufferSize, 0);
  ROS_INFO("activate message sent\n");
  //sleep_for();
   // valread = read(sockfd, buffer, 1023);
    //ROS_INFO("%s\n", buffer);
}else if(error != tempError){
  //int _length = std::strlen(std::to_string(cartacc).c_str());

tempError = error;
  int errbufferSize = 12; 

  char errbuffer[errbufferSize];
  std::sprintf(errbuffer, "ResetError\n");
  send(sockfd, errbuffer, errbufferSize, 0);
  //delay(100);
  ROS_INFO("reseterror message sent\n");
   // valread = read(sockfd, buffer, 1023);
    //ROS_INFO("%s\n", buffer);
}else if(home != tempHome){
  //int _length = std::strlen(std::to_string(cartacc).c_str());

  tempHome = home;
  int hmbufferSize = 6; 

  char hmbuffer[hmbufferSize];
  std::sprintf(hmbuffer, "Home\n");
  send(sockfd, hmbuffer, hmbufferSize, 0);
  ROS_INFO("home message sent\n");
   // delay(100);

//    valread = read(sockfd, buffer, 1023);
  //  ROS_INFO("%s\n", buffer);

}else{

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

   


   const int CMDbufferSize = 2048;// Lx_length + Ly_length + Lz_length + Rx_length + Ry_length + Rz_length + 27; //27 is the # of bytes of surrounding characters
   char CMDbuffer[CMDbufferSize];
   std::sprintf(CMDbuffer, "MoveLinVelTrf(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)\0", lx, ly, lz, rx, ry, rz);
   //outmsg << command[0]; //new char[outmsgSize + 1];

     //int outmsgSize = std::strlen(outmsg);//Ly_length + Rz_length;

    // This send function sends the correct number bytes of the string to the socket
     
     send(sockfd, CMDbuffer, CMDbufferSize, 0);
     ROS_INFO("control message sent\n");
    //  char bufferree[1023];
    //  int n = read(sockfd,bufferree,1023);
    //   if (n < 0) ROS_ERROR("ERROR reading from socket");
    //   ROS_INFO("Here is the message: %s\n",bufferree);
     bzero(CMDbuffer,CMDbufferSize);
     //bzero(bufferree,1024);

    //valread = read(sockfd, buffer, 1024);
    //ROS_INFO("%s\n", buffer);
}
  //delay(100);



   // cmd_pub.publish(cmd);
   //delete[] outmsg;

 }


float BattleServer::mapFloat(float input, float fromMin, float fromMax, float toMin, float toMax){
    float m = (toMax - toMin) / (fromMax - fromMin); //slope, should be a constant?
    float output = fromMin + (m*(input - fromMin));

    return output;

    //time complexity: O(n)
}
 