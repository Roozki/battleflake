#include <BattleVision.h>
/*
author: rowan zawadzki

*/

    auto previous_time = std::chrono::high_resolution_clock::now();
    auto current_time = std::chrono::high_resolution_clock::now();

   
BattleVision::BattleVision(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Rate loop_rate(fps);
    ros::Duration sleep_duration(2.0);





    // Setup image transport
    image_transport::ImageTransport it(nh);

    // Initialize ArUco parameters
    detectorParams = cv::aruco::DetectorParameters::create();
    dictionary =  cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));
     // Adjust the ArUco parameters
    //detectorParams->adaptiveThreshConstant = 2;
    detectorParams->minMarkerPerimeterRate = 0.1;
    detectorParams->maxMarkerPerimeterRate = 0.3;
    // detectorParams->polygonalApproxAccuracyRate = 0.06;
    // detectorParams->minCornerDistanceRate = 0.05;
    // detectorParams->markerBorderBits = 1;
    // detectorParams->minDistanceToBorder = 2;
    // detectorParams->minMarkerDistanceRate = 0.05;
    // detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    // detectorParams->cornerRefinementWinSize = 5;
    // detectorParams->cornerRefinementMaxIterations = 30;
    // detectorParams->cornerRefinementMinAccuracy = 0.1;
    // detectorParams->errorCorrectionRate = 0.6;

    std::string topic_to_subscribe_to = "cam_1/color/image_raw";


    int queue_size = 5;
    shutter  = it.subscribe(
    topic_to_subscribe_to, queue_size, &BattleVision::frameCallback, this);

    std::string topic = private_nh.resolveName("identified");
    queue_size        = 10;

    robot_1_status = nh.subscribe<bb_msgs::robotStatus>("robot_1_status", 5, &BattleVision::robot_1_callBack, this);

   // point_pub = private_nh.advertise<bb_msgs::bbVision2point>("battlepoint", queue_size);

    cmd_pubber = nh.advertise<geometry_msgs::Twist>("robot1_cmd_vel", queue_size);

    // image detection publisher
    bounder = it.advertise("bounding_boxes", 5);
    cv::namedWindow("Battle Vision", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("Battle Vision", &BattleVision::clickCallbackHandler, this);
    //Size targetResolution(1920, 1080);

    ROS_INFO("initiation appears successfull.");

        // Check if your system has an NVIDIA GPU and if OpenCV is built with CUDA support
    // if (!cv::cuda::getCudaEnabledDeviceCount()) {
    //     ROS_ERROR("No GPU found or the library is compiled without CUDA support.");
    // }
    //Create a window
    VideoCapture cap;
    //iterate through cameras, make sure only one is connected
    for(int i = 0; i < 10; i++){
     cap.open(i);
     if (!cap.isOpened())
    {
        ROS_ERROR("Failed to open camera: %d", i);

    }else{
        i = 10;
    }
    }

    
    //double fps = cap.get(CAP_PROP_FPS);
    ROS_WARN("%f", fps);
    
    cv::Mat frame;
     
    
    cap.set(CAP_PROP_FPS, BattleVision::fps);

    frame_clock frameCLK_5;//up to here is used

    while (ros::ok())
    {

    cap >> frame;
  
      
            ros::spinOnce();

            cv::waitKey(1000/fps);
            frameCLK_1++;
            frameCLK_2++;  
            //frameCLK_5.clk++;
            // frameCLK_6++;
            // frameCLK_7++;
            // frameCLK_8++;
            // frameCLK_9++;
            // frameCLK_10++;

            frameCLK_5.resetCheck();
          

            loop_rate.sleep();
  if (frame.empty())
         {
             ROS_WARN("Failed to read frame from camera");
                    
            

         }else{
            processMarkers(frame);
         }

    }

}

    void BattleVision::robot_1_callBack(const bb_msgs::robotStatus::ConstPtr& msg){

        robot1.status = msg->status;
        robot1.L_speed = msg->L_speed;
        robot1.R_speed = msg->R_speed;
        return;
    }

    void BattleVision::network_callBack(const bb_msgs::networkStatus::ConstPtr& msg){
        networkStatus = msg->status;
        networkSpeed = msg->speed;
        return;
    }




void BattleVision::frameCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // ROS_INFO("Received message");
    // std::vector<int> markerIds = processMarkers(rosToMat(msg));
    // std::cout << "Markers detected:";
    // for (int markerId : markerIds) { std::cout << " " << markerId; }
    // std::cout << std::endl;
}

std::vector<int> BattleVision::processMarkers(const cv::Mat& image) {
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    // /cv::Point2f p1(0.0, 10.0);
   // cv::Point2f p2(50.0, 50.0);
   


    cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, detectorParams);

        image.copyTo(outputImage);
        if(!started){
        dramaticSetup(); //fancy startup just for fun
        started = true;
        }
        if(markerIds.size() > 0){
            std::unordered_set<int> orginizedIds(markerIds.begin(), markerIds.end());
            
            if(orginizedIds.find(ROBOT_ID) != orginizedIds.end()){
                int robot_index = std::distance(markerIds.begin(), std::find(markerIds.begin(), markerIds.end(), ROBOT_ID));   
                robotTracked = true;
                m1 = (markerCorners[robot_index][0] + markerCorners[robot_index][1])/2;
                m2 = (markerCorners[robot_index][2] + markerCorners[robot_index][3])/2;
                cv::Point2f robot_longitudinal_direction = m2 - m1;
                cv::Point2f robot_lateral_direction = markerCorners[robot_index][2] - markerCorners[robot_index][3];

                cv::Point2f front_robot = m1 + robot_longitudinal_direction*ROBOT_LONG_SCALE;
                cv::Point2f right_robot = m1 + robot_lateral_direction*ROBOT_LAT_SCALE;
                cv::Point2f left_robot = m1 - robot_lateral_direction*ROBOT_LAT_SCALE;
                hammerHitPoint = m1 + robot_longitudinal_direction*WEAPON_SCALE;
                
                //robot'render'

                cv::line(outputImage, BattleVision::m1, front_robot, cv::Scalar(0, 0, 200), 5);
                cv::line(outputImage, BattleVision::m1, right_robot, cv::Scalar(100, 100, 0), 5);
                cv::line(outputImage, BattleVision::m1, left_robot, cv::Scalar(100, 100, 0), 5);
                std::string hammer_msg = "HAMMER ";
                cv::Scalar hammer_msg_colour;

                //weapon attack point
                cv::circle(outputImage, hammerHitPoint, 25, cv::Scalar(0, 0, 200), 3);

                if(orginizedIds.find(ENEMY_ID) != orginizedIds.end()){ // make update even if robot is not locked
                    int enemy_index = std::distance(markerIds.begin(), std::find(markerIds.begin(), markerIds.end(), ENEMY_ID));   
                    enemy_position = (markerCorners[enemy_index][0] + markerCorners[enemy_index][2])/2;
                    cv::line(outputImage, BattleVision::m1, enemy_position, cv::Scalar(200, 100, 0), 4);
                    //frameCLK_5.threshold = 40;
                      //  frameCLK_5.resetCheck();
                        if(areCVPointsClose(hammerHitPoint, enemy_position, 30.0)){
                            
                            // Draw the 'X' on the hammerhit point
                            std::random_device rd;
                            std::mt19937 gen(rd());
                            int b = 0;//std::uniform_int_distribution<>(0, 255)(gen);
                            int g = 0;//std::uniform_int_distribution<>(0, 255)(gen);
                            int r = 0;//std::uniform_int_distribution<>(0, 255)(gen);
                            //if(frameCLK_5.clk > 28){

                            cv::Point2f line_length(70.0, 70.0);
                            cv::line(outputImage, cv::Point2f(enemy_position.x - line_length.x, enemy_position.y - line_length.y), cv::Point2f(enemy_position.x + line_length.x, enemy_position.y + line_length.y), cv::Scalar(b,g,r), 10);
                            cv::line(outputImage, cv::Point2f(enemy_position.x - line_length.x, enemy_position.y + line_length.y), cv::Point2f(enemy_position.x + line_length.x, enemy_position.y - line_length.y), cv::Scalar(b,g,r), 10);
                            hammer_counter ++;
                            if(cmd.angular.y == 70.0 && hammer_counter > 10){
                                hammer_counter = -50; //cooldown
                                hammer_STATUS = -1;
                                cmd.angular.y = 0;
                            }else if(hammer_counter > 10){
                                
                                hammer_counter = 0;
                                hammer_STATUS = 1;
                                cmd.angular.y = 70.0;

                            }
                            
                        }else{
                            hammer_counter = 0;
                            cmd.angular.y = 0;
                            hammer_STATUS = 0;
                        }
                    

                }else{

                            cmd.angular.y = 0;
                            hammer_STATUS = 0;
                        }

                switch (hammer_STATUS)
                {
                case 1:
                    hammer_msg += "TARGET LOCKED";
                    hammer_msg_colour = cv::Scalar(0, 0, 250);
                    break;
                case 0:
                    hammer_msg += "READY";
                    hammer_msg_colour = cv::Scalar(0, 250, 0);
                    break;
                default:
                    hammer_msg += "STANDBY";
                    hammer_msg_colour = cv::Scalar(255, 255, 0);
                    break;
                }
                flashWarning(hammer_msg, 50, 600, 1, 1.5, hammer_msg_colour, 5, 3, &frameCLK_2);


                //desired traj vector


            }else{
            robotTracked = false;
            
            }
            
        }else{
            flashWarning("NO MARKERS DETECTED", 400, 300, 3, 2, cv::Scalar(0, 100, 200), 15, 2, &frameCLK_1);
            robotTracked = false;

        }

        cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
        bounder.publish(
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", outputImage)
        .toImageMsg());


    sendCmd();

    imshow("Battle Vision", outputImage);

    

    return markerIds;
}

void BattleVision::processClick(int x, int y){
    //todo reduntant function
    ROS_INFO("Left Button clicked at: %d, %d", x, y);

        //cmd.linear.x = 0; 
        //cmd.angular.z = 0; 


    cv::Point2f tmp(x, y);
    enemy_position = tmp;
    //cv::line(outputImage, BattleVision::m1, click, cv::Scalar(200, 100, 0), 4);

    return;
}

bool BattleVision::areCVPointsClose(const cv::Point2f &point1, const cv::Point2f &point2, float threshold)
{
    float distance = cv::norm(point1 - point2);
    return distance <= threshold;
}


void BattleVision::sendCmd(){
//TODO: sophisticated PID
//algebruh
//may need to be in another node
if(robotTracked){

    std::string temp("ROBOT LOCKED");
    cv::putText(outputImage, temp, robot_locked_point, font2, 2.0, cv::Scalar(0, 200, 0), thickness, lineType, false);

cv::Point2f currTraj(m2.x - m1.x, m2.y - m1.y); //current trajectory
cv::Point2f desTraj(enemy_position.x - m1.x, enemy_position.y - m1.y); //desired traj
//cv::Point2f desTraj(hammerHitPoint.x - m1.x, hammerHitPoint.y - m1.y); //desired traj



 float dot = desTraj.dot(currTraj);
 float mag1 = norm(currTraj);
 float mag2 = norm(desTraj);

 float cos_theta = dot/(mag1 * mag2);
 float angle = fastAtan2(sqrt(1 - cos_theta*cos_theta), cos_theta);

float cross = currTraj.x * desTraj.y - currTraj.y * desTraj.x;

std::string wee("angle is: ");

if (cross > 0){
    wee += std::to_string(-angle);
    angresetflag = true;
    cv::putText(outputImage, wee, angle_to_go_point, font1, fontScale, cv::Scalar(0, 100, 0), thickness, lineType, false);
   
    cmd.angular.z = 1;

    
}else if (cross < 0){
    wee += std::to_string(angle);
    cv::putText(outputImage, wee, angle_to_go_point, font1, fontScale, cv::Scalar(110, 0, 0), thickness, lineType, false);
angresetflag = false;

    cmd.angular.z = -0.98;

}
if(tempangresetflag != angresetflag){
        integral = 0;
        tempangresetflag = angresetflag;
        ROS_ERROR("ang ang ang  RESET");

    }
if(angle > 5){

    //angular PID
    error = (setpoint - angle) / 180.0; //setpoint is always 0, dividing by 180 to normilize
    current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> dt = current_time - previous_time;
    integral += error * dt.count();
    derivative = (error - previous_error) / dt.count();
    cmd.angular.z *= (Kp * error + Ki * integral + Kd * derivative - offset);
   


    float intergral_adj = Ki*integral;
    float derrivative_adj = Kd * derivative;
    float proportional_adj = Kp * error;
     
        
    if(cmd.angular.z < -255){
        cmd.angular.z = -255;
    }
    if(cmd.angular.z > 255){
        cmd.angular.z = 255;
    }
    wee = "PID Control Verbose";
    cv::putText(outputImage, wee, cv::Point2f(50, 300), font1, 1.3, cv::Scalar(10, 10, 10), thickness, lineType, false);
    wee = "Poportional adjustment (Kp * Pe): ";
    wee += std::to_string(proportional_adj);
    cv::putText(outputImage, wee, proportional_adj_point, font1, 1.1, cv::Scalar(120, 0, 0), thickness, lineType, false);
    wee = "Intergral adjustment (Ki * I): ";
    wee += std::to_string(intergral_adj);
    cv::putText(outputImage, wee, intergral_adj_point, font1, 1.1, cv::Scalar(0, 0, 120), thickness, lineType, false);
    wee = "Derrivitave adjustment (Kd * D): ";
    wee += std::to_string(derrivative_adj);
    cv::putText(outputImage, wee, derrivative_adj_point, font1, 1.1, cv::Scalar(0, 120, 0), thickness, lineType, false);
    // if(angle <= 4){
    //     integral = 0;
    //     //cmd.angular.z = 0;

    // }
   
}else{
    cmd.angular.z = 0;
}
    if (angle <= 5){
    cmd.angular.z = 0;
    integral = 0;

    //cv::Point2f currTraj(m2.x - m1.x, m2.y - m1.y); //current trajectory
    cv::Point2f hammerToRobot = m2 - hammerHitPoint;
    cv::Point2f hammer_to_enemy = enemy_position - hammerHitPoint;
    float dot_hammer_to_enemy = hammer_to_enemy.dot(hammerToRobot);


   // cv::Point2f projected_hammer_to_enemy = dot_hammer_to_enemy * (hammerHitPoint - m2);
    //cv::line(outputImage, projected_hammer_to_enemy, hammerHitPoint, cv::Scalar(100, 100, 100), 5);
    //cv::Point2f displacement_hammer_enemy = hammer_to_enemy - projected_hammer_to_enemy;

    linerror = (linsetpoint - dot_hammer_to_enemy/10000000); //setpoint is always 0, xxxxdividing by sqrt(width*height) to sorta normilize
    current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> dt = current_time - previous_time;
   linintegral += linerror * dt.count();
    linderivative = (linerror - linprevious_error) / dt.count();
    //cmd.linear.x = 1;
    if(linerror < 0){
        linoffset = abs(offset);
    }else{
        linoffset = -1* abs(offset);
    }
    cmd.linear.x = (linKp * linerror + linKi * linintegral + linKd * linderivative - linoffset);

      if(abs(dot_hammer_to_enemy/1000) < 10){
          linintegral = 0;
          ROS_WARN("LIN  LIN    LIN  RESET");
     }

    //my brain is like broken rn

    // derivative = 0;

}else{
    cmd.linear.x = 0; 
}

    wee = "Angular Effort Output: ";
    wee += std::to_string(cmd.angular.z);
    cv::putText(outputImage, wee, cv::Point2f(50, 500), font1, 1.2, cv::Scalar(255, 255, 255), thickness, lineType, false);

   



}else{ //if robot marker is not detected


    std::string temp("ROBOT LOST");

    flashWarning(temp, 400, 450, 3, 3, cv::Scalar(0, 0, 200), 5, 3, &frameCLK_2);

    cmd.linear.x = 0;
    cmd.angular.z = 0;
    cmd.angular.y = 0; //ang y is hammer, yeah should be a custom msg but daid is less than a week away
}


if (cmd.linear.x > 300){
    cmd.linear.x = 300;
}
if(cmd.linear.x < -300){
    cmd.linear.x = -300;
}
//cmd.angular.z = 0;
cmd_pubber.publish(cmd);

previous_error = error;
linprevious_error = linerror;

previous_time = current_time;

}
void BattleVision::flashWarning(std::string msg, int x, int y, double size, int thick, cv::Scalar colour, int blink_interval, int cycle, int* frameCLK){
if (blink_interval == 1){
    cv::putText(outputImage, msg, cv::Point2f(x, y), font2, size, colour, thick, lineType, false);
    return;
}
    if (*frameCLK > blink_interval) {
*frameCLK = 0;
    }

if(*frameCLK > blink_interval / cycle){
cv::putText(outputImage, msg, cv::Point2f(x, y), font2, size, colour, thick, lineType, false);

}
    }





cv::Mat BattleVision::rosToMat(const sensor_msgs::Image::ConstPtr& image) {
    cv_bridge::CvImagePtr image_ptr;
    image_ptr = cv_bridge::toCvCopy(image, image->encoding);
    return image_ptr->image;
}

 void BattleVision::dramaticSetup(){//fancy startup just for fun

int res = 1500;
  for (int i = 0; i <= res; i++) {
    outputImage.setTo(cv::Scalar(0, 0, 0));
    double progress = static_cast<double>(i) / res;
    draw_loading_bar(outputImage, progress);
    flashWarning("SYSTEMS START", window_width / 6, 200, 4, 2, cv::Scalar(200, 200, 200), 100, 3, &frameCLK_3);
    frameCLK_3++;
    frameCLK_4++;
     if(i > res/100){
        cv::putText(outputImage, "ROS core:", cv::Point2f(50, 350), font2, 2, cv::Scalar(200, 200, 200), 1, lineType, false);
        cv::putText(outputImage, "network:", cv::Point2f(50, 410), font2, 2, cv::Scalar(200, 200, 200), 1, lineType, false);
        cv::putText(outputImage, "weapon systems:", cv::Point2f(50, 470), font2, 2, cv::Scalar(200, 200, 200), 1, lineType, false);


    }
    if(i > res/100  && i < res / 5){
        flashWarning("STANDBY", 1000, 350, 2, 2, cv::Scalar(0, 150, 150), 50, 4, &frameCLK_4);
        flashWarning("ROS ERROR", 1000, 410, 2, 2, cv::Scalar(0, 0, 100), 1, 100, &frameCLK_4);
        flashWarning("ROS ERROR", 1000, 470, 2, 2, cv::Scalar(0, 0, 100), 1, 100, &frameCLK_4);

        //flashWarning("NETWORK ERROR", 610 + 45, 410, 2, 2, cv::Scalar(0, 0, 100), 1, 100, &frameCLK_4);

    }
    if(i > res / 5){
        cv::putText(outputImage, "running", cv::Point2f(1000, 350), font2, 2, cv::Scalar(0, 200, 0), 1, lineType, false);

    }
    if(i > res/ 5  && i < 3*res / 5){
        flashWarning("STANDBY", 1000, 410, 2, 2, cv::Scalar(0, 150, 150), 50, 4, &frameCLK_4);
        flashWarning("NETWORK ERROR", 1000, 470, 2, 2, cv::Scalar(0, 0, 100), 1, 100, &frameCLK_4);

       // flashWarning("", 610 + 45, 410, 2, 2, cv::Scalar(0, 0, 100), 1, 100, &frameCLK_4);

    }
    if(i > 3*res/ 5 ){
        cv::putText(outputImage, "online", cv::Point2f(1000, 410), font2, 2, cv::Scalar(0, 200, 0), 1, lineType, false);
    }
      if(i > 3*res/ 5  && i < 4*res / 5){
        flashWarning("STANDBY", 1000, 470, 2, 2, cv::Scalar(0, 150, 150), 50, 4, &frameCLK_4);

       // flashWarning("", 610 + 45, 410, 2, 2, cv::Scalar(0, 0, 100), 1, 100, &frameCLK_4);

    }
    if(i > 4*res/ 5 ){
        cv::putText(outputImage, "online", cv::Point2f(1000, 470), font2, 2, cv::Scalar(0, 200, 0), 1, lineType, false);
    }

    cv::imshow("Battle Vision", outputImage);
    cv::waitKey(5);
  }
    outputImage.setTo(cv::Scalar(0, 0, 0));
  cv::putText(outputImage, "INITIATION SUCCESS", cv::Point(window_width / 4, window_height / 2), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(100, 200, 100), 2);
  cv::imshow("Battle Vision", outputImage);
  cv::waitKey(1000);
  outputImage.setTo(cv::Scalar(0, 0, 0));
  cv::imshow("Battle Vision", outputImage);
  cv::waitKey(500);
  cv::putText(outputImage, "ALL SYSTEMS OPERATIONAL", cv::Point(window_width / 4, window_height / 2), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 255, 255), 2);
  cv::imshow("Battle Vision", outputImage);
  cv::waitKey(1000);
  outputImage.setTo(cv::Scalar(0, 0, 0));
  cv::imshow("Battle Vision", outputImage);
  cv::waitKey(500);
  cv::putText(outputImage, "OPENING CAMERA FEED...", cv::Point(window_width / 4, window_height / 2), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 255, 255), 2);
  cv::imshow("Battle Vision", outputImage);
  cv::waitKey(1500);



 }

void BattleVision::draw_loading_bar(cv::Mat &output, double progress) {
int bar_width = 1000;
int bar_height = 10;
int bar_x = (window_width - bar_width) /2;
int bar_y = (window_height - bar_height) - 200;

//float
cv::Rect outer_rect(bar_x, bar_y, bar_width, bar_height);
cv::rectangle(output, outer_rect, cv::Scalar(255, 255, 255), 2);

int inner_width = static_cast<int>(bar_width * progress);
cv::Rect inner_rect(bar_x + 2, bar_y + 2, inner_width - 4, bar_height - 4);
cv::rectangle(output, inner_rect, cv::Scalar(255, 255, 255), -1);
}


 