#include <BattleVision.h>
/*
author: rowan zawadzki

*/




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
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    parameters = cv::aruco::DetectorParameters::create();

    std::string topic_to_subscribe_to = "cam_1/color/image_raw";

    int queue_size = 5;
    shutter  = it.subscribe(
    topic_to_subscribe_to, queue_size, &BattleVision::frameCallback, this);

    std::string topic = private_nh.resolveName("identified");
    queue_size        = 10;

   // point_pub = private_nh.advertise<bb_msgs::bbVision2point>("battlepoint", queue_size);

    cmd_pubber = nh.advertise<geometry_msgs::Twist>("robot1_cmd_vel", queue_size);

    // image detection publisher
    bounder = it.advertise("bounding_boxes", 5);
    cv::namedWindow("Battle Vision", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("Battle Vision", &BattleVision::clickCallbackHandler, this);
    //Size targetResolution(1920, 1080);

    ROS_INFO("initiation appears successfull.");

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


    while (ros::ok())
    {

    cap >> frame;
  
      
            ros::spinOnce();

            cv::waitKey(1000/fps);
            frameCLK_1++;
            frameCLK_2++;            

            loop_rate.sleep();
  if (frame.empty())
         {
             ROS_WARN("Failed to read frame from camera");
                    
            

         }else{
            processMarkers(frame);
         }

    }

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
   


    cv::aruco::detectMarkers(
    image, dictionary, markerCorners, markerIds, parameters);
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
                BattleVision::m1 = (markerCorners[robot_index][0] + markerCorners[robot_index][1])/2;
                BattleVision::m2 = (markerCorners[robot_index][2] + markerCorners[robot_index][3])/2;
                cv::Point2f robot_longitudinal_direction = m2 - m1;
                cv::Point2f robot_lateral_direction = markerCorners[robot_index][2] - markerCorners[robot_index][3];

                cv::Point2f front_robot = m1 + robot_longitudinal_direction*ROBOT_LONG_SCALE;
                cv::Point2f right_robot = m1 + robot_lateral_direction*ROBOT_LAT_SCALE;
                cv::Point2f left_robot = m1 - robot_lateral_direction*ROBOT_LAT_SCALE;

                
                //robot 'render'
                cv::line(outputImage, BattleVision::m1, front_robot, cv::Scalar(0, 0, 200), 5);
                cv::line(outputImage, BattleVision::m1, right_robot, cv::Scalar(100, 100, 0), 5);
                cv::line(outputImage, BattleVision::m1, left_robot, cv::Scalar(100, 100, 0), 5);
                std::string hammer_msg = "HAMMER ";
                cv::Scalar hamer_msg_colour;
                switch (hammer_STATUS)
                {
                case 1:
                    hammer_msg += "READY";
                    hamer_msg_colour = cv::Scalar(0, 250, 0);
                    break;
                case 0:
                    hammer_msg += "STANDBY";
                    hamer_msg_colour = cv::Scalar(255, 255, 0);
                    break;
                default:
                    hammer_msg += "READY";
                    hamer_msg_colour = cv::Scalar(0, 250, 0);
                    break;
                }
                flashWarning(hammer_msg, 50, 600, 1, 1, cv::Scalar(0, 200, 200), 5, 3, &frameCLK_2);


                //desired traj vector
                cv::line(outputImage, BattleVision::m1, click, cv::Scalar(200, 100, 0), 4);

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
    cv::Point2f tmp(x, y);
    click = tmp;

}


void BattleVision::sendCmd(){
//TODO: sophisticated PID
//may need to be in another node
geometry_msgs::Twist cmd;
if(robotTracked){

    std::string temp("ROBOT LOCKED");
    cv::putText(outputImage, temp, robot_locked_point, font2, 2.0, cv::Scalar(0, 200, 0), thickness, lineType, false);

cv::Point2f currTraj(m2.x - m1.x, m2.y - m1.y); //current trajectory
cv::Point2f desTraj(click.x - m1.x, click.y - m1.y); //desired traj


 float dot = desTraj.dot(currTraj);
 float mag1 = norm(currTraj);
 float mag2 = norm(desTraj);

 float cos_theta = dot/(mag1 * mag2);
 float angle = fastAtan2(sqrt(1 - cos_theta*cos_theta), cos_theta);

float cross = currTraj.x * desTraj.y - currTraj.y * desTraj.x;



if (cross > 0){
std::string wee("angle is: ");
wee += std::to_string(angle);
    cv::putText(outputImage, wee, angle_to_go_point, font1, fontScale, cv::Scalar(0, 100, 0), thickness, lineType, false);
    cmd.angular.z = -40;
}else if (cross < 0){
std::string wee("angle is: ");
wee += std::to_string(-angle);
    cv::putText(outputImage, wee, angle_to_go_point, font1, fontScale, cv::Scalar(110, 0, 0), thickness, lineType, false);

    cmd.angular.z = 40;

}

if (angle < 20){
    if(mag2 > 50){
    cmd.linear.x = -35;
    }else{
    cmd.linear.x = 0;
    }
    cmd.angular.z = 0;

}else{
    cmd.linear.x = 0; 
}
if(mag2 < 70){
    cmd.angular.z = 0;
    }
}else{ //if robot marker is not detected


    std::string temp("ROBOT LOST");

    flashWarning(temp, 400, 450, 3, 3, cv::Scalar(0, 0, 200), 5, 3, &frameCLK_2);

    cmd.linear.x = 0;
    cmd.angular.z = 0;
}


if (cmd.linear.x < -35){
    cmd.linear.x = -35;
}

cmd_pubber.publish(cmd);

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
  cv::putText(outputImage, "battlebot ready", cv::Point(window_width / 4, window_height / 2), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 255, 255), 2);
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


 