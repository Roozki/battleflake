#include <BattleVision.h>
/*
author: rowan zawadzki

*/




BattleVision::BattleVision(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Rate loop_rate(30);


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

    ROS_INFO("initiation appears successfull.");
    if (draw_markers) {
        ROS_INFO("I WILL ATTEMPT TO DRAW DETECTED MARKERS");
    } else {
        ROS_WARN(
        "I WILL ****NOT**** ATTEMPT TO DRAW DETECTED MARKERS");
    }

    //Create a window
     VideoCapture cap(1);
     
     if (!cap.isOpened())
    {
        ROS_ERROR("Failed to open camera");
    }
    
    //double fps = cap.get(CAP_PROP_FPS);
    ROS_WARN("%f", fps);
     cv::namedWindow("Battle Eye", 0);
     cv::setMouseCallback("Battle Eye", &BattleVision::clickCallbackHandler, this);

      Mat frame;
      Size targetResolution(1280, 720);

    
    cap.set(CAP_PROP_FPS, BattleVision::fps);

    while (ros::ok())
    {

    cap >> frame;
  
    //qDebug()  <<" VideoStreamer::queryFrame " + QString::number(elapsedTimer.elapsed());
   // resize(frame, frame, targetResolution);
  //  Mat img = frame;//imread(frame);
        if (frame.empty())
         {
             ROS_ERROR("Failed to read frame from camera");
         }
            ros::spinOnce();

            cv::waitKey(1000/30);
            loop_rate.sleep();

            processMarkers(frame);

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
    if (draw_markers) {
        image.copyTo(outputImage);
        if(markerCorners.size() == 1){
            robotTracked = true;
            BattleVision::m1 = (markerCorners[0][0] + markerCorners[0][1])/2;
            BattleVision::m2 = (markerCorners[0][2] + markerCorners[0][3])/2;
            cv::line(outputImage, BattleVision::m1, BattleVision::m2, cv::Scalar(0, 0, 255), 4);
            cv::line(outputImage, BattleVision::m1, click, cv::Scalar(200, 100, 0), 4);

        }else{
            robotTracked = false;
        }

        cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
        bounder.publish(
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", outputImage)
        .toImageMsg());


    imshow("Battle Eye", outputImage);
    sendCmd();

    }

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

cv::Point2f currTraj(m2.x - m1.x, m2.y - m1.y); //current trajectory
cv::Point2f desTraj(click.x - m1.x, click.y - m1.y); //desired traj


 float dot = desTraj.dot(currTraj);
 float mag1 = norm(currTraj);
 float mag2 = norm(desTraj);

 float cos_theta = dot/(mag1 * mag2);
 float angle = fastAtan2(sqrt(1 - cos_theta*cos_theta), cos_theta);

float cross = currTraj.x * desTraj.y - currTraj.y * desTraj.x;



if (cross > 0){
ROS_ERROR("angle is clockwise: %.5f", angle);

    cmd.angular.z = -40;
}else if (cross < 0){
ROS_WARN("angle is counterclockwise: %.5f", angle);

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
}else{
    cmd.linear.x = 0;
    cmd.angular.z = 0;
}


if (cmd.linear.x < -35){
    cmd.linear.x = -35;
}

cmd_pubber.publish(cmd);

}


cv::Mat BattleVision::rosToMat(const sensor_msgs::Image::ConstPtr& image) {
    cv_bridge::CvImagePtr image_ptr;
    image_ptr = cv_bridge::toCvCopy(image, image->encoding);
    return image_ptr->image;
}

 