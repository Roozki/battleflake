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

    point_pub = private_nh.advertise<bb_msgs::bbVision2point>("battlepoint", queue_size);

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
     VideoCapture cap(0);
     
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
         cap.set(CAP_PROP_FPS, BattleVision::fps);
        ros::spinOnce();

        imshow("Battle Eye", frame);
            cv::waitKey(1);
            loop_rate.sleep();

            ROS_INFO("wee");

    }

}


void BattleVision::frameCallback(const sensor_msgs::Image::ConstPtr& msg) {
    ROS_INFO("Received message");
    std::vector<int> markerIds = processMarkers(rosToMat(msg));
    std::cout << "Markers detected:";
    for (int markerId : markerIds) { std::cout << " " << markerId; }
    std::cout << std::endl;
}

std::vector<int> BattleVision::processMarkers(const cv::Mat& image) {
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    // /cv::Point2f p1(0.0, 10.0);
   // cv::Point2f p2(50.0, 50.0);
   


    cv::aruco::detectMarkers(
    image, dictionary, markerCorners, markerIds, parameters);
    if (draw_markers) {
        cv::Mat outputImage;
        image.copyTo(outputImage);
        if(markerCorners.size() > 1){

            cv::Point2f m1 = (markerCorners[0][0] + markerCorners[0][2])/2;
            cv::Point2f m2 = (markerCorners[1][0] + markerCorners[1][2])/2;
            cv::line(outputImage, m1, m2, cv::Scalar(0, 0, 255), 5);
        }

        cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
        bounder.publish(
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", outputImage)
        .toImageMsg());
        ROS_INFO("attempted draw");
    }
    return markerIds;
}


cv::Mat BattleVision::rosToMat(const sensor_msgs::Image::ConstPtr& image) {
    cv_bridge::CvImagePtr image_ptr;
    image_ptr = cv_bridge::toCvCopy(image, image->encoding);
    return image_ptr->image;
}

 