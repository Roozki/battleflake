//quick multithread camera pubber

// #include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.h>
// #include <opencv2/highgui/highgui.hpp>
// #include <ros/ros.h>
// #include <thread>

#include <init_camera/camera_pubber.h>
// using namespace cv;
// using namespace std;
// using namespace concurrency;

void shutter(){
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    VideoCapture cap(1); // captures the first camera
    float fps = 30;
   

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("cam_1/color/image_raw", 2);

         unsigned int width = cap.get(CAP_PROP_FRAME_WIDTH); 
    unsigned int height = cap.get(CAP_PROP_FRAME_HEIGHT); 
    unsigned int pixels = width*height;
         Mat inputImage(1280, 720, 1280*720);

 if (!cap.isOpened()) {
        cout << "Camera cannot be opened" << endl;
        ROS_ERROR("AHHH");  
           //  VideoCapture cap(1); // captures the first camera
  }


 cap.set(CAP_PROP_FPS, fps);
while (nh.ok()) {
        cap.read(inputImage);
        // if (!isRead) {
        //     cout << "Failed to read image from camera" << endl;
        //     break;
        // }
       // imshow(inputWindow, inputImage);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", inputImage).toImageMsg();
        pub.publish(msg);
        //waitKey(1000/120);
        ros::spinOnce();
        loop_rate.sleep();
    }
    cap.release();
}

int main(int argc, char** argv) {
 //   string inputWindow = "Camera";
  //  namedWindow(inputWindow, CV_WINDOW_AUTOSIZE);


    ros::init(argc, argv, "camera_pubber");
    //clock_t t1;
    std::thread t1(shutter);
        t1.join();// = clock();

   

    


}











    