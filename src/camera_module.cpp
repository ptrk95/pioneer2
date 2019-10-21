#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "chrono"
#include "thread"

static const std::string OPENCV_WINDOW = "Image window";
static const std::string URL = "http://192.168.0.3:8080/video";
static std::string video_src = "/home/ubuntu/catkin_ws/src/pioneer2/videos/test.mp4";
static int publish_rate = 10;
static int height = 800;
static bool piCamera = true;

int main(int argc, char  **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle node_handle;

    image_transport::Publisher image_pub;
    image_transport::ImageTransport image_trans(node_handle);

    image_pub = image_trans.advertise("camera_module/video_stream", 1);
    ros::param::get("camera_module/publish_rate", publish_rate);
    ros::param::get("camera_module/height", height);
    ros::param::get("camera_module/video_src", video_src);
    ros::param::get("camera_module/piCamera", piCamera);
    ros::Rate loop_rate(publish_rate);

	cv::VideoCapture cap;
	
	if(piCamera){
		cap.open(0);
	}else{
		cap.open(video_src);
	}
    
    
    // wait for camera to setup
    std::this_thread::sleep_for(std::chrono::seconds(2));

    if(!cap.isOpened()){
        std::cerr << "Can't open video. Please check path of param 'video_src' in pioneer2launch.launch!";
        return 1;
    }
    
    
    int counter = 0;

    cv::Mat frame;

    while(ros::ok()){

        
        cap >> frame;
        if(frame.empty()){
            std::cerr << "Frame is empty.";
            return 1;
        }

        float scale = float(height)/float(frame.rows);
        cv::resize(frame, frame, cv::Size(0,0),scale, scale, CV_INTER_AREA);

        //cv::cvtColor(frame, frame, CV_RGB2BGR);

        sensor_msgs::Image img_msg;

        std_msgs::Header header; // empty header
        header.seq = counter; // user defined counter
        header.stamp = ros::Time::now(); // time
        cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);
        img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image 

        image_pub.publish(img_msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++counter;
    }
    cap.~VideoCapture();
    return 0;
}
