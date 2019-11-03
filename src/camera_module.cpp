#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "pioneer2/control.h"

static const std::string OPENCV_WINDOW = "Image window";
static const std::string URL = "http://192.168.0.3:8080/video";
static std::string video_src = "/home/ubuntu/catkin_ws/src/pioneer2/videos/test.mp4";
static int publish_rate = 10;
static int height = 800;
static bool piCamera = true;
static int angletestTilt = 0;
static int angletestPan = 0;

int main(int argc, char  **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle node_handle;

    image_transport::Publisher image_pub;
    image_transport::ImageTransport image_trans(node_handle);
    ros::Publisher pub = node_handle.advertise<pioneer2::control>("camera_module/robot_control", 2);
    ros::Publisher pub_servo = node_handle.advertise<pioneer2::control>("visualizer/servo_control", 2);
   ros::param::get("camera_module/angletestPan", angletestPan);
    ros::param::get("camera_module/angletestTilt", angletestTilt);
    pioneer2::control message;
    message.msg = "pan_camera";
    message.num = angletestPan;

    pioneer2::control servo_msg;
    servo_msg.msg = "tilt_camera";
    servo_msg.num = angletestTilt;
    

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
    ros::Duration(2).sleep();

    if(!cap.isOpened()){
        std::cerr << "Can't open video. Please check path of param 'video_src' in pioneer2launch.launch!";
        return 1;
    }
    
    
    int counter = 0;

    cv::Mat frame;

    // fixed 1920x1080 capture size
    float scale = float(height)/float(1080);
    float width = scale * 1920;

 
    if(!cap.set(CV_CAP_PROP_FRAME_WIDTH, width)){
        std::cout <<"SUCCESS\n";
    }else{
        std::cout <<"FAIL\n";
    }


    if(!cap.set(CV_CAP_PROP_FRAME_HEIGHT, height)){
        std::cout <<"SUCCESS\n";
    }else{
        std::cout <<"FAIL\n";
    }

    if(!cap.set(CV_CAP_PROP_FPS, publish_rate)){
        std::cout <<"SUCCESS\n";
    }else{
        std::cout <<"FAIL\n";
    }
	
    while(ros::ok()){

        cap >> frame;
        if(frame.empty()){
            std::cerr << "Frame is empty.";
            return 1;
        }

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
