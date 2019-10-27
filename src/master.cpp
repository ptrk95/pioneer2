#include "ros/ros.h"
#include "opencv2/opencv.hpp"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/String.h"
#include "pioneer2/control.h"


static int height = 360;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
   {

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    float scale = float(height)/float(cv_ptr->image.rows);
    cv::Mat image;
    cv::resize(cv_ptr->image, image, cv::Size(0,0),scale, scale, CV_INTER_AREA);
    cv::imshow("video stream", image);
    cv::waitKey(1);
    
   }

int main(int argc,  char  **argv)
{
    ros::init(argc, argv, "master");

    ros::NodeHandle node_handle;

    image_transport::ImageTransport image_trans(node_handle);
    image_transport::Subscriber image_sub;
    image_sub = image_trans.subscribe("qr_scanner/video_stream", 4, imageCallback);
    
    ros::Publisher pub = node_handle.advertise<pioneer2::control>("visualizer/servo_control", 2);
	pioneer2::control servo_msg;
    servo_msg.msg = "pan_camera";
    servo_msg.num = 45;
    pub.publish(servo_msg);
    ros::spin();
    cv::destroyAllWindows();

    return 0;
}
