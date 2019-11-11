#include "ros/ros.h"
#include "opencv2/opencv.hpp"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/String.h"
#include "pioneer2/control.h"
#include "std_msgs/Int32MultiArray.h"

static int height_roi = 400;
static int width_roi = 640;

image_transport::Publisher pub;

std::queue<std::vector<int>> qr_positions = std::queue<std::vector<int>>();

void drawLine(cv::Mat &img){
    if(!qr_positions.empty()){
        if(qr_positions.size() >= 2){
			//cv::Rect roi_rect = cv::Rect((img.cols - width_roi)/2, (img.rows - height_roi)/2, width_roi, height_roi);
			//cv::Mat roi = img(roi_rect);
            std::vector<int> pos2 = qr_positions.front();
            std::vector<int> pos1 = qr_positions.back();
            cv::line(img, cv::Point(pos1[0], pos1[1]) , cv::Point(pos2[0], pos2[1]), cv::Scalar(255,0,0), 3);
            qr_positions.pop();
        }
    }
}

void qr_pos_Callback(const std_msgs::Int32MultiArray &msg){
    if(!qr_positions.empty()){
        if(qr_positions.size() <= 4){
            qr_positions.push(msg.data);
        }else{
            qr_positions.pop();
            qr_positions.push(msg.data);
        }
    }else{
        qr_positions.push(msg.data);
    }
}

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

    drawLine(cv_ptr->image);
    pub.publish(cv_ptr->toImageMsg());

    
}

void clear( std::queue<std::vector<int>> &q )
{
   std::queue<std::vector<int>> empty;
   std::swap( q, empty );
}

int main(int argc,  char  **argv)
{
    ros::init(argc, argv, "master");

    ros::NodeHandle node_handle;
    
    ros::param::get("qr_scanner/height_roi", height_roi);
    ros::param::get("qr_scanner/width_roi", width_roi);

    image_transport::ImageTransport image_trans(node_handle);
    image_transport::Subscriber image_sub;
    image_sub = image_trans.subscribe("qr_scanner/video_stream", 1, imageCallback);

    ros::Subscriber qr_pos_sub;
    qr_pos_sub = node_handle.subscribe("qr_scanner/qr_pos", 1, qr_pos_Callback);
    
    pub = image_trans.advertise("master/video_stream", 1);
	
    ros::spin();
   
    clear(qr_positions);

    return 0;
}
