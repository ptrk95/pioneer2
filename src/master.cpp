#include "ros/ros.h"
#include "opencv2/opencv.hpp"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/String.h"
#include "pioneer2/control.h"
#include "std_msgs/Int32MultiArray.h"

static int height_roi = 400;
static int width_roi = 640;

volatile int pan_angle = 0;

int width = 640;
int height = 480;

pioneer2::control cont_msg = pioneer2::control();
pioneer2::control servo_msg = pioneer2::control();

image_transport::Publisher pub;
ros::Publisher pub_robot;
ros::Publisher pub_servo;
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

bool check_pan_angle(int angle){
    if(pan_angle + angle <= 75 && pan_angle + angle >= -75){
        return true;
    }else{
        return false;
    }
}

void robot_turn_left(int angle){
    cont_msg.msg = "turn_left";
    cont_msg.num = angle;
   // pub_robot.publish(cont_msg);
}

void robot_turn_right(int angle){
    cont_msg.msg = "turn_right";
    cont_msg.num = angle;
   // pub_robot.publish(cont_msg);
}

void qr_pos_Callback(const std_msgs::Int32MultiArray &msg){
    
    if(!qr_positions.empty()){
        
	std::vector<int> pos = msg.data;
int offset = (width - width_roi) /2;
if(pos[0] <= offset + width_roi*0.25){
//cont_msg.msg = "turn_left";
servo_msg.msg = "pan_camera";
servo_msg.num = 15;
if(!check_pan_angle(15)){
    robot_turn_left(75);
    servo_msg.num = -75;
}
}else if(pos[0]>= offset + width_roi*0.75){
//cont_msg.msg = "turn_right";
servo_msg.msg = "pan_camera";
servo_msg.num = -15;
if(!check_pan_angle(-15)){
    robot_turn_right(75);
    servo_msg.num = 75;
}
}else{
cont_msg.msg = "drive";
}

int offset_h = (height - height_roi) /2;
if(pos[1] <= offset_h + height_roi*0.25){
servo_msg.msg = "tilt_camera";
servo_msg.num = 8;
}else if(pos[1] >= offset_h + height_roi*0.75){
servo_msg.msg = "tilt_camera";
servo_msg.num = -8;

}

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
ros::param::get("camera_module/width", width);
ros::param::get("camera_module/height", height);

    image_transport::ImageTransport image_trans(node_handle);
    image_transport::Subscriber image_sub;
    image_sub = image_trans.subscribe("qr_scanner/video_stream", 1, imageCallback);

    ros::Subscriber qr_pos_sub;
    qr_pos_sub = node_handle.subscribe("qr_scanner/qr_pos", 1, qr_pos_Callback);
    ros::NodeHandle nh;
    pub = image_trans.advertise("master/video_stream", 1);
    pub_robot = nh.advertise<pioneer2::control>("master/robot_control", 1);
    pub_servo = nh.advertise<pioneer2::control>("master/servo_control", 1);
	servo_msg.msg = "stop";
servo_msg.num = 0;
cont_msg.msg = "stop";
    cont_msg.num = 0;
ros::Rate loop_rate(30);

while(ros::ok()){

pub_servo.publish(servo_msg);
servo_msg.msg = "stop";
servo_msg.num = 0;
pub_robot.publish(cont_msg);
cont_msg.msg = "stop";
    cont_msg.num = 0;
ros::spinOnce();
loop_rate.sleep();

}
   
   
    clear(qr_positions);

    return 0;
}
