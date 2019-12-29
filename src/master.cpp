#include "ros/ros.h"
#include "opencv2/opencv.hpp"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/String.h"
#include "pioneer2/control.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Trigger.h"

#include <string>

#include "wiringPi.h"


#define LED 18

static int height_roi = 400;
static int width_roi = 640;

int width = 640;
int height = 480;
int publish_rate = 15;

pioneer2::control cont_msg = pioneer2::control();
pioneer2::control servo_msg_pan = pioneer2::control();
pioneer2::control servo_msg_tilt = pioneer2::control();

ros::ServiceClient service_pan_pos;
std_srvs::Trigger srv;

image_transport::Publisher pub;
ros::Publisher pub_robot;
ros::Publisher pub_servo_pan;
ros::Publisher pub_servo_tilt;

bool unregistered = false;
bool stop_blinking = false;
bool stop_blink = false;
bool started_blink = false;
int reset_counter = 0;

bool turning_robot_right = false;
bool turning_robot_left = false;


int turn_timer_ms = 2500;

PI_THREAD (blinky)
{
  while(!stop_blinking){
    digitalWrite (LED, HIGH) ;	// On
    delay (500) ;		// mS
    digitalWrite (LED, LOW) ;	// Off
    delay (500) ;
  }
	return 0;
}

PI_THREAD (blink)
{
started_blink = true;
while(!stop_blink){
    digitalWrite (LED, HIGH) ;	// On
    delay (100) ;		// mS
}
digitalWrite (LED, LOW) ;	// Off
started_blink = false;
	return 0;
}

int get_pan_angle_servo(){

service_pan_pos.call(srv);
return std::stoi(srv.response.message);

}

void robot_turn_left(int angle){
    cont_msg.msg = "turn_left";
    cont_msg.num = -angle;
   // pub_robot.publish(cont_msg);
}

void robot_turn_right(int angle){
    cont_msg.msg = "turn_right";
    cont_msg.num = angle;
   // pub_robot.publish(cont_msg);
}

PI_THREAD (turn_left_timer)
{
	turning_robot_left = true;
  	delay(turn_timer_ms);
	int pan_angle = get_pan_angle_servo();
	if(pan_angle != 0){
		robot_turn_left(abs(pan_angle)+15);
        servo_msg_pan.msg = "force_0_pan_camera";
		servo_msg_pan.num = 0;
        pub_servo_pan.publish(servo_msg_pan);
	}
	
	turning_robot_left = false;
	return 0;
}

PI_THREAD (turn_right_timer)
{
	turning_robot_right = true;
  	delay(turn_timer_ms);
	int pan_angle = get_pan_angle_servo();
	if(pan_angle != 0){
		robot_turn_right(abs(pan_angle)+15);
		servo_msg_pan.msg = "force_0_pan_camera";
		servo_msg_pan.num = 0;
        pub_servo_pan.publish(servo_msg_pan);
	}
	turning_robot_right = false;
	return 0;
}

void start_turn_left_timer(){
	piThreadCreate (turn_left_timer);
}

void start_turn_right_timer(){
	piThreadCreate (turn_right_timer);
}

void start_blink(){
	piThreadCreate (blink);
}

void start_blinking(){
	piThreadCreate (blinky);
}






void qr_reset_Callback(const std_msgs::Bool &msg){
unregistered = msg.data;
if(msg.data == true){
if(reset_counter ==0){
std::cout << "\nQrCode unregistered." << std::endl;
std::cout << "Scan Qr-code to register and follow it." << std::endl;
stop_blinking = false;
start_blinking();
}
reset_counter++;
}else{
reset_counter--;
if(reset_counter == 0){
stop_blinking = true;
std::cout << "\nQrCode registered. (Data from QrCode may print several times)" << std::endl;
}
}
}




bool check_pan_angle(int angle){ //maximum angle reached
int pan_angle = get_pan_angle_servo();
    if(pan_angle + angle <= 45 && pan_angle + angle >= -45){
        return true;
    }else{
        return false;
    }
}






void qr_pos_Callback(const std_msgs::Int32MultiArray &msg){
        
	std::vector<int> pos = msg.data;
int offset = (width - width_roi) /2;
if(pos[0] <= offset + width_roi*0.25){ // turn left
//cont_msg.msg = "turn_left";
servo_msg_pan.msg = "pan_camera";
servo_msg_pan.num = 15;
if(!check_pan_angle(15)){
    robot_turn_left(45);
    servo_msg_pan.num = -45;
}else{
	if(!turning_robot_left){
	start_turn_left_timer();
}
}
}else if(pos[0]>= offset + width_roi*0.75){ // turn right
//cont_msg.msg = "turn_right";
servo_msg_pan.msg = "pan_camera";
servo_msg_pan.num = -15;
if(!check_pan_angle(-15)){
    robot_turn_right(45);
    servo_msg_pan.num = 45;
}else{
	if(!turning_robot_right){
	start_turn_right_timer();
	}
}
}else if(get_pan_angle_servo() == 0){
cont_msg.msg = "drive";
}

int offset_h = (height - height_roi) /2;
if(pos[1] <= offset_h + height_roi*0.25){
servo_msg_tilt.msg = "tilt_camera";
servo_msg_tilt.num = 8;
}else if(pos[1] >= offset_h + height_roi*0.75){
servo_msg_tilt.msg = "tilt_camera";
servo_msg_tilt.num = -8;

}

pub_servo_pan.publish(servo_msg_pan);
servo_msg_pan.msg = "stop";
servo_msg_pan.num = 0;
pub_servo_tilt.publish(servo_msg_tilt);
servo_msg_tilt.msg = "stop";
servo_msg_tilt.num = 0;

       
	if(!started_blink)
	{
	start_blink();
	}
stop_blink = false;
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

    wiringPiSetupGpio();
    pinMode(LED, OUTPUT); // 1 for output
    
    ros::param::get("qr_scanner/height_roi", height_roi);
    ros::param::get("qr_scanner/width_roi", width_roi);
ros::param::get("camera_module/width", width);
ros::param::get("camera_module/height", height);
ros::param::get("master/publish_rate", publish_rate);
ros::param::get("master/turn_timer_ms", turn_timer_ms);

    image_transport::ImageTransport image_trans(node_handle);
    image_transport::Subscriber image_sub;
    image_sub = image_trans.subscribe("qr_scanner/video_stream", 1, imageCallback);
	ros::Subscriber qr_reset_sub;
    qr_reset_sub = node_handle.subscribe("qr_scanner/qr_reset", 4, qr_reset_Callback);
    ros::Subscriber qr_pos_sub;
    qr_pos_sub = node_handle.subscribe("qr_scanner/qr_pos", 1, qr_pos_Callback);
    ros::NodeHandle nh;
    pub = image_trans.advertise("master/video_stream", 1);
    pub_robot = nh.advertise<pioneer2::control>("master/robot_control", 1);
    pub_servo_pan = nh.advertise<pioneer2::control>("master/servo_control_pan", 1);
 pub_servo_tilt = nh.advertise<pioneer2::control>("master/servo_control_tilt", 1);
	servo_msg_pan.msg = "stop";
servo_msg_pan.num = 0;
servo_msg_tilt.msg = "stop";
servo_msg_tilt.num = 0;
cont_msg.msg = "stop";
    cont_msg.num = 0;
//ros::wait_for_service("servo_controller/pan_pos");
service_pan_pos = nh.serviceClient<std_srvs::Trigger>("servo_controller/pan_angle");

if(!service_pan_pos.exists()){
service_pan_pos.waitForExistence();

}
ros::Rate loop_rate(publish_rate);

while(ros::ok()){

pub_robot.publish(cont_msg);
cont_msg.msg = "stop";
    cont_msg.num = 0;
stop_blink = true;
ros::spinOnce();
loop_rate.sleep();

}
   
   digitalWrite (LED, LOW) ;

    return 0;
}
