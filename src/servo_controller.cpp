
#include "pca9685.h"

#include <wiringPi.h>

#include "std_msgs/String.h"
#include "pioneer2/control.h"
#include "ros/ros.h"

#include <iostream>
#include <math.h>

#define PIN_BASE 300
#define MAX_PWM 4096
#define HERTZ 50

class servo_controller{
	
	
	public:

	servo_controller(ros::NodeHandle &node_handle){
		// Calling wiringPi setup first.
		wiringPiSetup();

		// Setup with pinbase 300 and i2c location 0x40
		int fd = pca9685Setup(PIN_BASE, 0x40, HERTZ);
		if (fd < 0)
		{
			std::cerr << "Error in setup of servo_controller!\n" << fd;
			return;
		}

		// Reset all output
		pca9685PWMReset(fd);


		// Set servo to neutral position at 1.5 milliseconds
		// (View http://en.wikipedia.org/wiki/Servo_control#Pulse_duration)
		float millis = 1.5;
		int tick = calcTicks(millis, HERTZ);
		pwmWrite(PIN_BASE + 16, tick);
		delay(2000);

		std::cerr << "test servo cont!\n";
		//ros::param::get("visualizer/height", height);

		ros::Subscriber cont_sub = node_handle.subscribe("visualizer/servo_control", 4, &servo_controller::controllerCallback,this);
		std::cerr << "test servo cont!\n";
		std::cerr << cont_sub.getNumPublishers();
	}

	~servo_controller(){
	}

	void controllerCallback(const pioneer2::control::ConstPtr &msg){
		std::cout << "pan_camera sdfsdf";
		if(msg->msg == "pan_camera"){
			pan_camera(msg->num);
			std::cout << "pan_camera horizontal";
		}else if(msg->msg == "tilt_camera"){
			pan_camera(msg->num);
			std::cout << "tilt_camera vertikal";
		}else if(msg->msg == "drive"){

		}
	}

	float angle_to_milliseconds(int angle){
		if(angle >= -90 & angle < 0){
			return 1 + abs((0.5/90) * angle);

		}else if(angle== 0){
			return 1.5;
		}else if(angle <= 90 & angle > 0)
		{
			return 1.5 + abs((0.5/90) * angle);
		}else{
			std::cerr << "Error in rotating camera!\n" << "Only angles between -90 and 90 allowed!";
		}
		
	}

	//Rotate camera along z-axis(up) for angle degrees. -90 to 90 degrees allowed.
	void pan_camera(int angle){
		if(angle >= -90 & angle < 90){
			float milliseconds = angle_to_milliseconds(angle);
			int tick = calcTicks(milliseconds, HERTZ);
			pwmWrite(PIN_BASE + 1, tick);
			delay(2000);
		}else{
			std::cerr << "Error in rotating camera!\n" << "Only angles between -90 and 90 allowed!";
		}
		
	}

	//Rotate camera along x/y-axis(side) for angle degrees. -90 to 90 degrees allowed.
	void tilt_camera(int angle){
		if(angle >= -90 & angle < 90){
			float milliseconds = angle_to_milliseconds(angle);
			int tick = calcTicks(milliseconds, HERTZ);
			pwmWrite(PIN_BASE + 2, tick);
			delay(2000);
		}else{
			std::cerr << "Error in rotating camera!\n" << "Only angles between -90 and 90 allowed!";
		}
	}

	/**
	 * Calculate the number of ticks the signal should be high for the required amount of time
	 */
	int calcTicks(float impulseMs, int hertz)
	{
		float cycleMs = 1000.0f / hertz;
		return (int)(MAX_PWM * impulseMs / cycleMs + 0.5f);
	}


};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "servo_controller");
	ros::NodeHandle node_handle;
	ros::Rate r(30);
	

	servo_controller serv_cont = servo_controller(node_handle);
	while(ros::ok()){
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
