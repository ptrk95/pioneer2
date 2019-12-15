#include "Aria.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pioneer2/control.h"


 ros::Subscriber cont_sub;

// add Actions for automatic driving
  ArActionLimiterForwards limiter("speed limiter near", 800, 1000, 100); // arguments: name, stopDistance mm, slowDownDistance mm, maxSeed mm/sec
  ArActionLimiterForwards limiterFar("speed limiter far", 400, 1250, 300);
  ArActionConstantVelocity drive("drive", 150); // 400mm/sec
  ArActionStop stop("stop");
  ArActionTurn turn_left("turn left", 100, 0, 2);
ArActionTurn turn_right("turn right", 100, 0, -2);

void controllerCallback(const pioneer2::control::ConstPtr &msg){
  if(msg->msg == "stop"){
    
drive.deactivate();
turn_left.deactivate();
turn_right.deactivate();
  }else if(msg->msg == "stop_now"){
    if(!stop.isActive()){
      stop.activate();
    }
	
  }else if(msg->msg == "drive"){
    if(!drive.isActive()){
	
      drive.activate();
turn_left.deactivate();
turn_right.deactivate();
    }
  }else if(msg->msg == "turn_left"){
	//robot.setRotVel(msg->num);
	if(!turn_left.isActive()){
	std::cout<<"turn_left" <<std::endl;
      turn_left.activate();
turn_right.deactivate();
drive.deactivate();
    }
	//robot.stop();
	//robot.stopRunning();
  }else if(msg->msg == "turn_right"){
	//robot.setRotVel(msg->num);
	if(!turn_right.isActive()){
	std::cout<<"turn_right" <<std::endl;
      turn_right.activate();
turn_left.deactivate();
drive.deactivate();
    }
	//robot.stop();
	//robot.stopRunning();
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "robot_controller");

  ros::NodeHandle node_handle;
  //ros::param::get("visualizer/height", height);

  ros::Subscriber cont_sub = node_handle.subscribe("master/control", 2, controllerCallback);
  

  Aria::init();
  
  ArArgumentParser parser(&argc, argv);
  parser.addDefaultArgument("-rp /dev/ttyUSB0");
  parser.loadDefaultArguments();

ArRobot robot;

  // ArRobotConnector connects to the robot, get some initial data from it such as type and name,
  // and then loads parameter files for this robot.
  ArRobotConnector robotConnector(&parser, &robot);
  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "simpleMotionCommands: Could not connect to the robot.");
    if(parser.checkHelpAndWarnUnparsed())
    {
        Aria::logOptions();
        Aria::exit(1);
        return 1;
    }
  }
  if (!Aria::parseArgs())
  {
    Aria::logOptions();
    Aria::exit(1);
    return 1;
  }
  
  ArLog::log(ArLog::Normal, "Robot successfully connected.");

    // Sonar for basic obstacle avoidance
  ArSonarDevice sonar;

  

  //ArUtil::sleep(2000);
  
  // Add the sonar to the robot
  robot.addRangeDevice(&sonar);

robot.addAction(&stop, 100);
   robot.addAction(&limiter, 90);
  //robot.addAction(&limiterFar, 80);
robot.addAction(&turn_right,59);
robot.addAction(&turn_left, 59);
  robot.addAction(&drive, 50);
  
turn_right.deactivate();
turn_left.deactivate();
  drive.deactivate();
   stop.deactivate();
	robot.enableMotors();
robot.comInt(ArCommands::SOUNDTOG, 0);
// Start the robot processing cycle running in the background.
  // True parameter means that if the connection is lost, then the 
  // run loop ends.
  robot.runAsync(true);


  ros::spin();
   

  // exit
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Exiting.");
  Aria::exit(0);
  return 0;
}
