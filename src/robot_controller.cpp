#include "Aria.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pioneer2/control.h"

ArRobot robot;
 ros::Subscriber cont_sub;

// add Actions for automatic driving
  ArActionLimiterForwards limiter("speed limiter near", 350, 800, 200); // arguments: name, stopDistance mm, slowDownDistance mm, maxSeed mm/sec
  ArActionLimiterForwards limiterFar("speed limiter far", 400, 1250, 300);
  ArActionConstantVelocity drive("drive", 350); // 400mm/sec
  ArActionStop stop("stop");

void controllerCallback(const pioneer2::control::ConstPtr &msg){
  if(msg->msg == "stop_now"){
    drive.deactivate();
    robot.stop();
    robot.stopRunning();
  }else if(msg->msg == "stop"){
    if(drive.isActive()){
      drive.deactivate();
    }
    robot.stop();
  }else if(msg->msg == "drive"){
	std::cout<<"test" <<std::endl;
    if(!drive.isActive()){
      drive.activate();
    }
  }else if(msg->msg == "rotate"){
	robot.setRotVel(msg->num);
	ArUtil::sleep(2000);
	robot.stop();
	robot.stopRunning();
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "robot_controller");

  ros::NodeHandle node_handle;
  //ros::param::get("visualizer/height", height);

  ros::Subscriber cont_sub = node_handle.subscribe("master/robot_control", 2, controllerCallback);
  

  Aria::init();
  
  ArArgumentParser parser(&argc, argv);
  parser.addDefaultArgument("-rp /dev/ttyUSB0");
  parser.loadDefaultArguments();

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

  // Start the robot processing cycle running in the background.
  // True parameter means that if the connection is lost, then the 
  // run loop ends.
  robot.runAsync(true);
	robot.enableMotors();

  ArUtil::sleep(2000);
  
  // Add the sonar to the robot
  robot.addRangeDevice(&sonar);

  robot.addAction(&limiter, 100);
  robot.addAction(&limiterFar, 90);
  robot.addAction(&drive, 60);
  robot.addAction(&stop, 10);

  drive.deactivate();
   
   
  // wait for the thread to stop
  robot.waitForRunExit();

  ros::spin();

  // exit
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Exiting.");
  Aria::exit(0);
  return 0;
}
