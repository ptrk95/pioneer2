#include "Aria.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pioneer2/control.h"

ArRobot robot;

void controllerCallback(const pioneer2::control::ConstPtr &msg){
  if(msg->msg == "stop_now"){
    robot.stopRunning();
  }else if(msg->msg == "stop"){
    robot.stop();
  }else if(msg->msg == "drive"){

  }else if(msg->msg == "rotate"){
	robot.setRotVel(msg->num);
	ArUtil::sleep(2000);
	robot.stop();
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "robot_controller");

  ros::NodeHandle node_handle;
  //ros::param::get("visualizer/height", height);

  ros::Subscriber cont_sub = node_handle.subscribe("camera_module/control_robot", 4, controllerCallback);
  

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
  
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Connected.");

  // Start the robot processing cycle running in the background.
  // True parameter means that if the connection is lost, then the 
  // run loop ends.
  robot.runAsync(true);


  // wait for the thread to stop
  robot.waitForRunExit();

  ros::spin();

  // exit
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Exiting.");
  Aria::exit(0);
  return 0;
}
