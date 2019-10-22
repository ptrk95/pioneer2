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

  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "robot_controller");

  ros::NodeHandle node_handle;
  //ros::param::get("visualizer/height", height);

  ros::Subscriber cont_sub = node_handle.subscribe("master_node/control_robot", 4, controllerCallback);
  

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

  // Print out some data from the SIP.  

  // We must "lock" the ArRobot object
  // before calling its methods, and "unlock" when done, to prevent conflicts
  // with the background thread started by the call to robot.runAsync() above.
  // See the section on threading in the manual for more about this.
  // Make sure you unlock before any sleep() call or any other code that will
  // take some time; if the robot remains locked during that time, then
  // ArRobot's background thread will be blocked and unable to communicate with
  // the robot, call tasks, etc.
  
  robot.lock();
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Rot. Vel=%.2f, Battery=%.2fV",
    robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getRotVel(), robot.getBatteryVoltage());
  robot.unlock();

  // Sleep for 3 seconds.
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Will start driving in 3 seconds...");
  ArUtil::sleep(3000);

  // Set forward velocity to 50 mm/s
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Driving forward at 250 mm/s for 5 sec...");
  robot.lock();
  robot.enableMotors();
  robot.setVel(250);
  robot.unlock();
  ArUtil::sleep(5000);

  ArLog::log(ArLog::Normal, "simpleMotionCommands: Stopping.");
  robot.lock();
  robot.stop();
  robot.unlock();
  ArUtil::sleep(1000);

  ArLog::log(ArLog::Normal, "simpleMotionCommands: Rotating at 10 deg/s for 5 sec...");
  robot.lock();
  robot.setRotVel(10);
  robot.unlock();
  ArUtil::sleep(5000);

  ArLog::log(ArLog::Normal, "simpleMotionCommands: Rotating at -10 deg/s for 10 sec...");
  robot.lock();
  robot.setRotVel(-10);
  robot.unlock();
  ArUtil::sleep(10000);

  ArLog::log(ArLog::Normal, "simpleMotionCommands: Driving forward at 150 mm/s for 5 sec...");
  robot.lock();
  robot.setRotVel(0);
  robot.setVel(150);
  robot.unlock();
  ArUtil::sleep(5000);

  ArLog::log(ArLog::Normal, "simpleMotionCommands: Stopping.");
  robot.lock();
  robot.stop();
  robot.unlock();
  ArUtil::sleep(1000);


  // Other motion command functions include move(), setHeading(),
  // setDeltaHeading().  You can also adjust acceleration and deceleration
  // values used by the robot with setAccel(), setDecel(), setRotAccel(),
  // setRotDecel().  See the ArRobot class documentation for more.

  
  robot.lock();
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Rot. Vel=%.2f, Battery=%.2fV",
    robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getRotVel(), robot.getBatteryVoltage());
  robot.unlock();

  
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Ending robot thread...");
  robot.stopRunning();

  // wait for the thread to stop
  robot.waitForRunExit();

  ros::spin();

  // exit
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Exiting.");
  Aria::exit(0);
  return 0;
}
