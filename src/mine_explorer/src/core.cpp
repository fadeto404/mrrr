/*

  Core of explorer robot
  Handles decision making and outputs to GUI

*/
#include <ros/ros.h>
#include <ros/console.h>
#include <string>



float initParamFloat(std::string name, float def);
bool init();

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "explorer_core");
  ros::NodeHandle nh;

  if(!init())
  {
    ROS_DEBUG("Failed to init parameters! Exiting...");
    ros::shutdown();
    //Add functionality to exit main without doing anything more!
  }
  ROS_DEBUG("Initialized, proceeding...");



  while(ros::ok())
  {

    ros::spinOnce();
  }


  return 0;
}

float initParamFloat(std::string name, float def)
{
  ros::NodeHandle nh;
  float param;
  if(nh.hasParam(name))
  {
    nh.getParam(name, param);
    return param;
  }
  else
  {
    nh.setParam(name, def);
    return def;
  }
}

bool init()
{
  bool success = true;

  //Init speed parameters
  float linSpeed = initParamFloat("/mine_explorer/linSpeed", 0.4);
  float angSpeed = initParamFloat("/mine_explorer/angSpeed", 1);

  return success;
}
