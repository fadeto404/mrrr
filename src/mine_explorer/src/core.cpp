/*

  Core of explorer robot
  Handles decision making and outputs to GUI

*/
#include <ros/ros.h>
#include <string>

void initParam(string name, int value);


int main(int argc, char *argv[]) {

  ros::init(arc, argv, "explorer_core");
  ros::NodeHandle nh;

  int linSpeedMult = initParamInt("linSpeedMult", 1);


  return 0;
}


int initParamInt(string name, int default)
{
  int param;
  if(nh.hasParam(name))
  {
    nh.getParam(name, param);
    return param;
  }
  else
  {
    nh.setParam(name, default);
    return default;
  }
}
