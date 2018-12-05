// #include "autopilot.h"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>

// Include the explore_lite package
#include <explore/costmap_client.h>
#include <explore/frontier_search.h>
#include <explore/costmap_tools.h>
#include <explore/explore.h>
#include "explore/explore.cpp"
#include "explore/costmap_client.cpp"
#include "explore/frontier_search.cpp"

void start_exploration();

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "explorer_autopilot");
  ros::NodeHandle autopilot_nh;
  ros::Duration(1.0).sleep();

  bool manual_control;

  ros::Rate r(10);
  while (ros::ok)
  {
    autopilot_nh.getParam("/mine_explorer/control_mode", manual_control);
    if (!manual_control){
      start_exploration();
    }

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

void start_exploration(){

  explore::Explore explore;
  std::cout << "test" << std::endl;
  ros::spinOnce();
  return;
}














/*class Autopilot
{
public:
  Autopilot(){
    explore::Explore explorer;
  };
};*/


/*
explore::Explore autopilot::Autopilot()
{
  explore::Explore() explorer;
};

void autopilot::explore_mine() {
  explorer->start();
}

void autopilot::stop_exploring(){
  explorer->stop();
}

*/