/*
    ADD a check for manual == true
*/
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>
#include "kobuki_msgs/BumperEvent.h"
#include "kobuki_msgs/CliffEvent.h"


//Publishers,subscribers and their messages:
ros::Publisher cmd_vel_pub; //Velocity
ros::Publisher music_pub; //Turtlebot PC music
ros::Subscriber joy_sub;  //Joystick
ros::Subscriber bump_sub; //Bumper
ros::Subscriber cliff_sub; //Cliff


//Axes of controller
float axes[6];


//Prototype functions, defined below main
void playMusic(int choice);
void joy_callback(const sensor_msgs::Joy::ConstPtr& joyMsg);
void bumper_callback(const kobuki_msgs::BumperEvent::ConstPtr& bumpMsg);
void cliff_callback(const kobuki_msgs::CliffEvent::ConstPtr& cliffMsg);


int main(int argc, char *argv[]) {

  //Basic initialization of ROS, node_name = "ps3_teleop"
  ros::init(argc, argv, "explorer_teleop");
  ros::NodeHandle nh;

  //Butt-load of publishers and subscribers:
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
  music_pub = nh.advertise<std_msgs::Int16>("/mine_explorer/sound", 2);
  joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, joy_callback);
  bump_sub = nh.subscribe("/mobile_base/events/bumper", 1, bumper_callback);
  cliff_sub = nh.subscribe("/mobile_base/events/cliff", 5, cliff_callback);


  ros::Duration(1.0).sleep();

  geometry_msgs::Twist velMsg;
  float linSpeed = 0;
  float angSpeed = 0;

  ros::Rate r(25);
  while(ros::ok())
  {
    nh.getParam("/mine_explorer/angSpeed", angSpeed);
    nh.getParam("/mine_explorer/linSpeed", linSpeed);

    //Set speed, linear and angular, publish
    velMsg.linear.x = axes[1]*linSpeed;
    std::cout << "Axes[1]: " << axes[1] << ", linspeed: " << linSpeed << std::endl;
    velMsg.angular.z = axes[0]*angSpeed;
    cmd_vel_pub.publish(velMsg);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}



void playMusic(int choice)
{
  std_msgs::Int16 output;
  output.data = choice;
  music_pub.publish(output);
}

void bumper_callback(const kobuki_msgs::BumperEvent::ConstPtr& bumpMsg)
{

}

void cliff_callback(const kobuki_msgs::CliffEvent::ConstPtr& cliffMsg)
{

}

void joy_callback(const sensor_msgs::Joy::ConstPtr& joyMsg)
{
  //Retrieve Axes:
  for (size_t i = 0; i < 6; i++) {
    axes[i] = joyMsg->axes[i];
  }

  //Four buttons for sound effects
  for (size_t i = 0; i < 4; i++)
  {
    if(joyMsg->buttons[i])
    {
      playMusic(i+1); //Music values from 1-4, as such, i+1 is used, as i is 0-3
      std::cout << "sound: " << i << " is playing!" << std::endl;
    }
  }
  //Add buttons for changing speed parameters here:

}
