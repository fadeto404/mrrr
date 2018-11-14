/*

  Keyboard teleop, detects BumperEvent, and backs up
  Too many global variables, should me changed to ROS parameters
  
*/


#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "kobuki_msgs/BumperEvent.h"
#include <time.h>


#define KEY_UP 105
#define KEY_DOWN 107
#define KEY_LEFT 106
#define KEY_RIGHT 108
#define KEY_STRAFE_RIGHT 111
#define KEY_STRAFE_LEFT 117
#define KEY_EXIT 113


ros::Publisher cmd_vel_pub;
ros::Subscriber bump_sub;
geometry_msgs::Twist msg;


float linSpeedMult = 1;
float angSpeedMult = 1;
float linSpeed = 0.2;
float angSpeed = 1;


void bumper_callback(const kobuki_msgs::BumperEvent::ConstPtr& bumpMsg)
{
  uint32_t bumpNum = bumpMsg->bumper;;

  if(bumpMsg->state == kobuki_msgs::BumperEvent::PRESSED)
  {
    std::cout << "Pressed bumper: " << bumpNum << std::endl;

    if(bumpNum == 0) //left bumper
    {
      msg.linear.x = linSpeed/(-2);
      msg.angular.z = angSpeed/2;
      cmd_vel_pub.publish(msg);
      ros::Duration(1).sleep();
    }
    else if(bumpNum == 1) //center bumper
    {
      msg.linear.x = linSpeed/(-2);
      msg.angular.z = 0;
      cmd_vel_pub.publish(msg);
      ros::Duration(1).sleep();
    }
    else if(bumpNum == 2) //right bumper
    {
      msg.linear.x = linSpeed/(-2);
      msg.angular.z = angSpeed/(-2);
      cmd_vel_pub.publish(msg);
      ros::Duration(0.5).sleep();
    }
  }

  if(bumpMsg->state == kobuki_msgs::BumperEvent::RELEASED)
  {
    std::cout << "Released bumper: " << bumpNum << std::endl;
  }

}

int main(int argc, char *argv[]) {
  std::cout << "main, init" << std::endl;

  ros::init(argc, argv, "teleop");
  ros::NodeHandle nh;
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  bump_sub = nh.subscribe("/mobile_base/events/bumper", 1, bumper_callback);

  uint8_t c;
  ros::Rate r(10);
  while(ros::ok())
  {
    std::cout << "Control with arrowkeys:" << std::endl;
    std::cin >> c;
    switch(c)
    {
    case KEY_UP:
      msg.linear.x = linSpeed;
      msg.angular.z = 0;
      break;
    case KEY_DOWN:
      msg.linear.x = -linSpeed;
      msg.angular.z = 0;
      break;
    case KEY_LEFT:
      msg.linear.x = 0;
      msg.angular.z = angSpeed;
      break;
    case KEY_RIGHT:
      msg.linear.x = 0;
      msg.angular.z = -angSpeed;
      break;
    case KEY_STRAFE_LEFT:
      msg.linear.x = linSpeed/2;
      msg.angular.z = angSpeed;
      break;
    case KEY_STRAFE_RIGHT:
      msg.linear.x = linSpeed/2;
      msg.angular.z = -angSpeed;
      break;
    case KEY_EXIT:
      ros::shutdown();
    break;
    default:
      msg.linear.x = 0;
      msg.angular.z = 0;
    break;
    }


    cmd_vel_pub.publish(msg);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
