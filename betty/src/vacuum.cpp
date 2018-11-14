/*

Vacuum node, testing cliff and bumper sensors,
messy code

*/

#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "kobuki_msgs/BumperEvent.h"
#include "kobuki_msgs/CliffEvent.h"
#include "kobuki_msgs/Sound.h"
#include <time.h>


#define KEY_UP 105
#define KEY_DOWN 107
#define KEY_LEFT 106
#define KEY_RIGHT 108
#define KEY_STRAFE_RIGHT 111
#define KEY_STRAFE_LEFT 117


ros::Publisher cmd_vel_pub;
ros::Publisher sound_pub;
ros::Subscriber bump_sub;
ros::Subscriber cliff_sub;
geometry_msgs::Twist msg;
kobuki_msgs::Sound soundMsg;


float linSpeedMult = 1;
float angSpeedMult = 1;
float linSpeed = 0.2;
float angSpeed = 1;
uint8_t backUpTime = 1;

void playSound(int sound)
{
  soundMsg.value = sound;
  sound_pub.publish(soundMsg);
}

void go(float linX, float angZ, float s)
{
  msg.linear.x = linX;
  msg.angular.z = angZ;
  for (size_t i = 0; i < s*2; i++) {
    cmd_vel_pub.publish(msg);
    ros::Duration(0.5).sleep();
  }
}

void bumper_callback(const kobuki_msgs::BumperEvent::ConstPtr& bumpMsg)
{
  uint8_t bumpNum = bumpMsg->bumper;;

  if(bumpMsg->state == kobuki_msgs::BumperEvent::PRESSED)
  {
    playSound(2);
    if(bumpNum == 0) //left bumper
    {
      std::cout << "Pressed left bumper" << std::endl;
      go(-linSpeed/2, -angSpeed/2, backUpTime);
    }
    else if(bumpNum == 1) //center bumper
    {
      std::cout << "Pressed center bumper"<< std::endl;
      go(-linSpeed/2, angSpeed/2, backUpTime*2);
    }
    else if(bumpNum == 2) //right bumper
    {
      std::cout << "Pressed right bumper" << std::endl;
      go(-linSpeed/2, angSpeed/2, backUpTime);
    }
  }

  if(bumpMsg->state == kobuki_msgs::BumperEvent::RELEASED)
  {
    std::cout << "Released bumper: " << bumpNum << std::endl;
  }

}

void cliff_callback(const kobuki_msgs::CliffEvent::ConstPtr& cliffMsg)
{
  if(cliffMsg->state == 1)
  {
    playSound(2);
    if(cliffMsg->sensor == 0) //Left sensor
    {
      std::cout << "Cliff on the left!" << std::endl;
      go(-linSpeed/2, -angSpeed/2, backUpTime);
    }
    else if(cliffMsg->sensor == 1) //Center sensor
    {
      std::cout << "Cliff in front!" << std::endl;
      go(-linSpeed/2, -angSpeed/2, backUpTime*3);
    }
    else if(cliffMsg->sensor == 2) //Right sensor
    {
      std::cout << "Cliff on the right!" << std::endl;
      go(-linSpeed/2, angSpeed/2, backUpTime);
    }
  }
}

int main(int argc, char *argv[]) {
  std::cout << "main, init" << std::endl;

  ros::init(argc, argv, "teleop");
  ros::NodeHandle nh;
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  bump_sub = nh.subscribe("/mobile_base/events/bumper", 1, bumper_callback);
  cliff_sub = nh.subscribe("/mobile_base/events/cliff", 1, cliff_callback);
  sound_pub = nh.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 1);

  //Wail till master has established all nodes:
  ros::Duration(1.0).sleep();

  //play init sound:
  playSound(6);
  //Loop rate at 10HZ
  ros::Rate r(10);
  while(ros::ok())
  {
    //Go in a straigth line

    msg.linear.x = linSpeed/2;
    msg.angular.z = 0;
    cmd_vel_pub.publish(msg);
    ros::spinOnce();
    //Wait, in order to achieve 10Hz command frequency
    r.sleep();
  }
  return 0;
}
