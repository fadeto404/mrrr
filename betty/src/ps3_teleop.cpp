#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>
#include "kobuki_msgs/BumperEvent.h"
#include "kobuki_msgs/CliffEvent.h"
#include "kobuki_msgs/Sound.h"


//Publishers,subscribers and their messages:
ros::Publisher cmd_vel_pub; //Velocity
ros::Publisher sound_pub; //Turtlebot sounds
ros::Publisher music_pub; //Turtlebot PC music
ros::Subscriber joy_sub;  //Joystick
ros::Subscriber bump_sub; //Bumper
ros::Subscriber cliff_sub; //Cliff

//Global messages for publishing
geometry_msgs::Twist velMsg;
kobuki_msgs::Sound soundMsg;


//Global variables, make these to parameters instead!
float linSpeedMult = 1;
float angSpeedMult = 1;
float linSpeed = 0.2;
float angSpeed = 1;

//Axes of controller
float axes[6];
//Buttons of controller
uint8_t button[16];
uint8_t backUpTime = 1;


void go(float linX, float angZ, float s);
void playSound(int sound);
void playMusic(int choice);
void joy_callback(const sensor_msgs::Joy::ConstPtr& joyMsg);
void bumper_callback(const kobuki_msgs::BumperEvent::ConstPtr& bumpMsg);
void cliff_callback(const kobuki_msgs::CliffEvent::ConstPtr& cliffMsg);


int main(int argc, char *argv[]) {
  std::cout << "main, init" << std::endl;

  ros::init(argc, argv, "ps3_teleop");
  ros::NodeHandle nh;
  sound_pub = nh.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 1);
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  music_pub = nh.advertise<std_msgs::Int16>("/music",  2);
  joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, joy_callback);
  bump_sub = nh.subscribe("/mobile_base/events/bumper", 1, bumper_callback);
  cliff_sub = nh.subscribe("/mobile_base/events/cliff", 5, cliff_callback);

  if(nh.hasParam("linSpeedMult"))
    nh.getParam("linSpeedMult", linSpeedMult);
  else
    nh.setParam("linSpeedMult", 1);
  nh.getParam("linSpeedMult", linSpeedMult);

  if(nh.hasParam("angSpeedMult"))
    nh.getParam("angSpeedMult", angSpeedMult);
  else
    nh.setParam("angSpeedMult", 1);
  nh.getParam("angSpeedMult", angSpeedMult);

  ros::Duration(1.0).sleep();

  playMusic(1);

  ros::Rate r(10);
  while(ros::ok())
  {
    nh.getParam("linSpeedMult", linSpeedMult);
    nh.getParam("angSpeedMult", angSpeedMult);

    velMsg.linear.x = axes[1]*linSpeed*linSpeedMult;
    velMsg.angular.z = axes[0]*angSpeed*angSpeedMult;
    cmd_vel_pub.publish(velMsg);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

void go(float linX, float angZ, float s)
{
  velMsg.linear.x = linX*linSpeedMult;
  velMsg.angular.z = angZ*angSpeedMult;
  for (size_t i = 0; i < s*2; i++) {
    cmd_vel_pub.publish(velMsg);
    ros::Duration(0.5).sleep();
  }
}

void playSound(int sound)
{
  soundMsg.value = sound;
  sound_pub.publish(soundMsg);
}

void playMusic(int choice)
{
  std_msgs::Int16 output;
  output.data = choice;
  music_pub.publish(output);
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


void joy_callback(const sensor_msgs::Joy::ConstPtr& joyMsg)
{
  //Save Axes info
  for (size_t i = 0; i < sizeof(joyMsg->axes)/sizeof(joyMsg->axes[0]); i++) {
    axes[i] = joyMsg->axes[i];
  }

  //Retrieve Buttons:
  for (size_t i = 0; i < sizeof(joyMsg->buttons)/sizeof(joyMsg->buttons[0]); i++) {
    button[i] = joyMsg->buttons[i];
  }
  if(button[0])
    playMusic(1);

}
