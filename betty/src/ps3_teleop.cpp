/*
  Node to provide teleop of turtlebot with a ps3 controller.
  includes reactions to bumper and cliff sensors
  Allows for both sounds on the bot, and music on the bot-PC to be played
  Music requires DJ node to be running on the bot-PC
*/
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

//Current speed of robot, 0
float linSpeed = 1;
float angSpeed = 1;

//used for adjusting speed, using ROS parameters. Acts as a multiplier (See the go-function)
float linSpeedMult, angSpeedMult;

//Axes of controller
float axes[6];
//Buttons of controller
uint8_t button[16];
uint8_t backUpTime = 1;

//Prototype functions, defined below main
void go(float linX, float angZ, float s);
void playSound(int sound);
void playMusic(int choice);
void joy_callback(const sensor_msgs::Joy::ConstPtr& joyMsg);
void bumper_callback(const kobuki_msgs::BumperEvent::ConstPtr& bumpMsg);
void cliff_callback(const kobuki_msgs::CliffEvent::ConstPtr& cliffMsg);


int main(int argc, char *argv[]) {

  //Basic initialization of ROS, node_name = "ps3_teleop"
  std::cout << "main, init" << std::endl;
  ros::init(argc, argv, "ps3_teleop");
  ros::NodeHandle nh;

  //Butt-load of publishers and subscribers:
  sound_pub = nh.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 1);
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  music_pub = nh.advertise<std_msgs::Int16>("/music",  2);
  joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, joy_callback);
  bump_sub = nh.subscribe("/mobile_base/events/bumper", 1, bumper_callback);
  cliff_sub = nh.subscribe("/mobile_base/events/cliff", 5, cliff_callback);

  //Checking if speedMult parameters exist, for linear and angular.
  //If not, set them to default values
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
  //Retrieve Axes:
  for (size_t i = 0; i < sizeof(joyMsg->axes)/sizeof(joyMsg->axes[0]); i++) {
    axes[i] = joyMsg->axes[i];
  }

  //Retrieve Buttons:
  for (size_t i = 0; i < sizeof(joyMsg->buttons)/sizeof(joyMsg->buttons[0]); i++) {
    button[i] = joyMsg->buttons[i];
  }

  //Four buttons for sound effects
  for (size_t i = 0; i < 4; i++)
  {
    if(button[i])
    {
      playMusic(i+1); //Music values from 1-4, as such, i+1 is used, as i is 0-3
      std::cout << "sound: " << i << " is playing!" << std::endl;
    }
  }
}


//Following two functions are prototypes, and not used currently
bool buttonXOR(int a)
{
  bool output = true;
  for(int i = 0; i < 17; i++)
  {
      if(i != a)
        if(button[i] == 1)
          output = false;
      if(i == a)
        if(button[i] != 1)
          output = false;
  }
  return output;
}

bool buttonXOR(int a, int b)
{
  bool output = true;
  for(int i = 0; i < sizeof(button)/sizeof(button[0]); i++)
  {
      if(i != a && i != b)
        if(button[i] == true)
          output = false;
      if(i == a || i == b)
        if(button[i] != true)
          output = false;
  }
  return output;
}
