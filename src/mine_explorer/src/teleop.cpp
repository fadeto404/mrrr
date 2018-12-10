/*
*/
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>


//Publishers,subscribers and their messages:
ros::Publisher cmd_vel_pub; //Velocity
ros::Publisher message_pub; //Turtlebot PC music
ros::Subscriber joy_sub;  //Joystick


//Axes of controller
float axes[6];


//Prototype functions, defined below main
void playMessage(int choice);
void joy_callback(const sensor_msgs::Joy::ConstPtr& joyMsg);
void speedChange(std::string param, float value, ros::NodeHandle nh);

int main(int argc, char *argv[]) {

  //Basic initialization of ROS, node_name = "ps3_teleop"
  ros::init(argc, argv, "teleop");
  ros::NodeHandle nh;

  //Butt-load of publishers and subscribers:
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
  message_pub = nh.advertise<std_msgs::Int16>("/mine_explorer/sound_message", 2);
  joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, joy_callback);
  ros::Duration(1.0).sleep();

  //Get home position at startup:

  geometry_msgs::Twist velMsg;
  float linSpeed = 0;
  float angSpeed = 0;
  bool manual_control = false;
  ros::Rate r(25);
  while(ros::ok())
  {
    nh.getParam("/mine_explorer/manual_control", manual_control);
    if(manual_control)
    {
      nh.getParam("/mine_explorer/angSpeed", angSpeed);
      nh.getParam("/mine_explorer/linSpeed", linSpeed);
      //Set speed, linear and angular, publish
      velMsg.linear.x = axes[1]*linSpeed;
      velMsg.angular.z = axes[0]*angSpeed;
      cmd_vel_pub.publish(velMsg);
    }
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}


void playMessage(int choice)
{
  std_msgs::Int16 output;
  output.data = choice;
  message_pub.publish(output);
}

void speedChange(std::string param, float value, ros::NodeHandle nh)
{
  float speed;
  nh.getParam(param, speed);
  speed += value;
  nh.setParam(param, speed);
  ROS_INFO_STREAM("Speed changed, " << param << ": " << speed + value);
}

void joy_callback(const sensor_msgs::Joy::ConstPtr& joyMsg)
{
  ros::NodeHandle nh;
  //Retrieve Axes:
  for (size_t i = 0; i < 6; i++) {
    axes[i] = joyMsg->axes[i];
  }

  //Four buttons for sound effects (cross, circle, triangle and square)
  for (size_t i = 0; i < 4; i++)
  {
    if(joyMsg->buttons[i])
    {
      playMessage(i);
      std::cout << "sound message: " << i << " is playing!" << std::endl;
    }
  }

  //Select button
  if (joyMsg->buttons[8]) {
    bool manual_control;
    nh.getParam("/mine_explorer/manual_control", manual_control);
    if(manual_control)
      nh.setParam("/mine_explorer/manual_control", false);
    else
      nh.setParam("/mine_explorer/manual_control", true);
  }
  //Start button
  if (joyMsg->buttons[9])
  {
    bool exploring;
    nh.getParam("/mine_explorer/exploring", exploring);
    if (exploring)
    {
      nh.setParam("/mine_explorer/exploring", false);
      nh.setParam("/mine_explorer/manual_control", true);
    }
    else
    {
      nh.setParam("/mine_explorer/exploring", true);
      nh.setParam("/mine_explorer/manual_control", false);
    }

  }
  //Cross key up
  if(joyMsg->buttons[13])
  {
    speedChange("/mine_explorer/linSpeed", 0.1, nh);
    speedChange("/mine_explorer/angSpeed", 0.2, nh);
  }
  //Cross key down
  if(joyMsg->buttons[14])
  {
    speedChange("/mine_explorer/linSpeed", -0.1, nh);
    speedChange("/mine_explorer/angSpeed", -0.2, nh);
  }
}
