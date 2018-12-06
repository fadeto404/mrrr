/*
    ADD a check for manual == true
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


int main(int argc, char *argv[]) {

  //Basic initialization of ROS, node_name = "ps3_teleop"
  ros::init(argc, argv, "teleop");
  ros::NodeHandle nh;

  //Butt-load of publishers and subscribers:
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
  message_pub = nh.advertise<std_msgs::Int16>("/mine_explorer/sound_message", 2);
  joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, joy_callback);
  ros::Duration(1.0).sleep();

  geometry_msgs::Twist velMsg;
  float linSpeed = 0;
  float angSpeed = 0;
  bool manual_control = false;
  ros::Rate r(25);
  while(ros::ok())
  {
    nh.getParam("/mine_explorer/control_mode", manual_control);
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
      playMessage(i+1); //Music values from 1-4, as such, i+1 is used, as i is 0-3
      std::cout << "sound: " << i << " is playing!" << std::endl;
    }
  }

  if (joyMsg->buttons[8])
  {
    ros::NodeHandle nh;

    bool manual_control;
    nh.getParam("/mine_explorer/control_mode", manual_control);
    if (manual_control)
      manual_control = false;
    else
      manual_control = true;

    nh.setParam("/mine_explorer/control_mode", manual_control);
  }

  //Add buttons for changing speed parameters here:

}
