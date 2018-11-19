#include "ros/ros.h"
#include <iostream>
#include "std_msgs/Int16.h"

void music_callback(const std_msgs::Int16::ConstPtr& choice);

int main(int argc, char *argv[]) {
  ros::init(argc,  argv, "dj");
  ros::NodeHandle nh;
  ros::Subscriber music_sub = nh.subscribe<std_msgs::Int16>("/music", 1, music_callback);
  ros::Duration(1.0).sleep();
  std::cout << "Done with setup, waiting for music request" << std::endl;

  ros::Rate r(1);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}

void music_callback(const std_msgs::Int16::ConstPtr& choice)
{
  bool played = false;
  if (choice->data == 1)
  {
    std::cout << "playing music 1!" << std::endl;
    system("cvlc --play-and-exit /home/c2-19/ros_ws/src/betty/sounds/SurpriseMotherFcker.mp3");
    bool played = true;
  }
  if (choice->data == 2)
  {
    std::cout << "playing music 2!" << std::endl;
    system("cvlc --play-and-exit /home/c2-19/ros_ws/src/betty/sounds/I'll be back.mp3");
    bool played = true;
  }
  if (choice->data == 3)
  {
    std::cout << "playing music 3!" << std::endl;
    system("cvlc --play-and-exit /home/c2-19/ros_ws/src/betty/sounds/Shut up and take my money!.mp3");
    bool played = true;
  }
  if (choice->data == 4)
  {
    std::cout << "playing music 4!" << std::endl;
    system("cvlc --play-and-exit /home/c2-19/ros_ws/src/betty/sounds/This is Sparta.mp3");
    bool played = true;
  }

  if(!played)
  {
    std::cout << "Couldn't play sang: " << choice->data << ", maybe a wrong reference was used?" << std::endl;
  }
}
