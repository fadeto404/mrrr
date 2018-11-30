/*
  DJ Node
  Listens for a basic int on the /music topic
  Plays music when correct data is recieved.
  More songs can be added, be exact with file location,
  spaces not allowed in file names.
*/

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

  ros::Rate r(5);
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
<<<<<<< HEAD
    system("cvlc --play-and-exit /home/c2-19/ros_ws/src/mrrr/betty/sounds/Depression.mp3");
=======
    system("cvlc --play-and-exit /home/c2-19/ros_ws/src/mrrr/betty/sounds/SurpriseMotherFcker.mp3");
>>>>>>> 4852cb74f4c8033c6715195201f4d2cb34a99651
    bool played = true;
  }
  if (choice->data == 2)
  {
    std::cout << "playing music 2!" << std::endl;
<<<<<<< HEAD
    system("cvlc --play-and-exit /home/c2-19/ros_ws/src/mrrr/betty/sounds/Hey.mp3");
=======
    system("cvlc --play-and-exit /home/c2-19/ros_ws/src/mrrr/betty/sounds/Illbeback.mp3");
>>>>>>> 4852cb74f4c8033c6715195201f4d2cb34a99651
    bool played = true;
  }
  if (choice->data == 3)
  {
    std::cout << "playing music 3!" << std::endl;
<<<<<<< HEAD
    system("cvlc --play-and-exit /home/c2-19/ros_ws/src/mrrr/betty/sounds/Skiddadle.mp3");
=======
    system("cvlc --play-and-exit /home/c2-19/ros_ws/src/mrrr/betty/sounds/Shutupandtakemymoney!.mp3");
>>>>>>> 4852cb74f4c8033c6715195201f4d2cb34a99651
    bool played = true;
  }
  if (choice->data == 4)
  {
    std::cout << "playing music 4!" << std::endl;
<<<<<<< HEAD
    system("cvlc --play-and-exit /home/c2-19/ros_ws/src/mrrr/betty/sounds/YeahBoy.mp3");
=======
    system("cvlc --play-and-exit /home/c2-19/ros_ws/src/mrrr/betty/sounds/ThisisSparta.mp3");
>>>>>>> 4852cb74f4c8033c6715195201f4d2cb34a99651
    bool played = true;
  }

  if(!played)
  {
    std::cout << "Couldn't play song: " << choice->data << ", maybe a wrong reference was used?" << std::endl;
  }
}
