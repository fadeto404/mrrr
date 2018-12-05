/*

*/

#include <ros/ros.h>
#include <std_msgs/Int16.h>

void music_callback(const std_msgs::Int16::ConstPtr& choice);

int main(int argc, char *argv[]) {
  ros::init(argc,  argv, "explorer_sound");
  ros::NodeHandle nh;
  ros::Subscriber music_sub = nh.subscribe<std_msgs::Int16>("/mine_explorer/sound", 1, music_callback);
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
  switch (choice->data)
  {
    case 1:
    std::cout << "playing music 1!" << std::endl;
    //system("cvlc --play-and-exit /home/c2-19/ros_ws/src/mrrr/betty/sounds/Depression.mp3");
    break;

    default:
    break;
  }

  if(!played)
    std::cout << "Couldn't play song: " << choice->data << ", check error message above" << std::endl;
}
