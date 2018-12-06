/*

*/

#include <ros/ros.h>
#include <std_msgs/Int16.h>

void message_callback(const std_msgs::Int16::ConstPtr& choice)
{
  bool played = false;
  ROS_INFO("playing message %i!", choice->data);
  switch (choice->data)
  {
    case 1:
    //system("cvlc --play-and-exit /home/c2-19/ros_ws/src/mrrr/betty/sounds/Depression.mp3");
    played = true;
    break;

    default:
    break;
  }
  if(!played)
    ROS_WARN("Couldn't play message: %i, check error message above", choice->data);
}

int main(int argc, char *argv[]) {
  ros::init(argc,  argv, "communication");
  ros::NodeHandle nh;
  ros::Subscriber message_sub = nh.subscribe<std_msgs::Int16>("/mine_explorer/sound_message", 5, message_callback);
  ros::Duration(1.0).sleep();
  ROS_INFO("Done with setup, waiting for message...");


  ros::Rate r(10);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
