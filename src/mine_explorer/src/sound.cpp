/*

*/

#include <ros/ros.h>
#include <std_msgs/Int16.h>

void message_callback(const std_msgs::Int16::ConstPtr& choice)
{
  ROS_INFO("playing message %i!", choice->data);
  switch (choice->data)
  {
    case 0:
    system("cvlc --play-and-exit /home/c2-19/ros_ws/src/mrrr/src/mine_explorer/sounds/message0.mp3");
    break;
    case 1:
    system("cvlc --play-and-exit /home/c2-19/ros_ws/src/mrrr/src/mine_explorer/sounds/message1.mp3");
    break;
    case 2:
    system("cvlc --play-and-exit /home/c2-19/ros_ws/src/mrrr/src/mine_explorer/sounds/message2.mp3");
    break;
    case 3:
    system("cvlc --play-and-exit /home/c2-19/ros_ws/src/mrrr/src/mine_explorer/sounds/message3.mp3");
    break;

    default:
    ROS_WARN("Failed to play message!");
    break;
  }
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
