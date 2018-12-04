/*
  Core of explorer robot
  Handles decision making and outputs to GUI
  Handles Gas sensor readings
*/

#include <ros/ros.h>
#include <ros/console.h>
#include <string.h>

//Used by GasHandle::
#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <stdlib.h>
#include <time.h>


class GasHandle{
  int numOfReadings = 0;
  std::string name;
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray marker_array;
  ros::Publisher marker_pub;
  ros::Subscriber joy_sub;
  ros::NodeHandle nh;

  void joy_callback(const sensor_msgs::Joy::ConstPtr& joyMsg)
  {
    //Only do stuff if a gas reading should be taken
    if (joyMsg->buttons[5]) {
      ROS_INFO("Adding gas reading...");

      //Get the robot position, as this will be the position of the gas reading:
      geometry_msgs::TransformStamped transformStamped;
      tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener tfListener(tfBuffer);
      try
      {
        transformStamped = tfBuffer.lookupTransform("map", "base_footprint", ros::Time(0), ros::Duration(5)); //Original: lookupTransform("base_footprint", "map",...
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s", ex.what());
      }

      //Set the marker, with specific data to this reading
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = name + "_readings";
      marker.id = numOfReadings;
      numOfReadings++;

      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;

      //Position is that of the robot
      marker.pose.position.x = transformStamped.transform.translation.x;
      marker.pose.position.y = transformStamped.transform.translation.y;
      marker.pose.position.z = transformStamped.transform.translation.z+0.1;
      //Rotation is origin:
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // Set the scale of the marker, here it is 0.3m x 0.3m x 0.2m
      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.2;

      // Calculating the color as a gradient between blue and red:
      float value = (rand() % 255);

      marker.color.r = value/256;;
      marker.color.g = 0;
      marker.color.b = 1 - marker.color.r;
      marker.color.a = 1;

      //Set lifetime to infinite, and push it into the MarkerArray
      marker.lifetime = ros::Duration();
      marker_array.markers.push_back(marker);

      //New reading added, refresh rviz by publishing MarkerArray
      marker_pub.publish(marker_array);
    }
  }

public:

  GasHandle(std::string nameSet)
  {
    name = nameSet;
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>(name + "_markers", 1);
    joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 100, &GasHandle::joy_callback, this);
    srand (time(NULL));
  }
};

//function prototypes
float initParamFloat(std::string name, float def);
bool init();



int main(int argc, char *argv[]) {

  ros::init(argc, argv, "explorer_core");
  ros::NodeHandle nh;
  GasHandle oxygen("oxygen");
  GasHandle co("carbon_monoxide");

  if(!init())
  {
    ROS_DEBUG("Failed to init parameters! Exiting...");
    ros::shutdown();
    //Add functionality to exit main without doing anything more!
  }
  ROS_DEBUG("Initialized, proceeding...");

  //Run loop at 50Hz:
  ros::Rate r(50);
  while(ros::ok())
  {
    //Process callback queue
    ros::spinOnce();
    
    //Loop at 50 Hz
    r.sleep();
  }

  return 0;
}

float initParamFloat(std::string name, float def)
{
  ros::NodeHandle nh;
  float param;
  if(nh.hasParam(name))
  {
    nh.getParam(name, param);
    return param;
  }
  else
  {
    nh.setParam(name, def);
    return def;
  }
}

bool init()
{
  bool success = true;

  //Init speed parameters
  float linSpeed = initParamFloat("/mine_explorer/linSpeed", 0.4);
  float angSpeed = initParamFloat("/mine_explorer/angSpeed", 1);

  return success;
}
