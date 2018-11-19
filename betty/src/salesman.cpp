#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher music_pub; //Turtlebot PC music

void playMusic(int choice)
{
  std_msgs::Int16 output;
  output.data = choice;
  music_pub.publish(output);
}

class Route
{
private:
    uint8_t stops_initialized;
    uint8_t nextGoal = 0;
    geometry_msgs::PointStamped   stops[10] ;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client;
    ros::Publisher marker_pub;
    ros::Subscriber click_sub;

    void _send_goal(const geometry_msgs::PointStamped& goal_point)
    {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = goal_point.header.frame_id;
        goal.target_pose.pose.position = goal_point.point;
        goal.target_pose.pose.orientation.w = 1.0;
        client.sendGoal(goal,
            boost::bind(&Route::_target_reached_cb, this, _1, _2));
        //_send_markers();
    }

    void _target_reached_cb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
    {
        if(stops_initialized-1 > nextGoal)
          nextGoal++;
        else
          nextGoal = 0;

        playMusic(1);
        _send_goal(stops[nextGoal]);
    }

    void _send_markers()
    {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.ns = "bus_stops";
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 1.0;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0.7071;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 0.7071;
        marker.lifetime = ros::Duration();


        visualization_msgs::MarkerArray marker_array;
        for (size_t i = 0; i < stops_initialized; i++) {
          marker.header.frame_id = stops[i].header.frame_id;
          marker.id = 0;
          marker.pose.position = stops[i].point;
          marker.pose.position.z += marker.scale.x;
          marker_array.markers.push_back(marker);
        }

      }

    void _clicked_point_cb(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        ROS_INFO("Clicked: %f, %f, %f", msg->point.x,
            msg->point.y, msg->point.z);

        stops[stops_initialized] = *msg;

        stops_initialized++;

        _send_goal(stops[nextGoal]);
    }

public:
    Route() :
        stops_initialized(0), client("move_base")
    {
        ros::NodeHandle n;
        marker_pub = n.advertise<visualization_msgs::MarkerArray>(
            "busroute_markers", 1);
        click_sub = n.subscribe( "clicked_point", 100,
            &Route::_clicked_point_cb, this);
        music_pub = n.advertise<std_msgs::Int16>("/music",  2);

    };
    ~Route(){};


};

// This is where we start
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "busroute");

    Route r;

    ros::spin();
    return 0;
}
