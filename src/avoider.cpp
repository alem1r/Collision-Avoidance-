#include "ros/ros.h"
#include "std_msgs/String.h"

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>


void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  ROS_INFO("Received: [x:%f,y:%f,z:%f]", msg->linear.x, msg->linear.y,msg->linear.z);
}

void laser_cmd_vel_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("Laser: [%s]", msg->header.frame_id.c_str());
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "collision_avoidance");

  
  ros::NodeHandle n;
  //subscribe to cmd_vel topic
  ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel_call", 1000, cmd_vel_callback);
  //subscriber to laser scan topic
  ros::Subscriber laser_scan_sub = n.subscribe("base_scan", 1000, laser_cmd_vel_callback);

   
  
  ros::spin();

  return 0;
}