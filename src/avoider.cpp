#include "ros/ros.h"
#include "std_msgs/String.h"

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

ros::Publisher vel;
bool cmdReceived = false;


void star_cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  ROS_INFO("Received: [x:%f,y:%f,z:%f]", msg->linear.x, msg->linear.y,msg->linear.z);

  cmdReceived = true;
}

void laser_cmd_vel_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  if (!cmdReceived) return;
  cmdReceived = false;

  tf::TransformListener listener;
  laser_geometry::LaserProjection projector;
  sensor_msgs::PointCloud cloud;

  projector.transformLaserScanToPointCloud("base_laser_link",*msg,cloud,listener);

  for(auto& point : cloud.points){
    ROS_INFO("x:%f,y:%f,z:%f",point.x,point.y,point.z);
  }
  
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "collision_avoidance");

  
  ros::NodeHandle n;
  //subscribe to cmd_vel topic
  ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel_call", 1000, star_cmd_vel_callback);
  //subscriber to laser scan topic
  ros::Subscriber laser_scan_sub = n.subscribe("base_scan", 1000, laser_cmd_vel_callback);

  vel = n.advertise<geometry_msgs::Twist>("cmd_vel",1000); 
  
  ros::spin();

  return 0;
}