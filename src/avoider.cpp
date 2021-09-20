#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_utils_fd.h"
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

geometry_msgs::Twist vel_received;



void star_cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{ 
  cmdReceived = true; //mi salvo la velocità e aspetto di ricevere un laser scan. i calcoli li faccio solo con il laser scan e con la velocità più recente
  vel_received = *msg;
}

void laser_cmd_vel_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  if (!cmdReceived ) return; // per non processare un comando di velocità più volte
  cmdReceived = false;


  tf::TransformListener listener;
  laser_geometry::LaserProjection projector;
  sensor_msgs::PointCloud cloud;

 //obstacle points from laser scan (respect to laser)
  projector.transformLaserScanToPointCloud("base_laser_link", *msg, cloud, listener);
  
  tf::StampedTransform transform_obstacle;
  //calculate the transform in order to obtain obstacle points respect to the robot
    try{
        listener.waitForTransform("base_footprint", "base_laser_link", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("base_footprint", "base_laser_link", ros::Time(0), transform_obstacle); 

    }
    catch(tf::TransformException &ex){
        ROS_ERROR("%s", ex.what());
       // ros::Duration(1.0).sleep();
        return;
    }
//transform in 2d matrix
Eigen::Isometry2f laser_transform = convertPose2D(transform_obstacle);
Eigen::Vector2f p_min; // obstacle position rispetto al laser

p_min(0) = cloud.points[540].x;
p_min(1) = cloud.points[540].y;
auto obstacle_pos = laser_transform * p_min;
float obstacle_dist = sqrt(obstacle_pos(0)*obstacle_pos(0)+obstacle_pos(1)*obstacle_pos(1));
auto obstacle_pos_actual = laser_transform * p_min;

for(int i=cloud.points.size()/2-200;i<=cloud.points.size()/2+200;i++){
Eigen::Vector2f p;

p(0) = cloud.points[i].x; //la lunghezza di cloudpoints è 1080, la metà è 540 che sta davanti a me. cloud.points.size/2
p(1) = cloud.points[i].y;  

obstacle_pos_actual = laser_transform * p;
float obstacle_dist_actual = sqrt(obstacle_pos_actual(0)*obstacle_pos_actual(0)+obstacle_pos_actual(1)*obstacle_pos_actual(1));

  if(obstacle_dist_actual < obstacle_dist){
    obstacle_dist = obstacle_dist_actual;
    obstacle_pos = obstacle_pos_actual;
  }



}

if(obstacle_dist < 0.5 && vel_received.linear.x > 0 ){
float force_intens = (1.0 / obstacle_dist) * 0.3 ;
float force_x = -(obstacle_pos(0) / obstacle_dist) * force_intens;
float force_y = -(obstacle_pos(1) / obstacle_dist) * force_intens;

geometry_msgs::Twist msg_send;

msg_send.linear.x = vel_received.linear.x + force_x;


msg_send.linear.y = vel_received.linear.y + force_y;
msg_send.linear.z = vel_received.linear.z;

if(obstacle_pos_actual(1) > 0)
  msg_send.angular.z =  - force_intens * 2; 
else if (obstacle_pos_actual(1) < 0)
  msg_send.angular.z =  force_intens * 2; 

ROS_INFO("msg_send.linear.x %f",msg_send.linear.x);


vel.publish(msg_send);

} else{
  vel.publish(vel_received);
}
  
  
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "collision_avoidance");

  
  ros::NodeHandle n;
  //subscribe to cmd_vel topic
  ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel_call", 1, star_cmd_vel_callback);
  //subscriber to laser scan topic
  ros::Subscriber laser_scan_sub = n.subscribe("base_scan", 1000, laser_cmd_vel_callback);

  vel = n.advertise<geometry_msgs::Twist>("cmd_vel",1000); 
  
  ros::spin();

  return 0;
}