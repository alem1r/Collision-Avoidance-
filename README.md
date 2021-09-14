# ROS node Collision Avoidance 

# What

The aim of this ROS node is to locate obstacles through the laser scan and adjust the velocity 
in order to not collide.


# How to run 

* Copy this ROS package in a folder inside the `src` folder of your ROS workspace
* from your workspacke build with `catkin_make`
* run with `rosrun collision_avoidance avoider`


In order to test this node use a stage ros and a controller node like `teleop_twist_keyboard` that publish `Twist` messages. 


First, run the collision_avoidance node with:

```
rosrun rosrun collision_avoidance avoider
```
Second, the controller with:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=cmd_vel_call
```
Now run ros_stage with:

```
rosrun stage_ros stageros <worldfile>
```
In this test we used the DIAG department  map [cappero_laser_odom_diag_obstacle_2020-05-06-16-26-03.world](https://gitlab.com//grisetti/labiagi_2020_21/-/raw/master/workspaces/srrg2_labiagi/src/srrg2_navigation_2d/config/cappero_laser_odom_diag_obstacle_2020-05-06-16-26-03.world?inline=false)




