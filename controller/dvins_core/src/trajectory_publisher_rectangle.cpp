/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <cmath>
// use rectangle  line
double pi=3.14159;
double norm(double x)
{
  while(x>=2*pi)
    x -= 2*pi;
  while(x<0)
    x += 2*pi;
  return x;
}


int main(int argc, char** argv)
 {
  ros::init(argc, argv, "trajectory_publisher");
  ros::NodeHandle nh;
   // ros::Publisher trajectory_pub =nh.advertise<geometry_msgs::PoseStamped>(mav_msgs::default_topics::COMMAND_POSE, 10);
  ros::Publisher trajectory_pub =nh.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose", 10);
  ros::Publisher setpath_pub =nh.advertise<nav_msgs::Path>("traject", 10);

  ROS_INFO("Started trajectory_publisher.");

  ros::Rate loop_rate(10);
  double delay=1;
  double dt = 0;
  double dtttt=0;
  ROS_INFO("The dt is: %lf",dt);
  geometry_msgs::PoseStamped  geometry_msg;
  geometry_msg.header.stamp=ros::Time::now();
  geometry_msg.header.frame_id= "world";
  
  geometry_msgs::PoseStamped  geometry_msg_1;
  geometry_msg_1.header.stamp=ros::Time::now();
  geometry_msg_1.header.frame_id= "world";


  nav_msgs::Path path;
  path.header.stamp =ros::Time::now();
  path.header.frame_id="world";
 
 
  ros::Duration(delay).sleep();
  ros::Time start_time, current_time,last_time;
  last_time= ros::Time::now();
  start_time = ros::Time::now();
  double vx=0.1;
  double vy=0.1;
  double x=0.0;
  double y=0.0;
  double z=1.0;
  double theta = 0.0;

  int stage = 1;
  double l = 17.5;
  double lm = -1;
  double w = 8.2;
  double wm = 0;
  double h =1.5;
  geometry_msgs::Quaternion goal_quat;
//直线轨迹

  


  while(ros::ok())
  { 
    ROS_INFO("The dt is: %lf",dt);
    current_time = ros::Time::now();
    dt =(current_time -start_time).toSec();
    dtttt= (current_time - last_time).toSec();
    ROS_INFO("The dtttt is: %lf",dtttt);


    // rectangle line
    switch (stage)
    {
    case 1:
      x += dtttt * 0.5;
      goal_quat = tf::createQuaternionMsgFromYaw(0.0);
      geometry_msg.pose.position.x = x;  
      geometry_msg.pose.position.y = wm;
      geometry_msg.pose.position.z = h;
      geometry_msg.pose.orientation.x = goal_quat.x;
      geometry_msg.pose.orientation.y = goal_quat.y;
      geometry_msg.pose.orientation.z = goal_quat.z;
      geometry_msg.pose.orientation.w = goal_quat.w;      
      if(fabs(x-l) <= 0.1)
        stage = 2;
      break;
    
    case 2:
      theta += 0.1;
      goal_quat = tf::createQuaternionMsgFromYaw(theta);
      geometry_msg.pose.position.x = x;  
      geometry_msg.pose.position.y = y;
      geometry_msg.pose.position.z = h;
      geometry_msg.pose.orientation.x = goal_quat.x;
      geometry_msg.pose.orientation.y = goal_quat.y;
      geometry_msg.pose.orientation.z = goal_quat.z;
      geometry_msg.pose.orientation.w = goal_quat.w;
      if(fabs(norm(theta-0.5*pi)) <= 0.1)
        stage = 3;
      break;
    
    case 3:
      y += dtttt * 0.5;
      goal_quat = tf::createQuaternionMsgFromYaw(0.5*pi);
      geometry_msg.pose.position.x = l;  
      geometry_msg.pose.position.y = y;
      geometry_msg.pose.position.z = h;
      geometry_msg.pose.orientation.x = goal_quat.x;
      geometry_msg.pose.orientation.y = goal_quat.y;
      geometry_msg.pose.orientation.z = goal_quat.z;
      geometry_msg.pose.orientation.w = goal_quat.w;
      if(fabs(y-w) <= 0.1)
        stage = 4;
      break;
    
    case 4:
      theta += 0.1;
      goal_quat = tf::createQuaternionMsgFromYaw(theta);
      geometry_msg.pose.position.x = x;  
      geometry_msg.pose.position.y = y;
      geometry_msg.pose.position.z = h;
      geometry_msg.pose.orientation.x = goal_quat.x;
      geometry_msg.pose.orientation.y = goal_quat.y;
      geometry_msg.pose.orientation.z = goal_quat.z;
      geometry_msg.pose.orientation.w = goal_quat.w;
      if(fabs(norm(theta-pi)) <= 0.1)
        stage = 5;
      break;

    case 5:
      x -= dtttt * 0.5;
      goal_quat = tf::createQuaternionMsgFromYaw(pi);
      geometry_msg.pose.position.x = x;  
      geometry_msg.pose.position.y = w;
      geometry_msg.pose.position.z = h;
      geometry_msg.pose.orientation.x = goal_quat.x;
      geometry_msg.pose.orientation.y = goal_quat.y;
      geometry_msg.pose.orientation.z = goal_quat.z;
      geometry_msg.pose.orientation.w = goal_quat.w;
      if(fabs(x-lm) <= 0.1)
        stage = 6;
      break;

    case 6:
      theta += 0.1;
      goal_quat = tf::createQuaternionMsgFromYaw(theta);
      geometry_msg.pose.position.x = x;  
      geometry_msg.pose.position.y = y;
      geometry_msg.pose.position.z = h;
      geometry_msg.pose.orientation.x = goal_quat.x;
      geometry_msg.pose.orientation.y = goal_quat.y;
      geometry_msg.pose.orientation.z = goal_quat.z;
      geometry_msg.pose.orientation.w = goal_quat.w;
      if(fabs(norm(theta-1.5 * pi)) <= 0.1)
        stage = 7;
      break;

    case 7:
      y -= dtttt * 0.5;
      goal_quat = tf::createQuaternionMsgFromYaw(1.5*pi);
      geometry_msg.pose.position.x = lm;  
      geometry_msg.pose.position.y = y;
      geometry_msg.pose.position.z = h;
      geometry_msg.pose.orientation.x = goal_quat.x;
      geometry_msg.pose.orientation.y = goal_quat.y;
      geometry_msg.pose.orientation.z = goal_quat.z;
      geometry_msg.pose.orientation.w = goal_quat.w;
      if(fabs(y-wm) <= 0.1)
        stage = 8;
      break;

    case 8:
      theta += 0.1;
      goal_quat = tf::createQuaternionMsgFromYaw(theta);
      geometry_msg.pose.position.x = x;  
      geometry_msg.pose.position.y = y;
      geometry_msg.pose.position.z = h;
      geometry_msg.pose.orientation.x = goal_quat.x;
      geometry_msg.pose.orientation.y = goal_quat.y;
      geometry_msg.pose.orientation.z = goal_quat.z;
      geometry_msg.pose.orientation.w = goal_quat.w;
      if(fabs(norm(theta-2 * pi)) <= 0.1)
        stage = 1;
      break;

    default:
      ROS_ERROR("YOU ARE IN A WRONG CONDITION, INTROSPECT YOURSELF");
      break;
    }
    ROS_INFO("The position x: %lf",geometry_msg.pose.position.x );
    ROS_INFO("The position y: %lf",geometry_msg.pose.position.y );
    ROS_INFO("The position z: %lf",geometry_msg.pose.position.z );
    path.poses.push_back(geometry_msg);
    trajectory_pub.publish(geometry_msg);
    setpath_pub.publish(path);
    ros::spinOnce();
    last_time = current_time;
    loop_rate.sleep();
  }

//    double x = 0.0;
//    double y = 0.0;
//    double th = 0.0;
//    double vx = 0.1;
//     double vy = -0.1;
//    double vth = 0.1;

// // //轨迹为圆
// while (ros::ok())
//     {

//         current_time = ros::Time::now();
//         double dt = (current_time - last_time).toSec();
//         double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
//         double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
//         double delta_th = vth * dt;

//         x += delta_x;
//         y += delta_y;
//         th += delta_th;

//         geometry_msg.pose.position.x = x*0.6;
//         geometry_msg.pose.position.y = y*0.6;
//         geometry_msg.pose.position.z = dt/10;
        
//         geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th);
//         geometry_msg.pose.orientation.x = goal_quat.x;
//         geometry_msg.pose.orientation.y = goal_quat.y;
//         geometry_msg.pose.orientation.z = goal_quat.z;
//         geometry_msg.pose.orientation.w = goal_quat.w;

//         path.poses.push_back(geometry_msg);
//         trajectory_pub.publish(geometry_msg);
//         setpath_pub.publish(path);
//         ros::spinOnce();               // check for incoming messages

//         last_time = current_time;
//         loop_rate.sleep();
//     }


return 0;
 }
