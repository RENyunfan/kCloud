#include <iostream>
#include <thread>
#include <chrono>
// #include <mav_msgs/conversions.h>
// #include <mav_msgs/default_topics.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <map>
#include <cmath>

#include <sys/time.h> // 头文件

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <nav_msgs/Odometry.h>


// switch 姿态直接由odom发送
using namespace Eigen;
using namespace sensor_msgs;

ros::Publisher odom_pub;

//订阅gazebo 的话题，订阅一个adap_estimator 话题，进行筛选发布
int flag=0;

nav_msgs::Odometry  ga_Odom;
nav_msgs::Odometry  es_Odom;

void odomcallback(const nav_msgs::Odometry::ConstPtr &ga_msg)
{
   // struct timespec tn,te;
   // clock_gettime(CLOCK_REALTIME, &tn);
   // ga_Odom.header.stamp = ros::Time::now();
   ga_Odom.header.stamp =ga_msg->header.stamp;
   ga_Odom.header.frame_id="world";
   ga_Odom.child_frame_id="firefly/vi_sensor/base_link";
   // set position
   ga_Odom.pose=ga_msg->pose;


  //orientation
   ga_Odom.pose.pose.orientation.x=ga_msg->pose.pose.orientation.x;
   ga_Odom.pose.pose.orientation.y=ga_msg->pose.pose.orientation.y;
   ga_Odom.pose.pose.orientation.z=ga_msg->pose.pose.orientation.z;
   ga_Odom.pose.pose.orientation.w=ga_msg->pose.pose.orientation.w;
  
   ga_Odom.twist=ga_msg->twist;
   // odom_pub.publish(ga_Odom);
   
   // clock_gettime(CLOCK_REALTIME, &te);
   // int unsigned long t=te.tv_nsec-tn.tv_nsec;
   // ROS_INFO("ga_Odom.header:[%f], t_b:[%lu]",ga_msg->header.stamp.toSec(),t);
       

}



void esticallback(const nav_msgs::Odometry::ConstPtr &estimator_odom_msg)
{
 
   es_Odom.header.stamp =estimator_odom_msg->header.stamp;
   es_Odom.header.frame_id="world";
   es_Odom.child_frame_id="firefly/vi_sensor/base_link";
   //POSE
   es_Odom.pose.pose.position.x=estimator_odom_msg->pose.pose.position.x;
   es_Odom.pose.pose.position.y=estimator_odom_msg->pose.pose.position.y;
   es_Odom.pose.pose.position.z=estimator_odom_msg->pose.pose.position.z;
   //orientation
   // es_Odom.pose.pose.orientation.x=estimator_odom_msg->pose.pose.orientation.x;
   // es_Odom.pose.pose.orientation.y=estimator_odom_msg->pose.pose.orientation.y;
   // es_Odom.pose.pose.orientation.z=estimator_odom_msg->pose.pose.orientation.z;
   // es_Odom.pose.pose.orientation.w=estimator_odom_msg->pose.pose.orientation.w;
   // //set_velocity
   // es_Odom.twist.twist.linear.x=estimator_odom_msg->twist.twist.linear.x;
   // es_Odom.twist.twist.linear.y=estimator_odom_msg->twist.twist.linear.y;
   // es_Odom.twist.twist.linear.z=estimator_odom_msg->twist.twist.linear.z;
   // //set the angular
   // es_Odom.twist.twist.angular.x=estimator_odom_msg->twist.twist.angular.x;
   // es_Odom.twist.twist.angular.y=estimator_odom_msg->twist.twist.angular.y;
   // es_Odom.twist.twist.angular.z=estimator_odom_msg->twist.twist.angular.z;
   //flag
   
   //orientation
   es_Odom.pose.pose.orientation.x=ga_Odom.pose.pose.orientation.x;
   es_Odom.pose.pose.orientation.y=ga_Odom.pose.pose.orientation.y;
   es_Odom.pose.pose.orientation.z=ga_Odom.pose.pose.orientation.z;
   es_Odom.pose.pose.orientation.w=ga_Odom.pose.pose.orientation.w;
   
   es_Odom.twist=ga_Odom.twist;
  
   flag=1;
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "switch_node");
    ros::NodeHandle nh("~");
    ros::Subscriber gazebo_odom=nh.subscribe("/firefly/odometry_sensor1/odometry",1,odomcallback);
    ros::Subscriber estimator_odom=nh.subscribe("/odometry_pub",10,esticallback);
    odom_pub=nh.advertise<nav_msgs::Odometry>("/final_odom_pub", 1);
   
    ROS_INFO_STREAM( " Hello , SWITCH! " ) ;
    while(ros::ok())
    { 
        if(flag==0)
        {
          odom_pub.publish(ga_Odom);
        //   ROS_INFO(" estimatar.orientation:x[%f],y[%f],z[%f],w[%f]",ga_Odom.pose.pose.orientation.x ,ga_Odom.pose.pose.orientation.y,ga_Odom.pose.pose.orientation.z,ga_Odom.pose.pose.orientation.w);
       

        }
        else if(flag==1)
        {
        //   ROS_INFO(" estimatar.POSTION:x[%f],y[%f],z[%f]",es_Odom.pose.pose.position.x , es_Odom.pose.pose.position.y,es_Odom.pose.pose.position.z);
          odom_pub.publish(es_Odom);
          // ROS_INFO_STREAM("estimatar pub"); 

        }
          ros::spinOnce();
    }

   //  ros::spin(); 
    return 0;

    }
   


