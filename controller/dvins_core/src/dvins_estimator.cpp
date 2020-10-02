#include <iostream>
#include <thread>
#include <chrono>
#include <sys/time.h> 
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dvins_core/ArucoDetectionInImage.h>
#include <dvins_core/ArucoDetectionInImageArray.h>
#include <dvins_core/ArucoDetectionArray.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <map>
#include <cmath>
#include <geometry_msgs/Pose2D.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <nav_msgs/Odometry.h>



using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;

using std::map;
using std::pair;
using std::make_pair;

//在这个版本里是将迭代的特征点进行修改


double last_imu_t=0;
int count_time=0;
bool imu_flag=false;
bool img_flag=false;


ros::Subscriber sub_image;
ros::Publisher esti_pub;

//内参矩阵
// Eigen::Matrix3d K_(1.,2.,3.,4.,5.,6.,7.,8.,9.);
// MatrixXd m = MatrixXd::Random(3,3);
MatrixXd m(2,3);
MatrixXd A(2,3);
MatrixXd B(2,3);
MatrixXd K(3,3);

//系数矩阵
typedef struct adap_coff
{
   MatrixXd A_i;
   MatrixXd B_i;
   MatrixXd W_i;
   MatrixXd H_i;
   Vector2d nominal_error_1;
   Vector2d nominal_visual_error_1;
   MatrixXd B_e;
   MatrixXd W_s;
   MatrixXd B_s;

}A_COFF;



RowVector3d m1(448.1008985853343, 0.0, 376.5);
RowVector3d m2(0.0, 448.1008985853343, 240.5);
RowVector3d m3(0.0, 0.0, 1.0);

//初始化估计器 fife下的world  tf_fifey
//Eigen::Vector3d esti_position(-0.1311,0.0,-0.808);
// world下面position transform

Eigen::Vector3d esti_position(0.0,0.0,1.5);
Eigen::Vector3d esti_velocity(0,0,0);
//cameralink的位置下base link
// left_optical坐标系下baselink
Eigen::Vector3d tran_c_f(0.055,-0.013367,-0.1175);
// baselink 下的letf_optical
Eigen::Vector3d tran_c_b(0.1156,0.0550,-0.0250);
// camera坐标下baselink的旋转
Eigen::Quaterniond rot_c_f(-0.474386,-0.52436,0.52436,-0.474386);
Eigen::Matrix3d rot_b_c=rot_c_f.matrix();
// Eigen::Vector3d tran_c_f(-0.1175,-0.055,0.013367);
// Eigen::Quaterniond rot_c_f(1,0,-0.05,0);



Eigen::Vector3d tran_c1_c(0,0,0);
Eigen::Quaterniond rot_c1_c(-0.5,0.4996,-0.5,0.50038);
Eigen::Quaterniond q_0(1,0,0,0);
// Eigen::Quaterniond q_0(0.707,0,0,0.707);
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
Eigen::Matrix3d R=Eigen::Matrix3d::Identity();
Eigen::Matrix3d R_1 =Eigen::Matrix3d::Identity();
std::vector<sensor_msgs::ImuConstPtr> Imu_Buf;
std::vector<apriltag_ros::AprilTagDetectionArrayConstPtr> Img_Buf;



//map of 更新迭代
map<int,A_COFF>  ID_Coff;

//feature_buf, ID,pose_world,pose_camera
map<int,geometry_msgs::Pose> tag_pose_lib;
map<int,geometry_msgs::Pose> tag_pose_lib_0;
map<int,geometry_msgs::Pose> tag_pose_lib_1;
map<int,geometry_msgs::Pose> tag_pose_lib_2;
//feature buf with t
pair<double, map<int,geometry_msgs::Pose> > tag_pose_t_lib;
pair<double, map<int,geometry_msgs::Pose> > tag_pose_t_lib_0;
pair<double, map<int,geometry_msgs::Pose> > tag_pose_t_lib_1;
pair<double, map<int,geometry_msgs::Pose> > tag_pose_t_lib_2;//直接读取的相机坐标系下的pose
map<int,geometry_msgs::Pose2D> tag_center_lib;
map<int,geometry_msgs::Pose2D> tag_center_lib_0;
map<int,geometry_msgs::Pose2D> tag_center_lib_1;

map<int,geometry_msgs::Pose2D> tag_velocity_lib;


pair<double,map<int,geometry_msgs::Pose2D> > tag_center_t_lib;
pair<double,map<int,geometry_msgs::Pose2D> > tag_center_t_lib_0;
pair<double,map<int,geometry_msgs::Pose2D> > tag_center_t_lib_1;

pair<double,map<int,geometry_msgs::Pose2D> > tag_velocity_t_lib;


 //function
void IMU_measurements(const Imu::ConstPtr &Imu_msg);
std::map<int,geometry_msgs::Pose> TagPosecallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &Img_msg);
std::map<int,geometry_msgs::Pose2D> TagCentercallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &Img_msg);
std::map<int,geometry_msgs::Pose> TagPose_World(std::map<int,geometry_msgs::Pose> &tag_pose_buf,Eigen::Vector3d &trans,Eigen::Quaterniond &q);

Eigen::Isometry3d tf_fireflytoworld(Eigen::Vector3d &esti_position, Eigen::Quaterniond& q);
Eigen::Isometry3d Transform_frame(Eigen::Vector3d &trans,Eigen::Quaterniond &q);   //输入旋转和平移变量，求变换矩阵
void featureupdate(map<int,geometry_msgs::Pose> &tag_pose_lib,map<int,geometry_msgs::Pose2D> &tag_image_center,Eigen::Vector3d &esti_position);
void merge_matrixvo(MatrixXd &A,MatrixXd &B);
void update_estimate(map<int,geometry_msgs::Pose> &tag_pose_lib,map<int,A_COFF> &ID_Coff,Vector3d &esti_p,Vector3d &esti_v);
void print_lib(pair<double,map<int,geometry_msgs::Pose2D> > &tag_center_t_lib_0);
void print_lib_1(pair<double,map<int,geometry_msgs::Pose> > &tag_pose_t_lib_0);

void print_lib_center_notime(map<int,geometry_msgs::Pose2D> &tag_center_lib_0);
void print_lib_pose_notime(map<int,geometry_msgs::Pose>  &tag_pose_lib_0);
//初始化读取img_msg 初始化值

void imgcallback(const dvins_core::ArucoDetectionArray Img_msg)
{
  static bool enter=true;
  geometry_msgs::Pose2D  tag_image_center;
  geometry_msgs::Pose tag_pose;
  geometry_msgs::Pose tag_pose_pix;
  Vector3d tag_pose_pixel_1;
  Vector3d tag_pose_pixel_2;
  map<int,geometry_msgs::Pose2D> tag_center_buf;
  map<int,geometry_msgs::Pose> tag_pose_buf;
  //ID的vector<int>,注意是int 型  
 
  if(enter)
  {
     for(int32_t i=0; i<Img_msg.detection.size();i++)
    {
      
      const int32_t ID = Img_msg.detection[i].id;       
      // ROS_INFO("ID:[%d]",ID);
      
      // ROS_INFO("position[%f,%f]",tag_image_center.x,tag_image_center.y); 
      // 读取相机坐标下的坐标
      tag_pose.position.x=Img_msg.detection[i].pose.pose.pose.position.x;
      tag_pose.position.y=Img_msg.detection[i].pose.pose.pose.position.y;
      tag_pose.position.z=Img_msg.detection[i].pose.pose.pose.position.z;    
      
      tag_pose_pixel_1=Vector3d(tag_pose.position.x,tag_pose.position.y,tag_pose.position.z);
      // 新的tag_center用透视投影关系来完成
      tag_pose_pixel_2=K*tag_pose_pixel_1;
      tag_image_center.x=tag_pose_pixel_2(0)/tag_pose_pixel_2(2);
      tag_image_center.y=tag_pose_pixel_2(1)/tag_pose_pixel_2(2);

     // 根据img 检测到的tag信息
      tag_center_lib_0.insert(make_pair(ID,tag_image_center));  
      tag_pose_buf.insert(make_pair(ID,tag_pose));
      tag_pose_lib_0=TagPose_World(tag_pose_buf,esti_position, q_0); 

     // 初始化tag_center 像素坐标系下，相机坐标系下的位置
      double t= Img_msg.header.stamp.toSec();
      tag_center_t_lib_0=make_pair(t, tag_center_lib_0);  
      tag_pose_t_lib_0=make_pair(t,tag_pose_lib_0); 

    } 

    // 读取到的pose的相机坐标位置，利用相机的初始位置将坐标转换成为世界坐标系下    
    //  tag_pose_lib_0=TagPose_World(tag_pose_buf); 
    // 初始化tag_center 像素坐标系下，相机坐标系下的位置
    //  double t= Img_msg->header.stamp.toSec();
    //  tag_center_t_lib_0=make_pair(t, tag_center_lib_0); 
    //  tag_pose_t_lib_0=make_pair(t,tag_pose_lib_0);  
    //  ROS_INFO("time:[%f],detection_size:[%d]",tag_center_t_lib_0.first,tag_center_t_lib_0.second.size());
    //  ROS_INFO("time:[%f],detection_size:[%d]",tag_pose_t_lib_0.first,tag_pose_t_lib_0.second.size());
    //  print_lib(tag_center_t_lib_0);   
   // 以下是可以正常使用的
   // print_lib_1(tag_pose_t_lib_0);
   // print_lib(tag_center_t_lib_0);
   enter=false;
  }
}


void print_lib_center_notime(map<int,geometry_msgs::Pose2D>  &tag_center_lib_0)
{
    map<int,geometry_msgs::Pose2D>::iterator  map_it_center_1= tag_center_lib_0.begin();
    while(map_it_center_1!=tag_center_lib_0.end())
    {
      //  ROS_INFO("detection id:[%d],position:[%f,%f]",map_it_center_1->first,map_it_center_1->second.x,map_it_center_1->second.y);
       map_it_center_1++;
    }

}


void print_lib_pose_notime(map<int,geometry_msgs::Pose>  &tag_pose_lib_0)
{
    map<int,geometry_msgs::Pose>::iterator  map_it_center_2= tag_pose_lib_0.begin();
    while(map_it_center_2!=tag_pose_lib_0.end())
    {
      //  ROS_INFO("detection id:[%d],inital_position:[%f,%f,%f]",map_it_center_2->first,map_it_center_2->second.position.x,map_it_center_2->second.position.y,map_it_center_2->second.position.z);
       map_it_center_2++;
    }

}


void print_lib(pair<double,map<int,geometry_msgs::Pose2D> > &tag_center_t_lib_0)
{
    map<int,geometry_msgs::Pose2D>::iterator  map_it_center_1= tag_center_t_lib_0.second.begin();
    while(map_it_center_1!=tag_center_t_lib_0.second.end())
    {
      //  ROS_INFO("time:[%f],detection id:[%d],position:[%f,%f]",tag_center_t_lib_0.first,map_it_center_1->first,map_it_center_1->second.x,map_it_center_1->second.y);
       map_it_center_1++;
    }

}


void print_lib_1(pair<double,map<int,geometry_msgs::Pose> > &tag_pose_t_lib_0)
{
    map<int,geometry_msgs::Pose>::iterator  map_it_center_2= tag_pose_t_lib_0.second.begin();
    while(map_it_center_2!=tag_pose_t_lib_0.second.end())
    {
      //  ROS_INFO("time:[%f],detection id:[%d],inital_position:[%f,%f,%f]",tag_pose_t_lib_0.first,map_it_center_2->first,map_it_center_2->second.position.x,map_it_center_2->second.position.y,map_it_center_2->second.position.z);
       map_it_center_2++;
    }

}





void IMU_measurements(const Imu::ConstPtr &Imu_msg )
{
   std_msgs::Header header=Imu_msg->header;
   double dx = Imu_msg->linear_acceleration.x;
   double dy = Imu_msg->linear_acceleration.y;
   double dz = Imu_msg->linear_acceleration.z;

   double rx = Imu_msg->angular_velocity.x;
   double ry = Imu_msg->angular_velocity.y;
   double rz = Imu_msg->angular_velocity.z;
   Eigen::Vector3d angular_velocity{rx,ry,rz};
   // Eigen::Vector3d linear_acceleration{dx,dy,dz};
   Eigen::Vector3d linear_acceleration{0,0,0};
   Eigen::Quaterniond q;

   q.x()=Imu_msg->orientation.x;   //q1
   q.y()=Imu_msg->orientation.y;   //q2
   q.z()=Imu_msg->orientation.z;   //q3
   q.w()=Imu_msg->orientation.w;
   ROS_INFO("imu:Q [%f,%f,%f,%f]",q.x(),q.y(),q.z(),q.w());
   R = Eigen::Matrix3d(q);
   //获取到的是在机体坐标系下，需要转换到世界坐标系下
   // Eigen::Isometry3d T_v= Transform_frame(esti_position, q_0);
   // acc_0=T_v*linear_acceleration;
   // gyr_0=T_v*angular_velocity;
   ROS_INFO("imu is here");
   imu_flag=true;
   // 读取到的是在IMU坐标系下，而不是在baseLINK下的
   acc_0=linear_acceleration; 
   gyr_0=angular_velocity;
   ROS_INFO("Receive the Imu information");
   ROS_INFO("receivee_acc_0 [%f,%f,%f]",acc_0[0],acc_0[1],acc_0[2]);
 }







//从msg添加新的tag_pose tag_center加时间的种类，只读取tag中心center的位置
void TagPoseBufcallback(const dvins_core::ArucoDetectionInImageArray::ConstPtr &Img_msg )
{ 
 
  pair<double,map<int,geometry_msgs::Pose>> TagPosetBuf;
  geometry_msgs::Pose2D   tag_image_center;
  geometry_msgs::Pose   tag_pose;
  geometry_msgs::Pose   tag_pose_pix;
  Vector3d  tag_pose_pixel_1;
  Vector3d  tag_pose_pixel_2;
  map<int,geometry_msgs::Pose> tag_pose_buf;
  map<int,geometry_msgs::Pose2D> tag_center_buf;
  //ID的vector<int>,注意是int 型  
     for(int32_t i=0; i<Img_msg->detection.size();i++)
   {
      //ROS_INFO("i-th detections:[%d],detection_size:[%d]",i,Img_msg->detections.size());
      const int32_t ID = Img_msg->detection[i].id;     
      // tag_image_center.x=Img_msg->detections[i].image_center.x;
      // tag_image_center.y=Img_msg->detections[i].image_center.y;

      // tag_pose.position.x=Img_msg->detection[i].pose.pose.pose.position.x;
      // tag_pose.position.y=Img_msg->detection[i].pose.pose.pose.position.y;
      // tag_pose.position.z=Img_msg->detection[i].pose.pose.pose.position.z;  

      // tag_pose_pixel_1=Vector3d(tag_pose.position.x,tag_pose.position.y,tag_pose.position.z);
      // 新的tag_center用透视投影关系来完成

      // tag_pose_pixel_2=K*tag_pose_pixel_1;
      tag_image_center.x=Img_msg->detection[i].corners[0];
      tag_image_center.y=Img_msg->detection[i].corners[1];
      // ROS_INFO("i-th detections:[%d],tag_image_center:[%f,%f]",ID,tag_image_center.x,tag_image_center.y);
      tag_center_lib_1.insert(make_pair(ID,tag_image_center));
      // tag_pose_buf.insert(make_pair(ID,tag_pose));
      // tag_pose_lib_2是相机坐标系下的位置；
      // tag_pose_lib_2.insert(make_pair(ID,tag_pose));
      // tag_pose_lib_1=TagPose_World(tag_pose_buf,esti_position, q_0); 
      //根据img 检测到的tag信息
     
  } 
   //  ROS_INFO("Receive the tag information");
    double t= Img_msg->header.stamp.toSec();
   //  print_lib(tag_center_t_lib_1);
    tag_center_t_lib_1=make_pair(t, tag_center_lib_1);
    //  print_lib(tag_center_t_lib_1);
    // tag_pose_t_lib_1=make_pair(t,tag_pose_lib_1);
    // tag_pose_t_lib_2=make_pair(t,tag_pose_lib_2);
    // print_lib_1(tag_pose_t_lib_0);
    // print_lib_1(tag_pose_t_lib_2);
    // print_lib(tag_center_t_lib_0);
    // print_lib(tag_center_t_lib_1);
}




//输入tag与相机的位置，输出tag与world的位置
map<int,geometry_msgs::Pose> TagPose_World(map<int,geometry_msgs::Pose> &tag_pose_buf,Eigen::Vector3d &trans,Eigen::Quaterniond &q)
{
   map<int,geometry_msgs::Pose> buf_0;
   map<int,geometry_msgs::Pose>::iterator  map_it = tag_pose_buf.begin();
   geometry_msgs::Pose pose_buf;
   //获取变换矩阵
   //更改tag检测的相机坐标，原本是left_link改成是Z轴朝前的left_opitcal_link,所以无需T_c1_c
   //Eigen::Isometry3d T_c1_c=Transform_frame(tran_c1_c,rot_c1_c);  

   //这两个忘记在哪里出现的了
   // Eigen::Matrix3d T_cf_r=rot_c_f.toRotationMatrix();
   // Eigen::Isometry3d v_word= tf_fireflytoworld(esti_position, q_0);
   Eigen::Isometry3d T_c_f= tf_fireflytoworld(tran_c_f, rot_c_f);
   Eigen::Isometry3d v_word= Transform_frame(trans, q);
   
   // ROS_INFO(" Matrix T_c_f  [%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f]", T_c_f(0,0), T_c_f(0,1), T_c_f(0,2), T_c_f(0,3), T_c_f(1,0), T_c_f(1,1), T_c_f(1,2), T_c_f(1,3),T_c_f(2,0), T_c_f(2,1),T_c_f(2,2),T_c_f(2,3),T_c_f(3,0),T_c_f(3,1),T_c_f(3,2),T_c_f(3,3));
   while(map_it!=tag_pose_buf.end())
   {
      const int32_t ID = map_it->first; 
      // ROS_INFO("ID :[%d] ",map_it->first);
      Eigen::Vector3d v(map_it->second.position.x,map_it->second.position.y,map_it->second.position.z);
      // ROS_INFO("v :[%f %f %f] ",v(0),v(1),v(2));        
      Eigen::Vector3d v_transformed=v_word*T_c_f*v;

      // Eigen::Vector3d _transformed=T_cf_r*(T_c1_c*v)+tran_c_f;
      pose_buf.position.x= v_transformed(0);
      pose_buf.position.y= v_transformed(1);
      pose_buf.position.z= v_transformed(2);
      // ROS_INFO("v_transformed :[%f %f %f] ",v_transformed(0),v_transformed(1),v_transformed(2));
      buf_0.insert(std::make_pair(ID,pose_buf));
      map_it++;
   }    
 return buf_0;
}



//Input pose orientation；Output transformation from firefly to the world
Eigen::Isometry3d tf_fireflytoworld(Eigen::Vector3d &esti_position, Eigen::Quaterniond &q)
{ 
   // Eigen::Vector3d t(pose.position.x,pose.position.y,pose.position.z);
   // ROS_INFO(" Vector [%f,%f,%f]",esti_position[0],esti_position[1],esti_position[2]);
   //欧式变换矩阵
   Eigen::Quaterniond rotation_vector = Eigen::Quaterniond (q);
   Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
   Eigen::Isometry3d T_Inver=Eigen::Isometry3d::Identity();
   T.rotate(rotation_vector);
   T.pretranslate(Eigen::Vector3d(esti_position(0),esti_position(1),esti_position(2)));
   T_Inver=T.matrix().inverse();
   return T_Inver;
   // ROS_INFO(" Matrix [%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f]", T(0,0), T(0,1), T(0,2), T(0,3), T(1,0), T(1,1), T(1,2), T(1,3),T(2,0), T(2,1),T(2,2),T(2,3),T(3,0),T(3,1),T(3,2),T(3,3));
   //ROS_INFO(" Matrix [%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f]", T_Inver(0,0), T_Inver(0,1), T_Inver(0,2), T_Inver(0,3), T_Inver(1,0), T_Inver(1,1), T_Inver(1,2), T_Inver(1,3),T_Inver(2,0), T_Inver(2,1),T_Inver(2,2),T_Inver(2,3),T_Inver(3,0),T_Inver(3,1),T_Inver(3,2),T_Inver(3,3));
}


//Transform
Eigen::Isometry3d Transform_frame(Eigen::Vector3d &trans,Eigen::Quaterniond &q)
{
    Eigen::Isometry3d T_frame=Eigen::Isometry3d::Identity();
    Eigen::Quaterniond rotation_vector = Eigen::Quaterniond (q);
    //定义
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(rotation_vector);
    T.pretranslate(Eigen::Vector3d(trans(0),trans(1),trans(2)));
    T_frame=T.matrix();
    return T_frame;
}

// calculate visual velocity 
void visual_velocity(pair<double,map<int,geometry_msgs::Pose2D> > &tag_center_t_lib_0,pair<double,map<int,geometry_msgs::Pose2D> > &tag_center_t_lib_1)
{
   double t_0=tag_center_t_lib_0.first;
   double t_1=tag_center_t_lib_1.first;
   double delta_t=t_1-t_0;
   // Delta_t_estimate=t_1-t_0;
   double delta_x,delta_y;
   geometry_msgs::Pose2D center_velocity_buf;
   map<int,geometry_msgs::Pose2D>::iterator  map_it_center_0 = tag_center_t_lib_0.second.begin();
   map<int,geometry_msgs::Pose2D>::iterator  map_it_center_1 = tag_center_t_lib_1.second.begin();
   tag_velocity_lib.clear();
   // print_lib(tag_center_t_lib_1);
   // print_lib(tag_center_t_lib_0);
   while(map_it_center_1!=tag_center_t_lib_1.second.end())
  {
       const int32_t ID_1= map_it_center_1->first;
       map<int,geometry_msgs::Pose2D>::iterator it_find;
       it_find=tag_center_t_lib_0.second.find(ID_1);

       if(it_find!=tag_center_t_lib_0.second.end())
          {
              // ROS_INFO("ID:[%d],it_find[%f,%f]",map_it_center_1->first,map_it_center_1->second.x,it_find->second.x);
              // ROS_INFO("Delta_Time:[%f]",delta_t);
             delta_x=((map_it_center_1->second.x)-(it_find->second.x))/delta_t;
             delta_y=((map_it_center_1->second.y)-(it_find->second.y))/delta_t;
             
          }
      else
          {
              delta_x=0.01;  //注意容易出现重定义
              delta_y=0.01;
          } 

      // ROS_INFO("delta_x&delta_y:[%f,%f]",delta_x,delta_y);
      center_velocity_buf.x=delta_x;
      center_velocity_buf.y=delta_y;
      tag_velocity_lib.insert(make_pair(ID_1,center_velocity_buf));
      map_it_center_1++;
  }

 // ROS_INFO("tag_visual_velocity-size:[%d]",tag_velocity_lib.size());
 // print_lib_center_notime(tag_velocity_lib);
}


//derivative rotation matrix,旋转矩阵以及角速度
Matrix3d derive_rota(const Matrix3d &R, const Vector3d &gyr_0)
{
   Matrix3d w =Matrix3d::Identity();
   Matrix3d d_rota =Matrix3d::Identity();
   // double w_1=gyr_0(0);
   // double w_2=gyr_0(1);
   // double w_3=gyr_0(2);
   double w_1=0;
   double w_2=0;
   double w_3=0;
   w<<0,-w_3,w_2,w_3,0,-w_1,-w_2,w_1,0;
   d_rota=R*w;
   // ROS_INFO("w:[%f,%f,%f]",w_1,w_2,w_3);
   return d_rota; 
}


//MATRIX 根据行合并
 void merge_matrixvo(MatrixXd &A,MatrixXd &B)
 {

      MatrixXd T(A.rows(), A.cols()+B.cols());
      T << A, B;
      // ROS_INFO(" Matrix [%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f]", T(0,0), T(0,1), T(0,2), T(0,3), T(0,4), T(0,5), T(1,0), T(1,1),T(1,2), T(1,3),T(1,4),T(1,5));
 }





//进行feature 更替，需要用估计值更替一下特征点的位置；
void featureupdate(map<int,geometry_msgs::Pose> &tag_pose_lib,map<int,geometry_msgs::Pose2D> &tag_image_center,Vector3d &esti_p,Vector3d &esti_v)
{
//   ROS_INFO("featureupdate time");
   Vector2d nominal_error_1;
   Vector2d nominal_visual_error_1;
   Vector2d i_velocity;
   MatrixXd A_i(2,3);
   MatrixXd B_i(2,3);
   MatrixXd W_i(2,3);
   MatrixXd H_i(W_i.rows(), W_i.cols()+B_i.cols());
   Matrix3d d_R;
   A_COFF Coff;
   ID_Coff.clear();
   double error_norm;
   visual_velocity(tag_center_t_lib_0,tag_center_t_lib_1);
   d_R=derive_rota(R,gyr_0);

   double esti_depth;
   map<int,geometry_msgs::Pose2D>::iterator  map_it_center = tag_image_center.begin();
   map<int,geometry_msgs::Pose>::iterator  map_it_pose = tag_pose_lib.begin();

   while(map_it_pose!=tag_pose_lib.end() && map_it_center!=tag_image_center.end())
    {
       
       const int32_t ID = map_it_center->first; 
       Vector2d i_center(map_it_center->second.x,map_it_center->second.y);
       Vector3d i_pose(map_it_pose->second.position.x,map_it_pose->second.position.y,map_it_pose->second.position.z);

       map<int,geometry_msgs::Pose2D>::iterator it_velocity;
       it_velocity=tag_velocity_lib.find(ID);

       if(it_velocity!=tag_velocity_lib.end())
         {
            Vector2d i_velocity_1(it_velocity->second.x,it_velocity->second.y);
            i_velocity=i_velocity_1;
            // ROS_INFO("i_velocity_1:[%f,%f]",i_velocity_1(0),i_velocity_1(1));
          
         }
      else  
         {
            
            // ROS_INFO("Visual velocity id:[%d] is missing",ID);
         }

      //估计中要用的是上一个时刻的值

      //系数矩阵
       A_i=m-i_center*m3; 
       B_i=A_i*rot_b_c*R.transpose();
       W_i=A_i*rot_b_c*d_R.transpose()-i_velocity*m3*rot_b_c*R.transpose();
       H_i<<-W_i,B_i;

      // ROS_INFO("esti_p:[%f,%f,%f]",esti_p(0),esti_p(1),esti_p(2));
       float i_depth=m3*(rot_b_c*R.transpose())*(i_pose-esti_p-R*tran_c_b);
      // float i_depth_act=i_pose(2);
      // float i_depth_esti=m3*i_pose;
      //   ROS_INFO("i_depth:[%f]",i_depth);
      // ROS_INFO("rot_b_c:[%f,%f,%f,%f,%f,%f,%f,%f,%f]",rot_b_c(0,0),rot_b_c(0,1),rot_b_c(0,2),rot_b_c(1,0),rot_b_c(1,1),rot_b_c(1,2),rot_b_c(2,0),rot_b_c(2,1),rot_b_c(2,2));
      //   ROS_INFO("i_velocity:[%f,%f]",i_velocity(0),i_velocity(1));
      //   ROS_INFO("esti_p:[%f,%f,%f]",esti_p(0),esti_p(1),esti_p(2));
      //   ROS_INFO("i_center:[%f,%f]",i_center(0),i_center(1));
      //  ROS_INFO("tran_c_b:[%f,%f,%f]",tran_c_b(0),tran_c_b(1),tran_c_b(2));
   
      //   nominal _ERROR_1
      //   nominal_error_1=i_depth*i_center-m*i_pose;
         nominal_error_1=i_depth*i_center-m*(rot_b_c*R.transpose())*(i_pose-esti_p-R*tran_c_b);
         nominal_visual_error_1=i_depth*i_velocity-A_i*rot_b_c*((d_R.transpose()*(i_pose-esti_p-tran_c_b)-R.transpose()*esti_v));
         // ROS_INFO("ID:[%d] nominal_error:[%f,%f]",ID,nominal_error_1(0),nominal_error_1(1));
         // ROS_INFO("ID:[%d] nominal_visual_error_1:[%f,%f]",ID,nominal_visual_error_1(0),nominal_visual_error_1(1));
      

      //  添加一个判断的语句，就是去除掉误差的二范数大于2的特征点
        // if(nominal_error_1.norm()>=2)
        // {
        //    ROS_INFO("ID:[%d] is out of range",ID);
        //    // map_it_pose=tag_pose_lib_1.erase(map_it_pose);
        //    map_it_pose++;
        //    map_it_center++;
        //   continue;
        // }

      //迭代方程所需要使用的系数
        Coff.A_i=A_i;
        Coff.B_i=B_i;
        Coff.W_i=W_i;
        Coff.H_i=H_i;
        Coff.nominal_error_1=nominal_error_1;
        Coff.nominal_visual_error_1=nominal_visual_error_1;
        Coff.B_e=0.001*B_i.transpose()*nominal_error_1;
      //   ROS_INFO("ID:[%d] B_e:[%f,%f,%f]",ID,Coff.B_e(0),Coff.B_e(1),Coff.B_e(2));
        Coff.W_s=W_i.transpose()*nominal_visual_error_1;
        Coff.B_s=B_i.transpose()*nominal_visual_error_1;
        ID_Coff.insert(make_pair(ID,Coff));
 
       map_it_pose++;
       map_it_center++;

   }

  // 更新迭代
   tag_center_lib_0=tag_center_lib_1;
   // tag_pose_lib_0=tag_pose_lib_1;
   tag_center_t_lib_0=make_pair(tag_center_t_lib_1.first, tag_center_lib_0);
   // tag_pose_t_lib_0=make_pair(tag_pose_t_lib_1.first,tag_pose_lib_1);
   tag_center_lib_1.clear();
   // tag_pose_lib_1.clear();
 }


//update the estimation 

void update_estimate(map<int,geometry_msgs::Pose> &tag_pose_lib,map<int,A_COFF> &ID_Coff,Vector3d &esti_p,Vector3d &esti_v)
{
   Vector3d delt_x;
   Vector3d delt_v;
   Vector3d Co_1;
   Vector3d Co_2;
   Vector3d Co_3;
   Vector3d delt_p;
   geometry_msgs::Pose  tag_pose;
   map<int,geometry_msgs::Pose> tag_pose_buf;
   map<int,A_COFF>::iterator it_CO=ID_Coff.begin();

   while(it_CO!=ID_Coff.end())
   {
      const int32_t ID = it_CO->first; 
      map<int,geometry_msgs::Pose>::iterator it_find;
      it_find=tag_pose_lib.find(ID);
      // ROS_INFO("ID:[%d]",ID);
      if(it_find!=tag_pose_lib.end())
      {

        Vector3d p_posi(it_find->second.position.x,it_find->second.position.y,it_find->second.position.z);
        delt_p=-(it_CO->second.B_e)+(it_CO->second.W_s);
        Vector3d p_posi_up=p_posi+delt_p;
      //   ROS_INFO("p_posi:[%f,%f,%f]",p_posi(0),p_posi(1),p_posi(2));
      //   ROS_INFO("delt_p:[%f,%f,%f]",delt_p(0),delt_p(1),delt_p(2));
      //   ROS_INFO("p_posi_up:[%f,%f,%f]",p_posi_up(0),p_posi_up(1),p_posi_up(2));
        tag_pose.position.x=p_posi_up(0);
        tag_pose.position.y=p_posi_up(1);
        tag_pose.position.z=p_posi_up(2);
        tag_pose_lib_1.insert(make_pair(ID,tag_pose));

      }

      Co_1 +=it_CO->second.B_e;
      Co_2 +=it_CO->second.W_s;
      Co_3 +=it_CO->second.B_s;
      it_CO++;
   }

    delt_x=esti_v-Co_1-Co_2;
   //  ROS_INFO("Co_1:[%f,%f,%f],Co_2:[%f,%f,%f]",Co_1(0),Co_1(1),Co_1(2),Co_2(0),Co_2(1),Co_2(2));
   //  ROS_INFO("delt_x:[%f,%f,%f]",delt_x(0),delt_x(1),delt_x(2));
   //  ROS_INFO("acc_0:[%f,%f,%f]",acc_0(0),acc_0(1),acc_0(2));
   // 这里的期望位置是设定的，而不是读取的期望值
    delt_v=acc_0-Co_3+Vector3d(6,6,6).cwiseProduct(esti_p-Vector3d(0,0,1))+Vector3d(4.7,4.7,4.7).cwiseProduct(esti_v);
    esti_p =esti_p+delt_x;
   // esti_v =esti_v+delt_v;
    esti_v=Vector3d(0,0,0); 
    tag_pose_lib_0=tag_pose_lib_1;
    tag_pose_lib_1.clear();

}




void callback(const sensor_msgs::Imu::ConstPtr &Imu_msg, const dvins_core::ArucoDetectionInImageArray::ConstPtr &Img_msg)
{
  
   // ROS_INFO_STREAM("Hello you are here");
   IMU_measurements(Imu_msg);
   TagPoseBufcallback(Img_msg);
   // visual_velocity(tag_center_t_lib_0,tag_center_t_lib_1);
   // featureupdate(tag_pose_lib_1,tag_center_lib_1,esti_position,esti_velocity);
   featureupdate(tag_pose_lib_0,tag_center_lib_1,esti_position,esti_velocity);
   update_estimate(tag_pose_lib_0,ID_Coff,esti_position,esti_velocity);
   
   nav_msgs::Odometry Odom;
   Odom.header.stamp = ros::Time::now();
   Odom.header.frame_id="world";
   //POSE
   Odom.pose.pose.position.x=esti_position(0);
   Odom.pose.pose.position.y=esti_position(1);
   Odom.pose.pose.position.z=esti_position(2);

   Eigen::Quaterniond qq= Eigen::Quaterniond(R);

   Odom.pose.pose.orientation.x=qq.x();
   Odom.pose.pose.orientation.y=qq.y();
   Odom.pose.pose.orientation.z=qq.z();
   Odom.pose.pose.orientation.w=qq.w();

   Odom.twist.twist.linear.x=esti_velocity(0);
   Odom.twist.twist.linear.y=esti_velocity(1);
   Odom.twist.twist.linear.z=esti_velocity(2);
   Odom.twist.twist.angular.x=gyr_0(0);
   Odom.twist.twist.angular.y=gyr_0(1);
   Odom.twist.twist.angular.z=gyr_0(2);
   esti_pub.publish(Odom);
   // ROS_INFO("esti_position:[%f,%f,%f]",Odom.pose.pose.position.x,Odom.pose.pose.position.y,Odom.pose.pose.position.z);
   // ROS_INFO("Publisher");
}


int main(int argc, char** argv) {
   ros::init(argc, argv, "dvins_estimator");
   ros::NodeHandle n("~");
   m << 448.1008985853343, 0.0, 376.5,0.0, 448.1008985853343, 240.5;
   K<<448.1008985853343, 0.0, 376.5, 0.0, 448.1008985853343, 240.5, 0.0, 0.0, 1.0;
   esti_pub = n.advertise<nav_msgs::Odometry>("/firefly/odometry_pub", 1);
   ros::Subscriber sub_image=n.subscribe("/aruco_detector/aruco_detection",10,imgcallback);
   ROS_INFO_STREAM( " Hello , ROS! " ) ;
   
   //初始的特征点集初始化tag_pose,tag_image_center  
   message_filters::Subscriber<sensor_msgs::Imu> imu_sub(n, "/firefly/imu_body", 1);
   message_filters::Subscriber<dvins_core::ArucoDetectionInImageArray> img_sub(n, "/aruco_detector/aruco_corners", 1);

   typedef sync_policies::ApproximateTime<sensor_msgs::Imu, dvins_core::ArucoDetectionInImageArray> MySyncPolicy;
   // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)，范围可以调整
   Synchronizer<MySyncPolicy> sync(MySyncPolicy(25), imu_sub, img_sub);
   sync.registerCallback(boost::bind(&callback, _1, _2));
   ros::spin(); 

   return 0;
}