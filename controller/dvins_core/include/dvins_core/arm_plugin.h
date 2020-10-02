#ifndef GAZEBO_ARM_PLUGIN_H
#define GAZEBO_ARM_PLUGIN_H

#include <stdio.h>
#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "dvins_core/common.h"
#include "dvins_core/CommandArmServoAngle.h"
///////// for analysis ///////////
#include "dvins_core/PoseEuler.h"

namespace gazebo {

class GazeboArmPlugin : public ModelPlugin{
  public:
    GazeboArmPlugin()
        :ModelPlugin(),
        node_handle_(nullptr),
        prev_sim_time(0.0),
        sample_time(0.001),
        max_effort(10),
        jointPid(0.50, 0.003, 0.001, 100, -100, 100, -100)
  {
//    InitGlogHelper::instance().initGlog();
  }


    virtual ~GazeboArmPlugin();
    virtual void InitializeParams();
    void cmdCallBack(const dvins_core::CommandArmServoAngleConstPtr& cmd_msg);

protected:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void OnUpdate(const common::UpdateInfo& /*info*/);

private:
    std::string namespace_;
    std::string joint_rot_name_;     //name of the rotating joint
    std::string joint_gspl_name_;     //name of the grasping joint1
    std::string joint_gspr_name_;     //name of the grasping joint2
    std::string command_arm_sub_topic_;//name of the command topic

    ros::NodeHandle *node_handle_;
    ros::Subscriber cmd_sub;
    /////////////////// for analysis ///////////////////
    std::string odo_arm_pub_topic_;
    std::string err_arm_pub_topic_;
    ros::Publisher aly_odo_arm_pub_;
    ros::Publisher aly_err_arm_pub_;
    dvins_core::PoseEuler aly_odo_arm_;
    dvins_core::PoseEuler aly_err_arm_;
    ////////////////////////////////////////////////////
    physics::ModelPtr   model_;
    physics::ModelPtr   mav_model1_;
    physics::WorldPtr   world_;
    physics::JointPtr   joints_[3];
    physics::JointController* arm_controller;
    common::PID         jointPid;

    common::Time curr_time;
    common::Time prev_sim_time;
    common::Time sample_time;

    double max_effort;

    dvins_core::CommandArmServoAngle command_arm_servo_msg;
    /// \brief Pointer to the update event connection.
    event::ConnectionPtr updateConnection_;
};
}


#endif // GAZEBO_ARM_PLUGIN_H
