#include "dvins_core/arm_plugin.h"

namespace gazebo{

    GazeboArmPlugin::~GazeboArmPlugin()
    {
        event::Events::DisconnectWorldUpdateBegin(updateConnection_);
        if(node_handle_){
            node_handle_->shutdown();
            delete node_handle_;
        }
    }

    void GazeboArmPlugin::InitializeParams(){}

    void GazeboArmPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        model_ = _model;
        world_ = model_->GetWorld();
        arm_controller = new physics::JointController(model_);
        //default params or get params from sdf
        namespace_.clear();
        if(_sdf->HasElement("robotNamespace"))
            namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        else
            gzerr << "[gezebo_arm_plugin] Please specify a robotNamespace. \n";

        if(_sdf->HasElement("jointRot"))
            joint_rot_name_ = _sdf->GetElement("jointRot")->Get<std::string>();
        else
            gzerr << "[gezebo_arm_plugin] Please specify a jointRot. \n";
        joints_[0] = model_->GetJoint(joint_rot_name_);

        if(_sdf->HasElement("jointGspLeft"))
            joint_gspl_name_ = _sdf->GetElement("jointGspLeft")->Get<std::string>();
        else
            gzerr << "[gezebo_arm_plugin] Please specify a jointGspLeft. \n";
        joints_[1] = model_->GetJoint(joint_gspl_name_);

        if(_sdf->HasElement("jointGspRight"))
            joint_gspr_name_ = _sdf->GetElement("jointGspRight")->Get<std::string>();
        else
            gzerr << "[gezebo_arm_plugin] Please specify a jointGspRight. \n";
        joints_[2] = model_->GetJoint(joint_gspr_name_);

        if(_sdf->HasElement("commandArmSubTopic"))
            command_arm_sub_topic_ = _sdf->GetElement("commandArmSubTopic")->Get<std::string>();
        else
            ROS_WARN("[gezebo_arm_plugin] missing <commandArmSubTopic>,default to command_arm_sub_topic");
        if(_sdf->HasElement("odoArmPubTopic"))
            odo_arm_pub_topic_ = _sdf->GetElement("odoArmPubTopic")->Get<std::string>();
        else
            ROS_WARN("[gezebo_arm_plugin] missing <commandArmSubTopic>,default to /analysis/odo_arm");
        if(_sdf->HasElement("errArmPubTopic"))
            err_arm_pub_topic_ = _sdf->GetElement("errArmPubTopic")->Get<std::string>();
        else
            ROS_WARN("[gezebo_arm_plugin] missing <errArmSubTopic>,default to /analysis/err_arm");

        node_handle_ = new ros::NodeHandle(namespace_);
        updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboArmPlugin::OnUpdate, this, _1));

        getSdfParam<std::string>(_sdf, "commandArmPoseTopic", command_arm_sub_topic_, command_arm_sub_topic_);
        cmd_sub = node_handle_->subscribe(command_arm_sub_topic_,10,&GazeboArmPlugin::cmdCallBack, this);
        aly_odo_arm_pub_ = node_handle_->advertise<dvins_core::PoseEuler>(odo_arm_pub_topic_, 100);
        aly_err_arm_pub_ = node_handle_->advertise<dvins_core::PoseEuler>(err_arm_pub_topic_, 100);
    }

    void GazeboArmPlugin::cmdCallBack(const dvins_core::CommandArmServoAngleConstPtr &cmd_msg)
    {
        command_arm_servo_msg.beta = cmd_msg->beta;
        command_arm_servo_msg.grasp_or_not = cmd_msg->grasp_or_not;
    }

    void GazeboArmPlugin::OnUpdate(const common::UpdateInfo & _info)
    {
        curr_time = model_->GetWorld()->GetSimTime();
        sample_time =  curr_time - prev_sim_time;
        prev_sim_time = curr_time;

        double pos_curr, pos_target;
        pos_curr = joints_[0]->GetAngle(0).Radian();
        pos_target = command_arm_servo_msg.beta;
        double  pos_err = pos_curr - pos_target;

        double effort_cmd = jointPid.Update(pos_err,sample_time);
        effort_cmd = effort_cmd > max_effort ? max_effort :
                                               (effort_cmd < -max_effort ? -max_effort : effort_cmd);
//        std::cout << "pos_curr = " << pos_curr << std::endl;
//        std::cout << "pos_target = " << pos_target << std::endl;

        double grasp_angle;
        if(command_arm_servo_msg.grasp_or_not)//if receive grasping command
            grasp_angle = 0;        //rad, the gripper is close
        else
            grasp_angle = 1.57;     //rad, the gripper is open

//        joints_[0]->SetMaxForce(0,1000);
        joints_[0]->SetForce(0,effort_cmd);
//        arm_controller->SetJointPosition(joints_[0],pos_target);
        arm_controller->SetJointPosition(joints_[1],grasp_angle);  // left gripper
        arm_controller->SetJointPosition(joints_[2],grasp_angle);  // right gripper

        aly_odo_arm_.gama = pos_curr;
        aly_err_arm_.gama = pos_target - pos_curr;
        aly_odo_arm_.header.stamp = aly_err_arm_.header.stamp = ros::Time::now();
        aly_odo_arm_pub_.publish(aly_odo_arm_);
        aly_err_arm_pub_.publish(aly_err_arm_);
   }

    GZ_REGISTER_MODEL_PLUGIN(GazeboArmPlugin);
}
