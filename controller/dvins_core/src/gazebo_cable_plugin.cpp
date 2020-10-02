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


#include "dvins_core/gazebo_cable_plugin.h"

namespace gazebo {

GazeboCableModel::~GazeboCableModel() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboCableModel::InitializeParams() {}

void GazeboCableModel::CreateCablePayloadJoint() {

	if(created_)
		return;

	math::Pose _tmp_pose; //add a small offset between the links of mav and payload 
	//Connect the firefly1 to payload
	if(mav_name1_ != "none") {
		std::cout<<"mavname1:"<<mav_name1_<<std::endl;
		mav_model1_ = world_->GetModel(mav_name1_);
		while(mav_model1_ == NULL){
			return;
		}
   	payload_link_ = model_->GetLink("binding_link1");
	 	mav_link_ = mav_model1_->GetLink(mav_name1_ + "/suspended_cable/suspended_tail_link");
				
		if(payload_link_ != NULL && mav_link_ != NULL){
	  	_tmp_pose = payload_link_->GetWorldCoGPose();
    	_tmp_pose += math::Pose(0,0,0.001,0,0,0); 
    	mav_model1_->SetLinkWorldPose(_tmp_pose, mav_link_);	
    	joint_ = physics_engine_->CreateJoint("ball", model_);
    	joint_->Load(mav_link_, payload_link_, math::Pose(0, 0, -0.001, 0, 0, 0));
	  	joint_->Attach(mav_link_, payload_link_);
    	mav_model1_->SetLinkWorldPose(math::Pose(0.5, 0.0, 0.0, 0, 0, 0), mav_name1_ + "/base_link");	
		}
	}

 	//Connect the firefly2 to payload
	if(mav_name2_ != "none"){
		std::cout<<"mavname2:"<<mav_name2_.c_str()<<std::endl;
   	mav_model2_ = world_->GetModel(mav_name2_);
		while(mav_model2_ == NULL){
			return;
		}
   	payload_link_ = model_->GetLink("binding_link2");
	 	mav_link_ = mav_model2_->GetLink(mav_name2_ + "/suspended_cable/suspended_tail_link");
	 	_tmp_pose = payload_link_->GetWorldCoGPose();
   	_tmp_pose+=math::Pose(0,0,0.001,0,0,0); 
   	mav_model2_->SetLinkWorldPose(_tmp_pose, mav_link_);	
   	joint_ = physics_engine_->CreateJoint("ball", model_);
   	joint_->Load(mav_link_, payload_link_, math::Pose(0, 0, -0.001, 0, 0, 0));
	 	joint_->Attach(mav_link_, payload_link_);
   	mav_model2_->SetLinkWorldPose(math::Pose(0, -0.5, 0., 0, 0, 0), mav_name2_ + "/base_link");	
  }

  //Connect the firefly3 to payload
	if(mav_name3_ != "none") {
		std::cout<<"mavname3:"<<mav_name3_<<std::endl;
   	mav_model3_ = world_->GetModel(mav_name3_);
		while(mav_model3_ == NULL){
			return;
		}
		payload_link_ = model_->GetLink("binding_link3");
	 	mav_link_ = mav_model3_->GetLink(mav_name3_ + "/suspended_cable/suspended_tail_link");
	 	_tmp_pose = payload_link_->GetWorldCoGPose();
   	_tmp_pose+=math::Pose(0,0,0.001,0,0,0); 
   	mav_model3_->SetLinkWorldPose(_tmp_pose, mav_link_);	
   	joint_ = physics_engine_->CreateJoint("ball", model_);
   	joint_->Load(mav_link_, payload_link_, math::Pose(0, 0, -0.001, 0, 0, 0));
	 	joint_->Attach(mav_link_, payload_link_);
   	mav_model3_->SetLinkWorldPose(math::Pose(0.0, 0.5, 0., 0, 0, 0), mav_name3_ + "/base_link");	
	}

	created_ = true;
}
void GazeboCableModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
	
  model_ = _model;
  world_ = model_->GetWorld();
  physics_engine_ = world_->GetPhysicsEngine();
  created_ = false;  

  if(_sdf->HasElement("mavName1"))
    mav_name1_ = _sdf->GetElement("mavName1")->Get<std::string>();
	else
	 	mav_name1_ = "none";

  if(_sdf->HasElement("mavName2"))
    mav_name2_ = _sdf->GetElement("mavName2")->Get<std::string>();
	else
	 	mav_name2_ = "none";

  if(_sdf->HasElement("mavName3"))
    mav_name3_ = _sdf->GetElement("mavName3")->Get<std::string>();
	else
	 	mav_name3_ = "none";
  // joint_->SetAxis(0, math::Vector3(1,0,0));	
	// joint_->Update();
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboCableModel::OnUpdate, this, _1));
}

// This gets called by the world update start event.
void GazeboCableModel::OnUpdate(const common::UpdateInfo& _info) {
	CreateCablePayloadJoint();
}

GZ_REGISTER_MODEL_PLUGIN(GazeboCableModel);
}
