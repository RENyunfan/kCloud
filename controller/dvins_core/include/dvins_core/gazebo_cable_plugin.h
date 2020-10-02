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


#ifndef GAZEBO_PLUGINS_CABLE_MODEL_H
#define GAZEBO_PLUGINS_CABLE_MODEL_H

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

namespace gazebo {
// Default values
class GazeboCableModel : public ModelPlugin {
 public:
  GazeboCableModel()
      : ModelPlugin(),
        node_handle_(nullptr) {
  }

  virtual ~GazeboCableModel();

  virtual void InitializeParams();

 protected:
	void CreateCablePayloadJoint();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

 private:
  std::string joint_name_;
  std::string link_name_;
  std::string namespace_;
  std::string mav_name1_;
  std::string mav_name2_;
  std::string mav_name3_;
  std::string cable_name_;
  ros::NodeHandle* node_handle_;

  physics::PhysicsEnginePtr physics_engine_;
  physics::ModelPtr model_;
  physics::ModelPtr mav_model1_;
  physics::ModelPtr mav_model2_;
  physics::ModelPtr mav_model3_;
  physics::ModelPtr cable_model_;
  physics::WorldPtr world_;
  physics::JointPtr joint_;
  physics::LinkPtr payload_link_;
  physics::LinkPtr mav_link_;
  physics::LinkPtr cable_link_head_;
  physics::LinkPtr cable_link_tail_;

	bool created_;
  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

};
}

#endif // GAZEBO_PLUGINS_CABLE_MODEL_H
