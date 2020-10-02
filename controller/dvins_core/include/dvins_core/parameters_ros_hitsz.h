#ifndef INCLUDE_ROTORS_CONTROL_PARAMETERS_ROS_HITSZ_H_
#define INCLUDE_ROTORS_CONTROL_PARAMETERS_ROS_HITSZ_H_

#include "rotors_control/parameters_ros.h"
#include "rotors_control/parameters_hitsz.h"

namespace rotors_control {

inline void GetGripperParameters(const ros::NodeHandle& nh, Gripper* gripper_parameters){
    GetRosParameter(nh,"mass_gripper",gripper_parameters->mass,&gripper_parameters->mass);
    GetRosParameter(nh,"rotation_inertia_gripper",gripper_parameters->inertia_yy,&gripper_parameters->inertia_yy);
    GetRosParameter(nh,"length_center_gripper",gripper_parameters->length_center_gripper,&gripper_parameters->length_center_gripper);
}

inline void GetMavArmParameters(const ros::NodeHandle& nh, MavArmParameters* vehicle_parameters) {
  GetRosParameter(nh, "mass",
                  vehicle_parameters->mass_,
                  &vehicle_parameters->mass_);
  GetRosParameter(nh, "inertia/xx",
                  vehicle_parameters->inertia_(0, 0),
                  &vehicle_parameters->inertia_(0, 0));
  GetRosParameter(nh, "inertia/xy",
                  vehicle_parameters->inertia_(0, 1),
                  &vehicle_parameters->inertia_(0, 1));
  vehicle_parameters->inertia_(1, 0) = vehicle_parameters->inertia_(0, 1);
  GetRosParameter(nh, "inertia/xz",
                  vehicle_parameters->inertia_(0, 2),
                  &vehicle_parameters->inertia_(0, 2));
  vehicle_parameters->inertia_(2, 0) = vehicle_parameters->inertia_(0, 2);
  GetRosParameter(nh, "inertia/yy",
                  vehicle_parameters->inertia_(1, 1),
                  &vehicle_parameters->inertia_(1, 1));
  GetRosParameter(nh, "inertia/yz",
                  vehicle_parameters->inertia_(1, 2),
                  &vehicle_parameters->inertia_(1, 2));
  vehicle_parameters->inertia_(2, 1) = vehicle_parameters->inertia_(1, 2);
  GetRosParameter(nh, "inertia/zz",
                  vehicle_parameters->inertia_(2, 2),
                  &vehicle_parameters->inertia_(2, 2));
  GetRotorConfiguration(nh, &vehicle_parameters->rotor_configuration_);
  GetGripperParameters(nh,&vehicle_parameters->gripper);
}

inline void GetHookConfiguration(const ros::NodeHandle& nh,
                                  HookConfiguration* hook_configuration) {
  std::map<std::string, double> single_hook;
  std::string hook_configuration_string = "load_hook_configuration/";
  unsigned int i = 0;
  while (nh.getParam(hook_configuration_string + std::to_string(i), single_hook)) {
    if (i == 0) {
      hook_configuration->hooks.clear();
    }
    Hook hook;
    nh.getParam(hook_configuration_string + std::to_string(i) + "/x",
                 hook.position(0));
    nh.getParam(hook_configuration_string + std::to_string(i) + "/y",
                 hook.position(1));
    nh.getParam(hook_configuration_string + std::to_string(i) + "/z",
                 hook.position(2));
    nh.getParam(hook_configuration_string + std::to_string(i) + "/cable_length",
                 hook.cable_length);
    hook_configuration->hooks.push_back(hook);
    ++i;
  }
}

inline void GetPayloadParameters(const ros::NodeHandle& nh, PayloadParameters* payload_parameters) {
  GetRosParameter(nh, "load_mass",
                  payload_parameters->mass_,
                  &payload_parameters->mass_);
  GetRosParameter(nh, "load_size/length",
                  payload_parameters->size_(0),
                  &payload_parameters->size_(0));
  GetRosParameter(nh, "load_size/width",
                  payload_parameters->size_(1),
                  &payload_parameters->size_(1));
  GetRosParameter(nh, "load_size/height",
                  payload_parameters->size_(2),
                  &payload_parameters->size_(2));
  GetRosParameter(nh, "load_inertia/xx",
                  payload_parameters->inertia_(0, 0),
                  &payload_parameters->inertia_(0, 0));
  GetRosParameter(nh, "load_inertia/xy",
                  payload_parameters->inertia_(0, 1),
                  &payload_parameters->inertia_(0, 1));
  payload_parameters->inertia_(1, 0) = payload_parameters->inertia_(0, 1);
  GetRosParameter(nh, "load_inertia/xz",
                  payload_parameters->inertia_(0, 2),
                  &payload_parameters->inertia_(0, 2));
  payload_parameters->inertia_(2, 0) = payload_parameters->inertia_(0, 2);
  GetRosParameter(nh, "load_inertia/yy",
                  payload_parameters->inertia_(1, 1),
                  &payload_parameters->inertia_(1, 1));
  GetRosParameter(nh, "load_inertia/yz",
                  payload_parameters->inertia_(1, 2),
                  &payload_parameters->inertia_(1, 2));
  payload_parameters->inertia_(2, 1) = payload_parameters->inertia_(1, 2);
  GetRosParameter(nh, "load_inertia/zz",
                  payload_parameters->inertia_(2, 2),
                  &payload_parameters->inertia_(2, 2));
  GetHookConfiguration(nh, &payload_parameters->hook_configuration_);
}

}

#endif /* INCLUDE_ROTORS_CONTROL_PARAMETERS_ROS_HITSZ_H_ */
