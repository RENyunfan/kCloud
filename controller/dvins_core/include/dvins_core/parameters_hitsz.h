#ifndef INCLUDE_ROTORS_CONTROL_PARAMETERS_HITSZ_H_
#define INCLUDE_ROTORS_CONTROL_PARAMETERS_HITSZ_H_

#include "rotors_control/parameters.h"

namespace rotors_control {

// Default gripper parameters.
static constexpr double kDefaultGripperMass = 0.11;
static constexpr double kDefaultGripperInertiaYy = 0.0001;
static constexpr double kDefaultGripperLengthCenter = 0.005;

struct Gripper {
    Gripper()
        :mass(kDefaultGripperMass),
          inertia_yy(kDefaultGripperInertiaYy),
          length_center_gripper(kDefaultGripperLengthCenter){}
    Gripper(double _mass, double _inertia_yy, double _length_center_gripper)
        :mass(_mass),
          inertia_yy(_inertia_yy),
          length_center_gripper(_length_center_gripper){}
    double   mass;
    double   inertia_yy;
    double   length_center_gripper;
};

class MavArmParameters: VehicleParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MavArmParameters()
      : mass_(kDefaultMass),
        gravity_(kDefaultGravity),
        inertia_(Eigen::Vector3d(kDefaultInertiaXx, kDefaultInertiaYy,
                                 kDefaultInertiaZz).asDiagonal()) {}
  double mass_;
  const double gravity_;
  Eigen::Matrix3d inertia_;
  RotorConfiguration rotor_configuration_;
  Gripper gripper;
};

struct Hook {  //hook on the payload
  Hook()
      : position(0.0, 0.0, 0.0),
        cable_length(1.0){}
  Hook(Eigen::Vector3d _position, double _cable_length)
      : position(_position),
        cable_length(_cable_length){}
	Eigen::Vector3d position;
  double cable_length;
};

struct HookConfiguration {
  HookConfiguration() {
    // Hook configuration with only one hook.
    hooks.push_back(Hook());
  }
  std::vector<Hook> hooks;
};

class PayloadParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PayloadParameters()
      : mass_(0.2),
        gravity_(kDefaultGravity),
				size_(0.05, 0.05, 0.5),
        inertia_(Eigen::Vector3d(kDefaultInertiaXx, kDefaultInertiaYy,
                                 kDefaultInertiaZz).asDiagonal()) {}
  double mass_;
  const double gravity_;
	Eigen::Vector3d size_;
  Eigen::Matrix3d inertia_;
  HookConfiguration hook_configuration_;
};
}

#endif /* INCLUDE_ROTORS_CONTROL_PARAMETERS_HITSZ_H_ */
