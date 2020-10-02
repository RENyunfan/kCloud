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

#ifndef ROTORS_CONTROL_LEE_PAYLOAD_POSITION_CONTROLLER_H
#define ROTORS_CONTROL_LEE_PAYLOAD_POSITION_CONTROLLER_H

#include <dvins_core/conversions_hitsz.h>
#include <dvins_core/eigen_mav_msgs_hitsz.h>

#include "dvins_core/common_hitsz.h"
#include "dvins_core/parameters.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <sys/timeb.h>
namespace rotors_control {

// Default values for the lee position controller and the Asctec Firefly.
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(6, 6, 6);
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(4.7, 4.7, 4.7);
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 3, 0.035);
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 0.52, 0.025);

class LeePayloadPositionControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // aligned memory for 128 to use SSE acceleration
  LeePayloadPositionControllerParameters()
      : position_gain_(kDefaultPositionGain),
        velocity_gain_(kDefaultVelocityGain),
        attitude_gain_(kDefaultAttitudeGain),
        angular_rate_gain_(kDefaultAngularRateGain) {
    calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
  }

  Eigen::Matrix4Xd allocation_matrix_;
  Eigen::Vector3d position_gain_;
  Eigen::Vector3d velocity_gain_;
  Eigen::Vector3d attitude_gain_;
  Eigen::Vector3d angular_rate_gain_;
  RotorConfiguration rotor_configuration_;
};

class LeePayloadPositionController {
 public:
  LeePayloadPositionController();
  ~LeePayloadPositionController();
  void InitializeParameters();
  void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const;

  void SetMavOdometry(const EigenOdometry& odometry);
  void SetPayloadOdometry(const EigenOdometry& odometry);
  void SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& _command_trajectory);
  void SetMavPayloadTrajectory(const mav_msgs::EigenTrajectoryPointDeque& _command_trajectory);
  void SetRollPitchYawrateThrust(const mav_msgs::EigenRollPitchYawrateThrust& roll_pitch_yawrate_thrust);
  LeePayloadPositionControllerParameters controller_parameters_;
  VehicleParameters vehicle_parameters_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  bool initialized_params_;
  bool controller_active_;

  Eigen::Vector3d normalized_attitude_gain_;
  Eigen::Vector3d normalized_angular_rate_gain_;
  Eigen::MatrixX4d angular_acc_to_rotor_velocities_;

  mav_msgs::EigenTrajectoryPoint command_trajectory_;
  mav_msgs::EigenTrajectoryPoint command_payload_trajectory_;
  EigenOdometry mav_odometry_;
  EigenOdometry payload_odometry_;
  mav_msgs::EigenRollPitchYawrateThrust roll_pitch_yawrate_altitude_;

  void ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
                                Eigen::Vector3d* angular_acceleration) const;
  void ComputeDesiredAcceleration(Eigen::Vector3d* acceleration) const;
  void ComputeDesiredAccelerationold(Eigen::Vector3d* acceleration) const;
};
}

#endif // ROTORS_CONTROL_LEE_PAYLOAD_POSITION_CONTROLLER_H
