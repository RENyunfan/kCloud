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

#include "dvins_core/lpc.h"

namespace rotors_control {

LeePayloadPositionController::LeePayloadPositionController()
    : initialized_params_(false),
      controller_active_(false) {
  InitializeParameters();
  std::ofstream outFile;
	outFile.open("/home/crown/dv_ws/data/dvins_core/log/data.csv", std::ios::out); // 打开模式可省略
  outFile << "time"<<",";
  outFile << "positionx"<<"," << "positiony" <<"," << "positionz" <<",";
  outFile << "cmd_positionx"<<"," << "cmd_positiony" <<"," << "cmd_positionz" <<",";
  outFile << "pose_errorx"<<"," << "pose_errory" <<"," << "pose_errorz" <<",";
  outFile << "velocityx"<<"," << "velocityy" <<"," << "velocityz" <<",";
  outFile << "cmd_velx"<<"," << "cmd_vely" <<"," << "cmd_velz" <<",";
  outFile << "vel_errorx"<<"," << "vel_errory" <<"," << "vel_errorz" <<",";
  outFile << "accx"<<"," << "accy" <<"," << "accz" <<",";
  outFile << "ang_errorx"<<"," << "ang_errory" <<"," << "ang_errorz" <<",";
  outFile << "ang_accx"<<"," << "ang_accy" <<"," << "ang_accz" <<std::endl;
  outFile.close();

}

LeePayloadPositionController::~LeePayloadPositionController() {}

void LeePayloadPositionController::InitializeParameters() {
  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_attitude_gain_ = controller_parameters_.attitude_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_angular_rate_gain_ = controller_parameters_.angular_rate_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();

  Eigen::Matrix4d I;
  I.setZero();
  I.block<3, 3>(0, 0) = vehicle_parameters_.inertia_;
  I(3, 3) = 1;
  angular_acc_to_rotor_velocities_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
  // A^{ \dagger} = A^T*(A*A^T)^{-1}
  angular_acc_to_rotor_velocities_ = controller_parameters_.allocation_matrix_.transpose()
      * (controller_parameters_.allocation_matrix_
      * controller_parameters_.allocation_matrix_.transpose()).inverse() * I;
  initialized_params_ = true;
}
void LeePayloadPositionController::SetRollPitchYawrateThrust(
    const mav_msgs::EigenRollPitchYawrateThrust& roll_pitch_yawrate_thrust) {
  roll_pitch_yawrate_altitude_ = roll_pitch_yawrate_thrust;
  controller_active_ = true;
}

void LeePayloadPositionController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const {
  assert(rotor_velocities);
  assert(initialized_params_);

  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
    return;
  }

  Eigen::Vector3d acceleration;
  ComputeDesiredAcceleration(&acceleration);

  Eigen::Vector3d angular_acceleration;
  ComputeDesiredAngularAcc(acceleration, &angular_acceleration);

  // Project thrust onto body z axis.
  double thrust = -vehicle_parameters_.mass_ * acceleration.dot(mav_odometry_.orientation.toRotationMatrix().col(2));

  Eigen::Vector4d angular_acceleration_thrust;
  angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
  angular_acceleration_thrust(3) = thrust;

  *rotor_velocities = angular_acc_to_rotor_velocities_ * angular_acceleration_thrust;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
}

void LeePayloadPositionController::SetMavOdometry(const EigenOdometry& odometry) {
  mav_odometry_ = odometry;
}

void LeePayloadPositionController::SetPayloadOdometry(const EigenOdometry& odometry) {
  payload_odometry_ = odometry;
}

void LeePayloadPositionController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& _command_trajectory) {
  //command_trajectory_ = command_trajectory;
  //controller_active_ = true;
}

void LeePayloadPositionController::SetMavPayloadTrajectory(
    const mav_msgs::EigenTrajectoryPointDeque& _command_trajectory) {
  command_trajectory_ = _command_trajectory.front(); //mav
  command_payload_trajectory_ = _command_trajectory.back(); //payload
  // std::cout << command_trajectory_.position_W<<"cmd_traj\n";
  // std::cout << command_payload_trajectory_.position_W<<"payload_trj\n";
  controller_active_ = true;
}

// Implementation from the T. Lee et al. paper
// Geometric Control and Differential Flatness of a Quadrotor UAV with a Cable-Suspended Load
void LeePayloadPositionController::ComputeDesiredAcceleration(Eigen::Vector3d* acceleration) const {
  assert(acceleration);

  Eigen::Vector3d position_error;
  position_error = mav_odometry_.position - command_trajectory_.position_W;

  // Transform velocity to world frame.
  const Eigen::Matrix3d R_W_I = mav_odometry_.orientation.toRotationMatrix();
  Eigen::Vector3d velocity_W =   R_W_I * mav_odometry_.velocity;
  Eigen::Vector3d velocity_error;
  velocity_error = velocity_W - command_trajectory_.velocity_W;

  Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());

  *acceleration = (position_error.cwiseProduct(controller_parameters_.position_gain_)
      + velocity_error.cwiseProduct(controller_parameters_.velocity_gain_)) / vehicle_parameters_.mass_
      - vehicle_parameters_.gravity_ * e_3 - command_trajectory_.acceleration_W;
  // std::cout << "cta" << *acceleration<< vehicle_parameters_.gravity_<<" "<< position_error.cwiseProduct(controller_parameters_.position_gain_) << std::endl;
  // std::cout << "position_error" << position_error[0]<< "\t"<< position_error[1]<< "\t"<< position_error[2]<< "\t"<< std::endl;
  Eigen::Vector3d acc;
  acc = *acceleration;
  std::ofstream outFile;
	outFile.open("/home/crown/dv_ws/data/dvins_core/log/data.csv", std::ios::app); // 打开模式可省略
  struct timeb tb;
  ftime(&tb);
  outFile << tb.millitm<<",";
	outFile << mav_odometry_.position[0]<< ","<< mav_odometry_.position[1]<< ","<< mav_odometry_.position[2]<< ",";
	outFile << command_trajectory_.position_W[0]<< ","<< command_trajectory_.position_W[1]<< ","<< command_trajectory_.position_W[2]<< ",";
	outFile << position_error[0]<< ","<< position_error[1]<< ","<< position_error[2]<< ",";
	outFile << velocity_W[0]<< ","<< velocity_W[1]<< ","<< velocity_W[2]<< ",";
	outFile << velocity_error[0]<< ","<< velocity_error[1]<< ","<< velocity_error[2]<< ",";
	outFile << acc[0] << "," << acc[1] << "," << acc[2] << ",";
	outFile.close();
  // check 20200521 nothing wrong with acceleration
  // *acceleration = (position_error.cwiseProduct(controller_parameters_.position_gain_)
  //     + velocity_error.cwiseProduct(controller_parameters_.velocity_gain_)) / vehicle_parameters_.mass_
  //     - vehicle_parameters_.gravity_ * e_3 - command_trajectory_.acceleration_W;
}
void LeePayloadPositionController::ComputeDesiredAccelerationold(Eigen::Vector3d* acceleration) const {
  assert(acceleration);

 	Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());

  Eigen::Vector3d q, q_des;         //the measure/desired unit vector from quarotor to the load
	Eigen::Vector3d q_rate_des;
  //test
  //std::cout<<"Mav Trajectory (in controller):" << std::endl;
  //std::cout<<" desired_position:" << command_trajectory_.position_W << std::endl;
  //std::cout<<" desired_velocity:" << command_trajectory_.velocity_W << std::endl;
  //std::cout<<" desired_ori:" << command_trajectory_.orientation_W_B.toRotationMatrix() << std::endl;
  //std::cout<<" desired_angular_velocity:" << command_trajectory_.angular_velocity_W << std::endl;
  //test
  //std::cout<<"Payload Trajectory (in controller):" << std::endl;
  //std::cout<<" desired_position:" << command_payload_trajectory_.position_W << std::endl;
  //std::cout<<" desired_velocity:" << command_payload_trajectory_.velocity_W << std::endl;
  //std::cout<<" desired_ori:" << command_payload_trajectory_.orientation_W_B.toRotationMatrix() << std::endl;
  //std::cout<<" desired_angular_velocity:" << command_payload_trajectory_.angular_velocity_W << std::endl;

  Eigen::Vector3d position_error;
  //8888  position_error = mav_odometry_.position - command_trajectory_.position_W;
  const Eigen::Vector3d pos = Eigen::Vector3d(0.0, 0.0, 2);
  position_error =  mav_odometry_.position - pos;
  std::cout << position_error(0) << "\t" << position_error(1) <<"\t"<< position_error(2)<<"\n";
  
  // Transform velocity to world frame.
  const Eigen::Matrix3d R_W_I = mav_odometry_.orientation.toRotationMatrix();
  Eigen::Vector3d velocity_W =  R_W_I * mav_odometry_.velocity;
  Eigen::Vector3d velocity_error;
  //8888  velocity_error = velocity_W - command_trajectory_.velocity_W;
  velocity_error.setZero();
	// Transform payload velocity to world frame.
	Eigen::Vector3d payload_velocity_W =  payload_odometry_.orientation.toRotationMatrix() * payload_odometry_.velocity;

	// Force resultant design
	Eigen::Matrix3d q_matrix;
  Eigen::Vector3d q_rate, px_des, pv_des, pa_des;
  Eigen::Vector3d q_acc_des;
	Eigen::Vector3d q_error, q_rate_error, px_error, pv_error;
	Eigen::Vector3d w;                //Rotate rate
	Eigen::Vector3d A;
	Eigen::Vector3d F_n, F_rt;
	Eigen::Vector3d F_pd;
	Eigen::Vector3d F_ff, F_g;
  double p_mass(0.2);                          //payload mass
	double l(0.75);                              //cable length
	const Eigen::Vector3d kx = Eigen::Vector3d(1.0, 1.0, 1.0);
  const Eigen::Vector3d kv = Eigen::Vector3d(.5, .5, .5);
  const Eigen::Vector3d kq = Eigen::Vector3d(1.5, 0.1, 0.1);
  const Eigen::Vector3d kw = Eigen::Vector3d(0.1, 0.1, 0.1);  //p d controller parameters
  // const Eigen::Vector3d kmx = Eigen::Vector3d(0.5, 0.5, 0.5);
  const Eigen::Vector3d kmx = Eigen::Vector3d(1, 1, 1);
  const Eigen::Vector3d kmv = Eigen::Vector3d(0.1, 0.1, .5);
  const Eigen::Vector3d uint = Eigen::Vector3d(0, 0, 1);

  // const Eigen::Vector3d kx = Eigen::Vector3d(2.0, 2.0, 1.5);
  // const Eigen::Vector3d kv = Eigen::Vector3d(2.5, 2.5, 2.5);
  // const Eigen::Vector3d kq = Eigen::Vector3d(2.5, 2.5, 1.5);
  // const Eigen::Vector3d kw = Eigen::Vector3d(2.5, 2.5, 2.5);  //p d controller parameters
  // const Eigen::Vector3d kmx = Eigen::Vector3d(1.0, 1.0, 1.0);
  // const Eigen::Vector3d kmv = Eigen::Vector3d(1.5, 1.5, 2.5);
  // const Eigen::Vector3d uint = Eigen::Vector3d(0, 0, 1);
  //****    px_des = command_payload_trajectory_.position_W;  //payload position and velocity
  //****   pv_des = command_payload_trajectory_.velocity_W;

	pa_des.setZero();
	q_acc_des.setZero();
	q = payload_odometry_.position - mav_odometry_.position;
	q = q/q.norm();
    //q.norm()
	w = (payload_velocity_W - velocity_W) / (payload_odometry_.position - mav_odometry_.position).norm();
	w = q.cross(w);
	q_rate = w.cross(q); 
    skewMatrixFromVector(q, &q_matrix);
  //8888    px_error = payload_odometry_.position - px_des;
  //8888    pv_error = payload_velocity_W - pv_des;
  px_error = payload_odometry_.position - l*uint;
  pv_error.setZero();
  px_error.setZero();

  F_g << 0, 0, -1 * (vehicle_parameters_.mass_ + p_mass) * vehicle_parameters_.gravity_;
  // WSQ 02020507
  A = - px_error.cwiseProduct(kx) - pv_error.cwiseProduct(kv) + (vehicle_parameters_.mass_ + p_mass) * pa_des
          - F_g + vehicle_parameters_.mass_ * l * (q_rate.dot(q_rate) * q);  //F_g is very important here
  A += - position_error.cwiseProduct(kmx) - velocity_error.cwiseProduct(kmv);
  // WSQ 02020507
  // A = - position_error.cwiseProduct(kmx) - velocity_error.cwiseProduct(kmv);

	//  	- F_g; //F_g is very important here
	q_rate_error.setZero();
	//q_des = -1 * A/A.norm();
  //8888	q_des = command_payload_trajectory_.orientation_W_B.toRotationMatrix().transpose() * (-e_3);
  //8888	q_rate_des = command_payload_trajectory_.angular_velocity_W.cross(q_des);
  q_des =  mav_odometry_.orientation.toRotationMatrix()*(-e_3);
  q_rate_error.setZero();
	q_error = q_matrix * q_matrix * q_des;
	q_rate_error = q_rate - (q_des.cross(q_rate_des)).cross(q);
	//if((command_payload_trajectory_.orientation_W_B.toRotationMatrix() * (-e_3))(2) == -1)
	F_n = A;
  // std::cout<<A<<std::endl;
	//else
	//F_n = A.dot(q) * q;
	F_pd = - q_error.cwiseProduct(kq) - q_rate_error.cwiseProduct(kw);
	F_ff = vehicle_parameters_.mass_ * l * q.dot(q_des.cross(q_rate_des)) * q.cross(q_rate)
	        + vehicle_parameters_.mass_ * l * (q_des.cross(q_acc_des)).cross(q);
	
	F_rt = F_n - F_pd - F_ff;

	*acceleration = -1 * F_rt / vehicle_parameters_.mass_; 

  // WSQ cmt 20200507
  // printf("mav position value: [%f, %f, %f].\n", mav_odometry_.position(0), mav_odometry_.position(1), mav_odometry_.position(2));
  // printf("mav command position value: [%f, %f, %f].\n", command_trajectory_.position_W(0), command_trajectory_.position_W(1), command_trajectory_.position_W(2));
  // printf("mav position error value: [%f, %f, %f].\n", position_error(0), position_error(1), position_error(2));

  // printf("mav velocity value: [%f, %f, %f].\n", velocity_W(0), velocity_W(1), velocity_W(2));
  // printf("mav command velocity value: [%f, %f, %f].\n", command_trajectory_.velocity_W(0), command_trajectory_.velocity_W(1), command_trajectory_.velocity_W(2));
  // printf("mav velocity error value: [%f, %f, %f].\n", velocity_error(0), velocity_error(1), velocity_error(2));
  // WSQ cmt 20200507

  //printf("mav orientation: [%f, %f, %f, %f].\n", mav_odometry_.orientation.w(), mav_odometry_.orientation.x(), mav_odometry_.orientation.y(), 
	//		 mav_odometry_.orientation.z());
  //printf("mav desired orientation: [%f, %f, %f, %f].\n", command_trajectory_.orientation_W_B.w(), command_trajectory_.orientation_W_B.x(),
	//	 	 command_trajectory_.orientation_W_B.y(), command_trajectory_.orientation_W_B.z());

  // WSQ cmt 20200507
  // printf("payload position value: [%f, %f, %f].\n", payload_odometry_.position(0), payload_odometry_.position(1), payload_odometry_.position(2));
  // printf("payload command position value: [%f, %f, %f].\n", command_payload_trajectory_.position_W(0), command_payload_trajectory_.position_W(1),
	// 	 	command_payload_trajectory_.position_W(2));
  // printf("payload position error value: [%f, %f, %f].\n", px_error(0), px_error(1), px_error(2));
  // WSQ cmt 20200507

  //printf("payload velocity value: [%f, %f, %f].\n", payload_velocity_W(0), payload_velocity_W(1), payload_velocity_W(2));
  //printf("payload command velocity value: [%f, %f, %f].\n", command_payload_trajectory_.velocity_W(0), command_payload_trajectory_.velocity_W(1),
	//	 	command_payload_trajectory_.velocity_W(2));

  // WSQ cmt 20200507
  // printf("payload velocity error value: [%f, %f, %f].\n", pv_error(0), pv_error(1), pv_error(2));

  // printf("payload desired orientation: [%f, %f, %f, %f].\n", command_payload_trajectory_.orientation_W_B.w(), 
	// 		 command_payload_trajectory_.orientation_W_B.x(), command_payload_trajectory_.orientation_W_B.y(), 
	// 		 command_payload_trajectory_.orientation_W_B.z());
  // printf("q desired value: [%f, %f, %f].\n", q_des(0), q_des(1), q_des(2));
  // printf("q value: [%f, %f, %f].\n", q(0), q(1), q(2));
  // printf("q error value: [%f, %f, %f].\n", q_error(0), q_error(1), q_error(2));
  // printf("q_rate value: [%f, %f, %f].\n", q_rate(0), q_rate(1), q_rate(2));
  // printf("q_rate desired value: [%f, %f, %f].\n", q_rate_des(0), q_rate_des(1), q_rate_des(2));
  // printf("q rate  error value: [%f, %f, %f].\n", q_rate_error(0), q_rate_error(1), q_rate_error(2));
  //printf("Acceleration value: [%f, %f, %f].\n", (*acceleration)(0), (*acceleration)(1), (*acceleration)(2));
  // WSQ cmt 20200507

}
// void LeePayloadPositionController::ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
//                                                      Eigen::Vector3d* angular_acceleration) const {
//   assert(angular_acceleration);

//   Eigen::Matrix3d R = mav_odometry_.orientation.toRotationMatrix();

//   // Get the desired rotation matrix.
//   Eigen::Vector3d b1_des;
//   double yaw = command_trajectory_.getYaw();
//   b1_des << cos(yaw), sin(yaw), 0;

//   Eigen::Vector3d b3_des;
//   b3_des = -acceleration / acceleration.norm();

//   Eigen::Vector3d b2_des;
//   b2_des = b3_des.cross(b1_des);
//   b2_des.normalize();

//   Eigen::Matrix3d R_des;
//   R_des.col(0) = b2_des.cross(b3_des);
//   R_des.col(1) = b2_des;
//   R_des.col(2) = b3_des;

//   // Angle error according to lee et al.
//   Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
//   Eigen::Vector3d angle_error;
//   vectorFromSkewMatrix(angle_error_matrix, &angle_error);

//   // TODO(burrimi) include angular rate references at some point.
//   Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
//   angular_rate_des[2] = command_trajectory_.getYawRate();

//   Eigen::Vector3d angular_rate_error = mav_odometry_.angular_velocity - R_des.transpose() * R * angular_rate_des;

//   *angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
//                            - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
//                            + mav_odometry_.angular_velocity.cross(mav_odometry_.angular_velocity); // we don't need the inertia matrix here
// }


void LeePayloadPositionController::ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
                                                     Eigen::Vector3d* angular_acceleration) const {
  assert(angular_acceleration);

  Eigen::Matrix3d R = mav_odometry_.orientation.toRotationMatrix(); //There are EigenOdometry type in mav_msgs and rotor_control (confused)
	//std::cout<<"Yaw:" << command_trajectory_.getYaw() << std::endl;
  // Get the desired rotation matrix.
  //***********************************
  //  Eigen::Vector3d b1_des;
  //  b1_des << cos(command_trajectory_.getYaw()), sin(command_trajectory_.getYaw()), 0;

  //  Eigen::Vector3d b3_des;
  //  b3_des = -1 * acceleration / acceleration.norm();

  //  Eigen::Vector3d b2_des;
  //  b2_des = -1 * b3_des.cross(b3_des.cross(b1_des));
  //  b2_des.normalize();

  //  Eigen::Matrix3d R_des;
  //  R_des.col(0) = b2_des;
  //  R_des.col(1) = b3_des.cross(b1_des);
  //  R_des.col(2) = b3_des;
  //****************************************************

  //Eigen::Vector3d b1_des;
  //double yaw = command_trajectory_.getYaw();
  //b1_des << cos(yaw), sin(yaw), 0;

  //Eigen::Vector3d b3_des;
  //b3_des = -acceleration / acceleration.norm();

  //Eigen::Vector3d b2_des;
  //b2_des = b3_des.cross(b1_des);
  //b2_des.normalize();

  //Eigen::Matrix3d R_des;
  //R_des.col(0) = b2_des.cross(b3_des);
  //R_des.col(1) = b2_des;
  //R_des.col(2) = b3_des;


 	//Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());
	//if((command_payload_trajectory_.orientation_W_B.toRotationMatrix() * (-e_3))(2) != -1.0){
  //	R_des = command_trajectory_.orientation_W_B.toRotationMatrix();
	//	std::cout<<"R_des:" << R_des << std::endl;
	//}
  // Angle error according to lee et al.

  double yaw = atan2(R(1, 0), R(0, 0));

  // Get the desired rotation matrix.
  Eigen::Matrix3d R_des;
  R_des = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())  // yaw
        * Eigen::AngleAxisd(roll_pitch_yawrate_altitude_.roll, Eigen::Vector3d::UnitX())  // roll
        * Eigen::AngleAxisd(roll_pitch_yawrate_altitude_.thrust(2), Eigen::Vector3d::UnitY());  // pitch

  Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
  Eigen::Vector3d angle_error;
  vectorFromSkewMatrix(angle_error_matrix, &angle_error);

  // TODO(burrimi) include angular rate references at some point.
  Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
  // angular_rate_des[0] = command_trajectory_.
  // angular_rate_des[0] = command_trajectory_.getYawRate();
  angular_rate_des[2] = command_trajectory_.getYawRate();
  // angular_rate_des(2) = roll_pitch_yawrate_altitude_.yaw_rate; // wtf is this? why setting current yaw as desired yaw?

  Eigen::Vector3d angular_rate_error = mav_odometry_.angular_velocity - R_des.transpose() * R * angular_rate_des;

  *angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
                           - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
                           + mav_odometry_.angular_velocity.cross(mav_odometry_.angular_velocity); // we don't need the inertia matrix here
  Eigen::Vector3d acc_ang;
  acc_ang = *angular_acceleration;
  std::ofstream outFile;
	outFile.open("/home/crown/dv_ws/data/dvins_core/log/data.csv", std::ios::app); // 打开模式可省略
	outFile << angular_rate_error[0]<< ","<< angular_rate_error[1]<< ","<< angular_rate_error[2]<< ",";
	outFile << acc_ang[0] << "," << acc_ang[1] << "," << acc_ang[2] << std::endl;
	outFile.close();
  // std::cout<< "*angular_acceleration" << *angular_acceleration << std::endl;
}

// void LeePayloadPositionController::ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
//                                                      Eigen::Vector3d* angular_acceleration) const {
//   assert(angular_acceleration);

//   Eigen::Matrix3d R = mav_odometry_.orientation.toRotationMatrix();

//   // Get the desired rotation matrix.
//   Eigen::Vector3d b1_des;
//   double yaw = command_trajectory_.getYaw();
//   b1_des << cos(yaw), sin(yaw), 0;

//   Eigen::Vector3d b3_des;
//   b3_des = -acceleration / acceleration.norm();

//   Eigen::Vector3d b2_des;
//   b2_des = b3_des.cross(b1_des);
//   b2_des.normalize();

//   Eigen::Matrix3d R_des;
//   R_des.col(0) = b2_des.cross(b3_des);
//   R_des.col(1) = b2_des;
//   R_des.col(2) = b3_des;

//   // Angle error according to lee et al.
//   Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
//   Eigen::Vector3d angle_error;
//   vectorFromSkewMatrix(angle_error_matrix, &angle_error);

//   // TODO(burrimi) include angular rate references at some point.
//   Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
//   angular_rate_des[2] = command_trajectory_.getYawRate();

//   Eigen::Vector3d angular_rate_error = mav_odometry_.angular_velocity - R_des.transpose() * R * angular_rate_des;

//   *angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
//                            - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
//                            + mav_odometry_.angular_velocity.cross(mav_odometry_.angular_velocity); // we don't need the inertia matrix here
// }


}

