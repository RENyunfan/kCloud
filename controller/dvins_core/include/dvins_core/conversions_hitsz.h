/*
 * Copyright 2015 Xuancong Li, HITSZ, Shenzhen, China
 * Copyright 2015 Haoyao Chen, HITSZ, Shenzhen/ASL, ETH Zurich, Switzerland
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

// Extended Conversion functions between Eigen types and MAV ROS message types.

#ifndef MAV_MSGS_CONVERSIONS_HITSZ_H
#define MAV_MSGS_CONVERSIONS_HITSZ_H

#include "dvins_core/conversions.h"
#include "dvins_core/eigen_mav_msgs_hitsz.h"
#include <dvins_core/CommandMavAndGripper.h>
#include <dvins_core/CommandArmServoAngle.h>

namespace mav_msgs {

// Convenience method to quickly create a trajectory from a single waypoint.
inline void msgMultiDofJointTrajectoryFromPositionVelocityYaw(
  const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, double yaw,
  trajectory_msgs::MultiDOFJointTrajectory* msg) {
  assert(msg != NULL);

  EigenTrajectoryPoint point;
  point.position_W = position;
  point.velocity_W = velocity;
  point.setFromYaw(yaw);

  msgMultiDofJointTrajectoryFromEigen(point, msg);
}

inline void eigenGripperAngleFromMsg(const dvins_core::CommandArmServoAngle& msg,
                                     EigenGripperAngle* gripper_angle){
    assert(gripper_angle != NULL);
    gripper_angle->gripper_angle << msg.beta,
            msg.dbeta,
            msg.d2beta,
            msg.d3beta,
            msg.d4beta;
    gripper_angle->grasp_or_not = msg.grasp_or_not;

}

inline void msgCommandArmAngleFromEigen(const EigenGripperAngle &gripper_point,
                                        dvins_core::CommandArmServoAngle *msg){
    assert(msg != NULL);
    msg->beta = gripper_point.gripper_angle(0);
    msg->dbeta = gripper_point.gripper_angle(1);
    msg->d2beta = gripper_point.gripper_angle(2);
    msg->d3beta = gripper_point.gripper_angle(3);
    msg->d4beta = gripper_point.gripper_angle(4);
    msg->grasp_or_not = gripper_point.grasp_or_not;
}

inline void msgCommandArmAngleTrajectoryFromEigen(const EigenGripperAngle& gripper_point,
                                                  dvins_core::CommandArmServoAngleTrajectory* msg){
    assert(msg != NULL);
    dvins_core::CommandArmServoAngle point_msg;
    msgCommandArmAngleFromEigen(gripper_point, &point_msg);
    msg->desried_arm_rad.clear();
    msg->desried_arm_rad.push_back(point_msg);
}

MAV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenGripperAngle)

} // end namespace mav_msgs

#endif // MAV_MSGS_CONVERSIONS_HITSZ_H

