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

#ifndef MAV_MSGS_EIGEN_MAV_MSGS_HITSZ_H
#define MAV_MSGS_EIGEN_MAV_MSGS_HITSZ_H

#include "mav_msgs/eigen_mav_msgs.h"

namespace mav_msgs {

struct EigenGripperAngle{
    EigenGripperAngle()
        :gripper_angle(Eigen::VectorXd::Zero(5,1)),
         grasp_or_not(false){};
    EigenGripperAngle(const Eigen::Vector4d& _gripper_angle, bool _grasp_or_not){
        gripper_angle = _gripper_angle;
        grasp_or_not = _grasp_or_not;
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::VectorXd gripper_angle;
    bool grasp_or_not;
};

}

#endif // MAV_MSGS_EIGEN_MAV_MSGS_HITSZ_H
