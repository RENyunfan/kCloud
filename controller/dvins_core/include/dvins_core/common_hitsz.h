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

#ifndef INCLUDE_ROTORS_CONTROL_COMMON_HITSZ_H_
#define INCLUDE_ROTORS_CONTROL_COMMON_HITSZ_H_

#include "dvins_core/control_common.h"
#include "dvins_core/default_topics_hitsz.h"

namespace rotors_control {

enum taskState{HOVER=0, PATHFOLLOW, LOAD};
enum uavStatus{HOVERING=0, PATHFOLLOWING, LOADING, HOVERED, LOADED, RESTING};

// Default values, now using values from mav_comm.
static const std::string kDefaultCommandArmTrajectoryTopic =
    mav_msgs::default_topics::COMMAND_ARM_TRAJECTORY; // "command/command_arm_sub_topic"
static const std::string kDefaultCommandMavArmTrajectoryTopic =
    mav_msgs::default_topics::COMMAND_MAVARM_TRAJECTORY; // "command/mavarm_trajectory"
}

#endif /* INCLUDE_ROTORS_CONTROL_COMMON_HITSZ_H_ */
