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

#ifndef DEFAULT_TOPICS_HITSZ_H
#define DEFAULT_TOPICS_HITSZ_H

#include "mav_msgs/default_topics.h"
namespace mav_msgs {
namespace default_topics {

static constexpr char COMMAND_ARM_TRAJECTORY[] = "command/command_arm_sub_topic";
static constexpr char COMMAND_MAVARM_TRAJECTORY[] = "command/mavarm_trajectory";
static constexpr char COMMAND_CURRENT_REFERENCE[] = "command/current_reference";

} // end namespace default_topics
} // end namespace mav_msgs

#endif /* DEFAULT_TOPICS_HITSZ_H */
