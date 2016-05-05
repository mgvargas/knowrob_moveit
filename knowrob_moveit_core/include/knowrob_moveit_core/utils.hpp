/*
 * Copyright (c) 2016, Georg Bartels <georg.bartels@cs.uni-bremen.de>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of Institute for Artificial Intelligence nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef KNOWROB_MOVEIT_CORE_UTILS_HPP 
#define KNOWROB_MOVEIT_CORE_UTILS_HPP 

#include <moveit_msgs/ContactInformation.h>
#include <moveit/collision_detection/collision_tools.h>

// TODO: make argument const
inline std::vector<moveit_msgs::ContactInformation> collisionResultToMsg(
    collision_detection::CollisionResult& collision_result)
{
  std::vector<moveit_msgs::ContactInformation> result;

  for (collision_detection::CollisionResult::ContactMap::iterator it=collision_result.contacts.begin(); 
       it!=collision_result.contacts.end(); ++it)
    for (std::vector<collision_detection::Contact>::iterator it2=it->second.begin(); 
         it2 != it->second.end(); ++it2)
    {
      moveit_msgs::ContactInformation msg;
      collision_detection::contactToMsg(*it2, msg);
      result.push_back(msg);
    }

  ROS_INFO("Finished copying contacts.");

  return result;
}

#endif // KNOWROB_MOVEIT_CORE_UTILS_HPP
