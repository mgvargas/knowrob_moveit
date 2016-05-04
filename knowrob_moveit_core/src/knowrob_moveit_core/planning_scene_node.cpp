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

#include <ros/ros.h>
#include <knowrob_moveit_core/knowrob_moveit_core.hpp>
#include <string>
#include <urdf/model.h>
#include <knowrob_moveit_msgs/CheckCollisions.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/collision_detection/collision_tools.h>

void print_infos(planning_scene::PlanningScene& ps)
{
  std::string root_joint_type =
      ps.getRobotModel()->getRootJoint()->getTypeName();
  std::string root_joint_name =
      ps.getRobotModel()->getRootJoint()->getName();
  std::string model_frame =
      ps.getRobotModel()->getModelFrame();

  ROS_INFO("The root joint is called '%s', it is of type '%s', and the model_frame is called '%s'.", 
      root_joint_name.c_str(), root_joint_type.c_str(), model_frame.c_str());
}

std::vector<moveit_msgs::ContactInformation> collisionResultToMsg(collision_detection::CollisionResult& collision_result)
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

std::vector<moveit_msgs::ContactInformation> calculate_collisions(planning_scene::PlanningScene& ps)
{
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.contacts = true;
  collision_request.max_contacts = 10;
  collision_result.clear();
  ps.checkCollision(collision_request, collision_result);

  ROS_INFO("KnowRob-MoveIt check_collisions finished.");

  return collisionResultToMsg(collision_result);
}

bool callback(knowrob_moveit_msgs::CheckCollisions::Request& request, 
    knowrob_moveit_msgs::CheckCollisions::Response& response)
{
  ROS_INFO("KnowRob-MoveIt check_collisions called.");

  boost::shared_ptr<urdf::Model> urdf_ptr(new urdf::Model());
  if (!urdf_ptr->initString(request.urdf_model))
  {
    ROS_ERROR("Could not parse given urdf. Aborting.");
    return false;
  }

  boost::shared_ptr<srdf::Model> srdf_ptr(new srdf::Model());
  if (!srdf_ptr->initString(*urdf_ptr, request.srdf_model))
  {
    ROS_ERROR("Could not parse given srdf. Aborting.");
    return false;
  }

  boost::shared_ptr<const srdf::Model> srdf_const_ptr = srdf_ptr;
  boost::shared_ptr<const urdf::Model> urdf_const_ptr = urdf_ptr;

  planning_scene::PlanningScene ps(urdf_const_ptr, srdf_const_ptr);

  moveit_msgs::PlanningScene diff_ps_msg;
  diff_ps_msg.is_diff = true;
  diff_ps_msg.robot_state.joint_state = request.joint_states;
  ps.setPlanningSceneDiffMsg(diff_ps_msg);

  response.contacts = calculate_collisions(ps);

  print_infos(ps);

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "planning_scene");
  ros::NodeHandle nh("~");

  ros::ServiceServer service = nh.advertiseService("check_collisions", callback);

  ROS_DEBUG("KnowRob-MoveIt planning scene up.");

  ros::spin();

  return 0;
}
