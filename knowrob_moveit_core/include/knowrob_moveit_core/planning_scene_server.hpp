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
 *   contributors may be used to endorse or promote products derived from *   this software without specific prior written permission.
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

#ifndef KNOWROB_MOVEIT_CORE_PLANNING_SCENE_SERVER_HPP 
#define KNOWROB_MOVEIT_CORE_PLANNING_SCENE_SERVER_HPP 

#include <knowrob_moveit_core/utils.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <knowrob_moveit_msgs/CheckCollisions.h>
#include <vector>
#include <functional>

namespace knowrob_moveit_core
{
  // TODO: separate into ROS-dependent and ROS-agnostic functionality
  class PlanningSceneServer
  {
    public:
  
      PlanningSceneServer(const ros::NodeHandle& nh) : nh_( nh )
      {}
    
      ~PlanningSceneServer() {}
  
      void start()
      {
        service_ = nh_.advertiseService("check_collisions", &PlanningSceneServer::callback, this);
      }
  
    private:
      ros::NodeHandle nh_;
      ros::ServiceServer service_;
      size_t urdf_hash_, srdf_hash_;
  
      planning_scene::PlanningScenePtr ps_ptr_;
  
      // TODO: see whether it is also fine to just remember the robot model
      void createPlanningScene(const std::string& urdf, const std::string& srdf)
      {
        ROS_INFO("Hashing started.");
        std::hash<std::string> hash_fn;
        size_t new_urdf_hash = hash_fn(urdf);
        size_t new_srdf_hash = hash_fn(srdf);
        ROS_INFO("Hashing done.");
  
        if(!ps_ptr_ || urdf_hash_ != new_urdf_hash || srdf_hash_ != new_srdf_hash)
        {
          ROS_INFO("Parsing urdf started.");
          boost::shared_ptr<urdf::Model> urdf_ptr(new urdf::Model());
          if (!urdf_ptr->initString(urdf))
            throw std::runtime_error("Could not parse given urdf. Aborting.");
          ROS_INFO("Parsing urdf done.");
  
          ROS_INFO("Parsing srdf started.");
          boost::shared_ptr<srdf::Model> srdf_ptr(new srdf::Model());
          if (!srdf_ptr->initString(*urdf_ptr, srdf))
            throw std::runtime_error("Could not parse given srdf. Aborting.");
          ROS_INFO("Parsing srdf done.");
     
          ROS_INFO("Creation planning scene started.");
          boost::shared_ptr<const srdf::Model> srdf_const_ptr = srdf_ptr;
          boost::shared_ptr<const urdf::Model> urdf_const_ptr = urdf_ptr;
      
          ps_ptr_ = planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(urdf_const_ptr, srdf_const_ptr));
          urdf_hash_ = new_urdf_hash;
          srdf_hash_ = new_srdf_hash;
          ROS_INFO("Creation planning scene done.");
        }
      }
  
      std::vector<moveit_msgs::ContactInformation> calculate_collisions()
      {
        ROS_INFO("KnowRob-MoveIt check_collisions started.");
      
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        // TODO: check whether those are useful defaults
        collision_request.contacts = true;
        collision_request.max_contacts = 10;
        collision_result.clear();
        ps_ptr_->checkCollision(collision_request, collision_result);
      
        ROS_INFO("KnowRob-MoveIt check_collisions finished.");
      
        return collisionResultToMsg(collision_result);
      }

      bool callback(knowrob_moveit_msgs::CheckCollisions::Request& request, 
          knowrob_moveit_msgs::CheckCollisions::Response& response)
      {
        ROS_INFO("KnowRob-MoveIt check_collisions called.");
      
        try{
          createPlanningScene(request.urdf_model, request.srdf_model);
        }
        catch(const std::exception& e)
        {
          ROS_ERROR("%s", e.what());
          return false;
        }
  
        ROS_INFO("Update planning scene started.");
        moveit_msgs::PlanningScene diff_ps_msg;
        diff_ps_msg.is_diff = true;
        diff_ps_msg.robot_state.joint_state = request.joint_states;
        ps_ptr_->setPlanningSceneDiffMsg(diff_ps_msg);
        ROS_INFO("Update planning scene done.");
     
        response.contacts = calculate_collisions();
      
        return true;
      }
  
  };
}

#endif // KNOWROB_MOVEIT_CORE_PLANNING_SCENE_SERVER_HPP
