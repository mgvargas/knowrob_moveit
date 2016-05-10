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

#include <knowrob_moveit_core/planning_scene.hpp>
#include <knowrob_moveit_core/utils.hpp>
#include <knowrob_moveit_msgs/CheckCollisions.h>

namespace knowrob_moveit_core
{
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
      PlanningScene planning_scene_;

      bool callback(knowrob_moveit_msgs::CheckCollisions::Request& request, 
          knowrob_moveit_msgs::CheckCollisions::Response& response)
      {
        ROS_INFO("KnowRob-MoveIt check_collisions called.");
      
        try{
          // TODO: use collision objects
          response.contacts = planning_scene_.checkCollisions(request.urdf_model, request.srdf_model,
              request.joint_states);
        }
        catch(const std::exception& e)
        {
          ROS_ERROR("%s", e.what());
          return false;
        }

        ROS_INFO("KnowRob-MoveIt check_collisions finished.");
 
        return true;
      }
  
  };
}

#endif // KNOWROB_MOVEIT_CORE_PLANNING_SCENE_SERVER_HPP
