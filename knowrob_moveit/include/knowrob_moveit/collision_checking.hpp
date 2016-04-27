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

#ifndef KNOWROB_MOVEIT_COLLISION_CHECKER_HPP 
#define KNOWROB_MOVEIT_COLLISION_CHECKER_HPP 

#include <moveit/planning_scene/planning_scene.h>
#include <sensor_msgs/JointState.h>
#include <vector>

namespace knowrob_moveit
{
  // TODO: try to reuse sth from MoveIt instead of this
  class Collision {};

  // TODO: try to reuse sth from MoveIt instead of this
  class CollisionObject {};

  // TODO: try to reuse sth from MoveIt instead of this
  class CollisionChecker
  {
    public:
      CollisionChecker()
      {
        // TODO: implement me
      }
      ~CollisionChecker() {}
  
      void set_robot() 
      {
        // TODO: implement me
      }

      void set_collision_objects(const std::vector<CollisionObject>& objects)
      {
        // TODO: implement me
      }

      void set_joint_states(const sensor_msgs::JointState& joint_state)
      {
        // TODO: implement me
      }

      std::vector<Collision> get_collisions() const
      {

      }
  
    private:
      planning_scene::PlanningScenePtr planning_scene_ptr_;
  
  };
}
#endif // KNOWROB_MOVEIT_COLLISION_CHECKER_HPP
