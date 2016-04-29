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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planning_scene");
  ros::NodeHandle nh("~");

  std::string robot_description;
  if (!nh.getParam("robot_description", robot_description))
  {
    ROS_ERROR("Parameter 'robot_description' not found in namespace '%s'.", 
        nh.getNamespace().c_str());
    return 0;
  }

  urdf::Model urdf;
  if (!urdf.initString(robot_description))
  {
    ROS_ERROR("Could not parse robot description.");
    return 0;
  }

  srdf::Model srdf;
//  std::string empty_srdf_spec = "<?xml version=\"1.0\"?><robot name=\"pr2\"></robot>";
//  if (!srdf.initString(urdf, empty_srdf_spec))
//  {
//    ROS_ERROR("Could not initialize semantic robot model from specification.");
//    return 0;
//  }

  planning_scene::PlanningScene planning_scene(
      boost::shared_ptr<const urdf::Model>(&urdf),
      boost::shared_ptr<const srdf::Model>(&srdf));

  // TODO: implement me
  ROS_DEBUG("KnowRob-MoveIt collision checker up.");
  ros::spin();

  return 0;
}
