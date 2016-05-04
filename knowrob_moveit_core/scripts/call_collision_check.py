#!/usr/bin/env python

import sys
import rospy
from knowrob_moveit_msgs.srv import *
from sensor_msgs.msg import *

def get_param_or_die(param_name):
    if not rospy.has_param(param_name):
        rospy.logerr("Could not find parameter '%s' on the server.", param_name)
        sys.exit(0) 
    return rospy.get_param(param_name)

def joint_map_to_msg(joint_map):
    msg = JointState()

    
    return msg

if __name__ == "__main__":
    rospy.init_node('call_check_collisions', log_level=rospy.INFO)

    urdf_model = get_param_or_die('/robot_description')
    srdf_model = get_param_or_die('/moveit_config')
    collision_free_config = get_param_or_die('/collision_free_config')

    rospy.loginfo("Read config: %s", collision_free_config)

    rospy.wait_for_service('/planning_scene/check_collisions')
    try:
        check_collisions = rospy.ServiceProxy('/planning_scene/check_collisions', CheckCollisions)
        response = check_collisions(urdf_model, srdf_model, joint_map_to_msg(collision_free_config), [])
        rospy.loginfo("Detected contacts: %s", len(response.contacts))
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    sys.exit(0)
