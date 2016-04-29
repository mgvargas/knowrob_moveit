#!/usr/bin/env python

import sys
import rospy
from knowrob_moveit_msgs.srv import *
from sensor_msgs.msg import *

if __name__ == "__main__":
    rospy.init_node('call_check_collisions', log_level=rospy.INFO)

    if not rospy.has_param('/robot_description'):
        rospy.logerr("Could not find parameter '/robot_description' on the server.")
        sys.exit(0) 
    urdf_model = rospy.get_param('/robot_description')
    
    rospy.wait_for_service('/planning_scene/check_collisions')
    try:
        check_collisions = rospy.ServiceProxy('/planning_scene/check_collisions', CollisionCheck)
        response = check_collisions(urdf_model, JointState(), [])
        rospy.loginfo("Detected contacts: %s", response.contacts)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    sys.exit(0)
