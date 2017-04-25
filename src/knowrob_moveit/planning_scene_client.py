import rospy
import copy
from knowrob_moveit.msg import ContactList
from sensor_msgs.msg import JointState
from knowrob_moveit.srv import *


def read_param(nh, parameter_name):
    if not rospy.get_param(parameter_name, nh):
        rospy.logerr('Could not find parameter {} in namespace {}. Aborting.',
                     format(parameter_name, rospy.get_namespace()))
    result = rospy.get_param(parameter_name, nh)
    return result

class PlanningSceneClient:
    def __init__(self):
        # node handle:  'nh'
        self.robot_description_ = rospy.get_param("/robot_description")
        self.moveit_config_ = rospy.get_param("/moveit_config")
        self.collision_pub_ = rospy.Publisher('contacts', ContactList, queue_size=1)
        self.debug_pub_ = rospy.Publisher('debug_topic', JointState, queue_size=1)
        self.js_sub_ = rospy.Subscriber("/joint_states", JointState, self.callback)
        rospy.wait_for_service('/planning_scene_server/check_collisions')
        try:
            self.collision_service_ = rospy.ServiceProxy('/planning_scene_server/check_collisions', CheckCollisions)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        self.trigger_service_ = rospy.Service('trigger', UInt64Trigger, self.trigger_callback)

    def callback(self, message):
        self.joint_states_ = message.data

    def trigger_callback(self, request, response):
        self.debug_pub_.publish(self.joint_states_)
        srv = CheckCollisions()
        srv.urdf_model = self.robot_description_
        srv.srdf_model = self.moveit_config_
        srv.joint_states = self.joint_states_
        srv.max_contacts = request.data

        if self.collision_service_.call(srv):
            msg = ContactList()
            msg.contacts = response.contacts
            self.collision_pub_.publish(msg)
            response.success = True
            response.message = "Detected " + str(len(srv.size())) + " collisions."
            return True
        else:
            response.success = False
            response.message = "Service call to planning scene failed."
            return False


def main():
    rospy.init_node('planning_scene_client', anonymous=True)
    psclient = PlanningSceneClient()
    try:
        psclient.start()
    except (OSError, LookupError) as error:
        rospy.logerr('URDF not found in parameter server')
        print 'ERROR: ', error
        return 0
    rospy.spin()
    return 0