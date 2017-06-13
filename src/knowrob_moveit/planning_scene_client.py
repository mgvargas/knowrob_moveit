import rospy
import moveit_commander
import rospkg
import xml.etree.ElementTree as element_tree
from knowrob_moveit.msg import ContactList
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from urdf_parser_py.urdf import URDF
from knowrob_moveit.srv import *


def make_header(stamp,frame_id):
    msg = Header()
    msg.stamp = stamp
    msg.frame_id = frame_id
    return msg


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

        # From header
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.urdf_hash_ = None
        self.srdf_hash_ = None
        self.ps_ptr_ = None

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
            response.message = "Detected " + str(len(response.contacts)) + " collisions."
            return True
        else:
            response.success = False
            response.message = "Service call to planning scene failed."
            return False

    # From header file
    def check_collisions(self, urdf_model, srdf_model, joint_states, max_contacts):
        self.create_planning_scene(urdf_model, srdf_model)
        return self.calculate_collisions(max_contacts, joint_states)

    def create_planning_scene(self, urdf, srdf):
        rospy.loginfo("Hashing started.")
        new_urdf_hash = hash(urdf)
        new_srdf_hash = hash(srdf)
        rospy.loginfo("Hashing done.")
        if not self.ps_ptr_ or self.urdf_hash_ != new_urdf_hash or self.srdf_hash_ != new_srdf_hash:
            rospy.loginfo("Parsing urdf started.")
            rospack = rospkg.RosPack()
            dir = rospack.get_path('knowrob_moveit') + '/test_data/boxy.urdf'
            try:
                # self.urdf_ptr = urdf.Robot.from_xml_file(dir)
                # self.urdf_ptr = URDF.from_parameter_server()
                self.urdf_ptr = element_tree.parse(urdf)
            except (OSError, LookupError) as error:
                rospy.logerr('Could not parse given urdf. Aborting.')
                print 'ERROR: ', error
            rospy.loginfo("Parsing urdf done.")

            rospy.loginfo("Parsing srdf started.")
            try:
                self.srdf_ptr = element_tree.parse(srdf)
            except (OSError, LookupError) as error:
                rospy.logerr('Could not parse given srdf. Aborting.')
                print 'ERROR: ', error
            rospy.loginfo("Parsing srdf done.")

            rospy.loginfo("Creation planning scene started.")

            self.urdf_hash_ = new_urdf_hash
            self.srdf_hash_ = new_srdf_hash
            rospy.loginfo("Creation planning scene done.")


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