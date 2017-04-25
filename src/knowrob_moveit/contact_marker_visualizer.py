import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from knowrob_moveit.msg import ContactList


def to_marker(contact, ns, id):
    marker = Marker()
    marker.header = contact.header
    marker.ns = ns
    marker.id = id
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.02
    marker.scale.y = 0.02
    marker.scale.z = 0.02
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration()
    return marker


class ContactMarkerVisualizer:
    def __init__(self):
        # node handle: 'nh'
        self.pub = rospy.Publisher('visualization_marker_array',MarkerArray, queue_size=1)
        self.sub = rospy.Subscriber("contacts", ContactList, self.callback)

    def callback(self, msg):
        markerArray = MarkerArray()
        for i,contact in enumerate(msg.contacts):
            markerArray.markers.append(to_marker(contact, "contact_information", i))
        self.pub.publish(markerArray)
