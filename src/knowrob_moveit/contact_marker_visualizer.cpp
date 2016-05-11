#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <knowrob_moveit/ContactList.h>

inline visualization_msgs::Marker toMarker(const moveit_msgs::ContactInformation& contact, 
    const std::string& ns, size_t id)
{
  visualization_msgs::Marker marker;
  marker.header = contact.header;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = contact.position; 
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.02; 
  marker.scale.y = 0.02; 
  marker.scale.z = 0.02;

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  return marker;
}

class ContactMarkerVisualizer
{
  public:
    ContactMarkerVisualizer(const ros::NodeHandle& nh) : nh_( nh ) {}
    ~ContactMarkerVisualizer() {}
    void start() 
    {
      pub_ = nh_.advertise<visualization_msgs::MarkerArray>("marker_array", 1);
      sub_ = nh_.subscribe("contacts", 1, &ContactMarkerVisualizer::callback, this);
    }

  private:
   ros::NodeHandle nh_;
   ros::Publisher pub_;
   ros::Subscriber sub_;

   void callback(const knowrob_moveit::ContactList::ConstPtr& msg)
   {
     visualization_msgs::MarkerArray markers;
     for(size_t i=0; i<msg->contacts.size(); ++i)
       markers.markers.push_back(toMarker(msg->contacts[i], "contact_information", i));
     pub_.publish(markers);
   }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "contact_marker_visualizer");
  ros::NodeHandle nh("~");
  ContactMarkerVisualizer cmv(nh);

  try
  {
    cmv.start();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
    return 0;
  }

  ros::spin();

  return 0;
}
