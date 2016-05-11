#include <ros/ros.h>
#include <knowrob_moveit_msgs/CheckCollisions.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <stdexcept>

std::string readParam(const ros::NodeHandle& nh, const std::string& parameter_name)
{
  std::string result;
  if (!nh.getParam(parameter_name, result))
    throw std::runtime_error("Could not find parameter '" +
        parameter_name + "' in namespace '" + nh.getNamespace() +"'. Aborting.");

  return result;
}

class PlanningSceneClient
{
  public:
    PlanningSceneClient(const ros::NodeHandle& nh) : nh_( nh )
    {}
  
    ~PlanningSceneClient() {}

    void start()
    {
     robot_description_ = readParam(nh_, "/robot_description");
     moveit_config_ = readParam(nh_, "/moveit_config");
     nh_.subscribe("/joint_states", 1, &PlanningSceneClient::callback, this);
     collision_service_ = nh_.serviceClient<knowrob_moveit_msgs::CheckCollisions>("/planning_scene_server/check_collisions");
     trigger_service_ = nh_.advertiseService("trigger", &PlanningSceneClient::trigger_callback, this);
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber js_sub_;
    ros::ServiceServer trigger_service_;
    ros::ServiceClient collision_service_;
    sensor_msgs::JointState joint_states_;
    std::string robot_description_, moveit_config_;


    void callback(const sensor_msgs::JointStateConstPtr& message)
    {
      joint_states_ = *message;
    }

    bool trigger_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
    {
      knowrob_moveit_msgs::CheckCollisions srv;
      srv.request.urdf_model = robot_description_;
      srv.request.srdf_model = moveit_config_;
      srv.request.joint_states = joint_states_;
     
      if (collision_service_.call(srv))
      {
        ROS_INFO("Detected %lu collisions.", srv.response.contacts.size());
        return true;
      }
      else
        return false;
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planning_scene_client");
  ros::NodeHandle nh("~");
  PlanningSceneClient psclient(nh);

  try
  {
    psclient.start();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
    return 0;
  }

  ros::spin();

  return 0;
}
