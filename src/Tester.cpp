#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/PointStamped.h>
#include <ozurover_messages/Pathfind.h>
#include <ozurover_messages/FollowPathAction.h>

class PathtracingTester {
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::ServiceClient client_;
  actionlib::SimpleActionClient<ozurover_messages::FollowPathAction> ac_;
  
  void pointCallback(const geometry_msgs::PointStampedConstPtr &msg) {
    geometry_msgs::PoseStamped rover;
    rover.header.frame_id = "map";
    rover.pose.position.x = 0;
    rover.pose.position.y = 0;
    rover.pose.position.z = 0;
    rover.pose.orientation.x = 0;
    rover.pose.orientation.y = 0;
    rover.pose.orientation.z = 0;
    rover.pose.orientation.w = 1;

    geometry_msgs::PoseStamped endpoint;
    endpoint.header.frame_id = "map";
    endpoint.pose.position.x = msg->point.x;
    endpoint.pose.position.y = msg->point.y;
    endpoint.pose.position.z = msg->point.z;
    endpoint.pose.orientation.x = 0;
    endpoint.pose.orientation.y = 0;
    endpoint.pose.orientation.z = 0;
    endpoint.pose.orientation.w = 1;

    ozurover_messages::Pathfind srv;
    srv.request.rover = rover;
    srv.request.goal = endpoint;
    if (client_.call(srv)) {
      ozurover_messages::FollowPathGoal goal;
      pub_.publish(srv.response.path);
      goal.path = srv.response.path;
      ac_.sendGoal(goal);
    } else {
      ROS_ERROR("Failed to call service pathfind");
    }
  }

public:
  PathtracingTester(std::string name) :
  ac_("ares/follow_path", true)
  {
    pub_ = nh_.advertise<nav_msgs::Path>("/ares/path", 1);
    sub_ = nh_.subscribe("/clicked_point", 1, &PathtracingTester::pointCallback, this);
    client_ = nh_.serviceClient<ozurover_messages::Pathfind>("/pathfind");
    ac_.waitForServer();
  }

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pathtracing_tester");
  PathtracingTester tester("pathtracing_tester");
  ros::spin();
  return 0;
}