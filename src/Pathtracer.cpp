#include <cmath>
#include <thread>
#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ozurover_messages/FollowPathAction.h>
#include "../includes/pid.h"

class Pathtracer {
protected:
  ros::NodeHandle _nh;
  ros::Publisher _controlPub;
  actionlib::SimpleActionServer<ozurover_messages::FollowPathAction> _pathServer;
  tf2_ros::Buffer _tfBuffer;
  tf2_ros::TransformListener _tfListener;
  ros::AsyncSpinner _spinner;

  // Internal state
  ozurover_messages::FollowPathGoalConstPtr _path;
  geometry_msgs::PoseStamped _refPose;
  geometry_msgs::PoseStamped _roverPose;
  geometry_msgs::Twist _control;
  
  void followServer(const ozurover_messages::FollowPathGoalConstPtr &goal) {
    ROS_INFO("Received path to follow");
    if (goal->path.poses.size() < 2) {
      ROS_ERROR("Path too short");
      _pathServer.setAborted();
      return;
    }
    _path = goal;
    _refPose = _path->path.poses.front();
  }

  void updateInternalState() {
    ros::Rate rate(400.0);
    while (ros::ok()) {
      rate.sleep();

      /* Update Rover Position*/
      try {
        geometry_msgs::TransformStamped transform = _tfBuffer.lookupTransform("map", "base_link", ros::Time::now());
        _roverPose.pose.position.x = transform.transform.translation.x;
        _roverPose.pose.position.y = transform.transform.translation.y;
        _roverPose.pose.position.z = transform.transform.translation.z;
        _roverPose.pose.orientation = transform.transform.rotation;
        break;
      } catch (tf2::TransformException &ex) {
        ROS_WARN("Could not fetch rover pose: %s", ex.what());
        rate.sleep();
      }

      /* Stand Down When Preempted  */
      if (_pathServer.isPreemptRequested()) {
        _path = NULL;
        _pathServer.setPreempted();
      }

      /* Stop If No Goal*/
      if (!_path) {
        continue;
      }


      /* Check If Goal Reached*/
      geometry_msgs::PoseStamped ref = geometry_msgs::PoseStamped(_path->path.poses.back());
      ref.pose.position.x -= _roverPose.pose.position.x;
      ref.pose.position.y -= _roverPose.pose.position.y;
      if (ref.pose.position.x * ref.pose.position.x + ref.pose.position.y * ref.pose.position.y < 4.0f) {
        ROS_INFO("Goal reached");
        _path = NULL;
        _pathServer.setSucceeded();
        continue;
      }

      /* Update Reference Position */
      geometry_msgs::PoseStamped newRef = geometry_msgs::PoseStamped(_path->path.poses.front());
      for (std::vector<geometry_msgs::PoseStamped>::const_iterator it = _path->path.poses.begin(); it != _path->path.poses.end(); ++it) {
        geometry_msgs::PoseStamped pose = *it;
        pose.pose.position.x -= _roverPose.pose.position.x;
        pose.pose.position.y -= _roverPose.pose.position.y;
        if (pose.pose.position.x * pose.pose.position.x + pose.pose.position.y * pose.pose.position.y < newRef.pose.position.x * newRef.pose.position.x + newRef.pose.position.y * newRef.pose.position.y) {
          newRef = pose;
        }
      }

    }
  }

  void updateCommandLoop() {
    PIDController pid;
    PIDController_Init(&pid);
    ros::Rate rate(400.0f);

    while (ros::ok()) {
      rate.sleep();
      if (!_path) {
        _control.angular.z = 0.0f;
        _control.linear.x = 0.0f;
        continue;
      }

      tf2::Vector3 refBearing(_roverPose.pose.position.x - _refPose.pose.position.x, _roverPose.pose.position.y - _refPose.pose.position.y, 0.0f);
      tf2::Vector3 refDirection = tf2::Vector3(cos(_refPose.pose.orientation.z), sin(_refPose.pose.orientation.z), 0.0f).normalized(); // TODO
      tf2::Vector3 errVector = refBearing - tf2::tf2Dot(refBearing, refDirection) * refDirection;

      float error = atan2(errVector[1], errVector[0]);
      float output = PIDController_Update(&pid, 0.0, error);
      _control.linear.x = 1.0f;
      _control.angular.z = output;
      
    }
  }

  void publishCommandLoop() {
    ros::Rate rate(400.0f);
    while (ros::ok() && _path) {
      _controlPub.publish(_control);
      rate.sleep();
    }
  }

public:
  Pathtracer() :
    _tfListener(_tfBuffer),
    _spinner(1),
    _pathServer(_nh, "ares/follow_path", boost::bind(&Pathtracer::followServer, this, _1), false)
  {
    _roverPose.header.frame_id = "map";
    _refPose.header.frame_id = "map";
    _control.angular.z = 0.0f;
    _control.linear.x = 1.0f;
    _controlPub = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    _pathServer.start();
  }

  void run() {
    _spinner.start();
    _pathServer.start();
    std::thread updateThread(&Pathtracer::updateInternalState, this);
    updateThread.detach();
    updateCommandLoop();
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pathtracer");
  Pathtracer tracer;
  tracer.run();
  return 0;
}