#include <cmath>
#include <thread>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <ozurover_locomotion/PathtracerPIDConfig.h>
#include <actionlib/server/simple_action_server.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <ozurover_messages/FollowPathAction.h>
#include "../includes/pid.h"

#define CONTROL_FREQ 200.0f

class Pathtracer {
protected:
  ros::NodeHandle _nh;
  ros::Publisher _controlPub;
  ros::Publisher _refPub;
  ros::Publisher _errPub;
  ros::ServiceServer _pidResetService;
  actionlib::SimpleActionServer<ozurover_messages::FollowPathAction> _pathServer;
  dynamic_reconfigure::Server<ozurover_locomotion::PathtracerPIDConfig> _cfgServer;
  dynamic_reconfigure::Server<ozurover_locomotion::PathtracerPIDConfig>::CallbackType _cfgCallback;
  tf2_ros::Buffer _tfBuffer;
  tf2_ros::TransformListener _tfListener;
  ros::AsyncSpinner _spinner;

  // Internal state
  PIDController pid;
  ozurover_messages::FollowPathGoalConstPtr _path;
  geometry_msgs::PoseStamped _refPose;
  geometry_msgs::PoseStamped _roverPose;
  geometry_msgs::Twist _control;
  bool _isTracing;

  //pid
  float kp, ki, kd;

  bool resetPIDCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    PIDController_Reset(&pid);
    return true;
  }
  
  void dynamicReconfigureCallback(ozurover_locomotion::PathtracerPIDConfig &config, uint32_t level) {
    kp = config.kp;
    ki = config.ki;
    kd = config.kd;
  }
  
  void followServer(const ozurover_messages::FollowPathGoalConstPtr &goal) {
    ROS_INFO("Received path to follow");
    if (goal->path.poses.size() < 2) {
      ROS_ERROR("Path too short");
      _pathServer.setAborted();
      return;
    }

    _path = goal;
    _refPose = _path->path.poses.front();
    _isTracing = true;

    ros::Rate rate(1.0f);
    while (_isTracing) {
      rate.sleep();
      if (_pathServer.isPreemptRequested()) {
        ROS_INFO("Path following preempted");
        _isTracing = false;
        _path = NULL;
        _pathServer.setPreempted();
        return;
      }
    }
  }

  void updateInternalState() {
    ros::Rate rate(CONTROL_FREQ);
    while (ros::ok()) {
      rate.sleep();

      /* Update Rover Position*/
      try {
        geometry_msgs::TransformStamped transform = _tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
        _roverPose.pose.position.x = transform.transform.translation.x;
        _roverPose.pose.position.y = transform.transform.translation.y;
        _roverPose.pose.position.z = transform.transform.translation.z;
        _roverPose.pose.orientation = transform.transform.rotation;
        ROS_INFO("Rover pose: %f, %f", _roverPose.pose.position.x, _roverPose.pose.position.y);
      } catch (tf2::TransformException &ex) {
        ROS_WARN("Could not fetch rover pose: %s", ex.what());
        rate.sleep();
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
        float dx1 = it->pose.position.x - _roverPose.pose.position.x;
        float dy1 = it->pose.position.y - _roverPose.pose.position.y;
        float dx2 = newRef.pose.position.x - _roverPose.pose.position.x;
        float dy2 = newRef.pose.position.y - _roverPose.pose.position.y;
        if (dx1 * dx1 + dy1 * dy1 < dx2 * dx2 + dy2 * dy2) {
          newRef = *it;
        }
      }
      _refPose = newRef;
      _refPub.publish(_refPose);
    }
  }

  void updateCommandLoop() {
    PIDController_Init(&pid, 1.0f/CONTROL_FREQ);
    ros::Rate rate(CONTROL_FREQ);
    std_msgs::Float32 err;

    while (ros::ok()) {
      rate.sleep();
      if (!_path) {
        _control.angular.z = 0.0f;
        _control.linear.x = 0.0f;
        continue;
      }

      tf2::Vector3 line(1.0f, 0.0f, 0.0f);
      tf2::Vector3 errVector(_roverPose.pose.position.x - _refPose.pose.position.x, _roverPose.pose.position.y - _refPose.pose.position.y, 0.0f);
      tf2::Quaternion q;
      tf2::fromMsg(_refPose.pose.orientation, q);
      line = tf2::quatRotate(q, line);
      tf2::Vector3 cross = line.cross(errVector);

      float error = cross.getZ();
      float output = PIDController_Update(&pid, 0.0, error, kp, ki, kd);
      _control.angular.z = atan(output);
      _control.linear.x = 1.0f;

      err.data = error;
      _errPub.publish(err);
    }
  }

  void publishCommandLoop() {
    geometry_msgs::Twist _controlBackup;
    _controlBackup.linear.x = 0.0f;
    _controlBackup.angular.z = 0.0f;

    ros::Rate rate(CONTROL_FREQ);
    while (ros::ok()) {
      rate.sleep();
      _controlPub.publish(_control);
    }
  }

public:
  Pathtracer() :
    _isTracing(false),
    _tfListener(_tfBuffer),
    _spinner(1),
    _pathServer(_nh, "ares/follow_path", boost::bind(&Pathtracer::followServer, this, _1), false),
    kp(1.0f),
    ki(0.0f),
    kd(0.0f)
  {
    _pidResetService = _nh.advertiseService("/ares/reset_pid", &Pathtracer::resetPIDCallback, this);
    _cfgCallback = boost::bind(&Pathtracer::dynamicReconfigureCallback, this, _1, _2);
    _cfgServer.setCallback(_cfgCallback);
    _roverPose.header.frame_id = "map";
    _refPose.header.frame_id = "map";
    _control.angular.z = 0.0f;
    _control.linear.x = 0.0f;
    _controlPub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    _refPub = _nh.advertise<geometry_msgs::PoseStamped>("/ares/ref_pose", 1);
    _errPub = _nh.advertise<std_msgs::Float32>("/ares/error", 1); 
    _pathServer.start();
  }

  void run() {
    _spinner.start();
    _pathServer.start();
    std::thread updateThread(&Pathtracer::updateInternalState, this);
    updateThread.detach();
    std::thread updateCommandThread(&Pathtracer::updateCommandLoop, this);
    updateCommandThread.detach();
    publishCommandLoop();
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pathtracer");
  Pathtracer tracer;
  tracer.run();
  return 0;
}