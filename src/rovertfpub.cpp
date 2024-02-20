#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define dT 0.001
#define SIM_FREQ 100.0f

geometry_msgs::TransformStamped _transformStamped;
geometry_msgs::PoseStamped _roverPose;
geometry_msgs::Twist _commandVel;

void cmdCallback(const geometry_msgs::TwistConstPtr &msg) {
  ROS_INFO("Received command velocity");
  _commandVel = *msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rover_tf_pub");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/cmd_vel", 10, &cmdCallback);
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("ares/pose", 10);
  ros::AsyncSpinner spinner(1);

  _roverPose.header.frame_id = "map";
  _roverPose.pose.position.x = 0;
  _roverPose.pose.position.y = 0;
  _roverPose.pose.position.z = 0;
  _roverPose.pose.orientation.x = 0;
  _roverPose.pose.orientation.y = 0;
  _roverPose.pose.orientation.z = 0;
  _roverPose.pose.orientation.w = 1;

  _transformStamped.header.frame_id = "map";
  _transformStamped.child_frame_id = "base_link";
  _transformStamped.transform.translation.x = 0;
  _transformStamped.transform.translation.y = 0;
  _transformStamped.transform.translation.z = 0;
  _transformStamped.transform.rotation.x = 0;
  _transformStamped.transform.rotation.y = 0;
  _transformStamped.transform.rotation.z = 0;
  _transformStamped.transform.rotation.w = 1;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformBroadcaster tfBroadcaster;
  tf2_ros::TransformListener tfListener(tfBuffer);

  spinner.start();

  ros::Rate rate(SIM_FREQ);
  while (nh.ok()) {
    _roverPose.header.stamp = ros::Time::now();
    _transformStamped.header.stamp = ros::Time::now();

    if (_commandVel.angular.z != 0.0f) {
      tf2::Quaternion q1, q2;
      q1.setRPY(0, 0, _commandVel.angular.z * dT);
      tf2::fromMsg(_roverPose.pose.orientation, q2);
      q1 = q1 * q2;
      q1.normalize();
      _roverPose.pose.orientation = tf2::toMsg(q1);
    }

    /* Update poses and rotation. */
    double yaw = tf2::getYaw(_roverPose.pose.orientation);
    _roverPose.pose.position.x += _commandVel.linear.x * cos(yaw) * dT;
    _roverPose.pose.position.y += _commandVel.linear.x * sin(yaw) * dT;

    /* Set and publish transform*/
    _transformStamped.transform.translation.x = _roverPose.pose.position.x;
    _transformStamped.transform.translation.y = _roverPose.pose.position.y;
    _transformStamped.transform.rotation = _roverPose.pose.orientation;

    pub.publish(_roverPose);
    tfBroadcaster.sendTransform(_transformStamped);
    rate.sleep();
  }
  return 0;
}