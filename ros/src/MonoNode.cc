#include "MonoNode.h"
#include "System.h"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nodelet/loader.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/core/core.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "orb_slam2_ros");

  nodelet::Loader manager(true);
  nodelet::M_string remappings(ros::names::getRemappings());
  nodelet::V_string my_argv(argv + 1, argv + argc);

  manager.load(ros::this_node::getName(), "orb_slam2_ros/MonoSlam", remappings,
               my_argv);

  ros::spin();
}

namespace orb_slam2_ros {

MonoNode::MonoNode() : Node(ORB_SLAM2::System::MONOCULAR) {
  NODELET_INFO_STREAM("ORB SLAM node constructor");
}

void MonoNode::onInit() {
  name_of_node_ = getName();
  sensor_ = ORB_SLAM2::System::MONOCULAR;

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  NODELET_INFO_STREAM("ORB SLAM node " << name_of_node_
                                       << " initialization ...");

  image_transport::ImageTransport image_transport(getNodeHandle());

  camera_info_topic_ = "/camera/camera_info";

  Init(getNodeHandle(), image_transport);

  image_subscriber_ = image_transport.subscribe("/camera/image_raw", 1,
                                                &MonoNode::ImageCallback, this);

  const std::string odom_topic = "/odom";

  NODELET_INFO_STREAM("Listening for odom on topic "
                      << getNodeHandle().resolveName(odom_topic));
  odom_subscriber_ =
      getNodeHandle().subscribe(odom_topic, 1, &MonoNode::OdomCallback, this);
}

MonoNode::~MonoNode() {}

void MonoNode::ImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr cv_in_ptr;
  try {
    cv_in_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  current_frame_time_ = msg->header.stamp;

  orb_slam_->TrackMonocular(cv_in_ptr->image, cv_in_ptr->header.stamp.toSec());

  Update();
}

void MonoNode::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(msg->pose.pose.position.x,
                                   msg->pose.pose.position.y,
                                   msg->pose.pose.position.z));
  transform.setRotation(tf2::Quaternion(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
  AddOdometry(transform, msg->header.stamp.toSec());
}
}  // namespace orb_slam2_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(orb_slam2_ros::MonoNode, nodelet::Nodelet)
