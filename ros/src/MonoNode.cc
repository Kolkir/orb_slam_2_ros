#include "MonoNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport (node_handle);

    MonoNode node (ORB_SLAM2::System::MONOCULAR, node_handle, image_transport);

    node.Init();

    ros::spin();

    ros::shutdown();

    return 0;
}


MonoNode::MonoNode (ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : Node (sensor, node_handle, image_transport) {
  image_subscriber = image_transport.subscribe ("/camera/image_raw", 1, &MonoNode::ImageCallback, this);
  camera_info_topic_ = "/camera/camera_info";
  odom_subscriber = node_handle.subscribe("odom", 1, &MonoNode::OdomCallback, this);
}


MonoNode::~MonoNode () {
}


void MonoNode::ImageCallback (const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr cv_in_ptr;
  try {
      cv_in_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msg->header.stamp;

  orb_slam_->TrackMonocular(cv_in_ptr->image,cv_in_ptr->header.stamp.toSec());

  Update ();
}

void MonoNode::OdomCallback (const nav_msgs::Odometry::ConstPtr& msg) {
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    transform.setRotation(tf2::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
    AddOdometry(transform, msg->header.stamp.toSec());
}
