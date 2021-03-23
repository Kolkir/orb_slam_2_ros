#pragma once

#include <opencv2/imgproc/imgproc_c.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/core/types.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/core/core.hpp>

#include "Node.h"
#include "System.h"

class StereoNode : public Node {
 public:
  StereoNode(const ORB_SLAM2::System::eSensor sensor,
             ros::NodeHandle& node_handle);
  ~StereoNode();
  void ImageCallback(const sensor_msgs::ImageConstPtr& msgLeft,
                     const sensor_msgs::ImageConstPtr& msgRight);

 private:
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image>
      sync_pol;
  message_filters::Subscriber<sensor_msgs::Image>* left_sub_;
  message_filters::Subscriber<sensor_msgs::Image>* right_sub_;
  message_filters::Synchronizer<sync_pol>* sync_;

  int resize_horizontal;
  int resize_vertical;
};
