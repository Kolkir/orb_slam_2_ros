/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
 * of Zaragoza) For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ORBSLAM2_ROS_MONONODE_H_
#define ORBSLAM2_ROS_MONONODE_H_

#include "Node.h"

#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

namespace orb_slam2_ros {

class MonoNode : public Node, public nodelet::Nodelet {
 public:
  MonoNode();

  ~MonoNode() override;
  void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
  void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);

 private:
  // nodelet::Nodelet implementation
  void onInit() override;

 private:
  image_transport::Subscriber image_subscriber_;
  ros::Subscriber odom_subscriber_;
};

}  // namespace orb_slam2_ros

#endif  // ORBSLAM2_ROS_MONONODE_H_
