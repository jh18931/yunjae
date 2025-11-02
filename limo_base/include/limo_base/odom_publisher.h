#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>

namespace limo_base {

class OdomPublisher {
public:
  explicit OdomPublisher(ros::NodeHandle nh, ros::NodeHandle pnh);

private:
  void odomCb(const nav_msgs::Odometry::ConstPtr&);
  void imuCb(const sensor_msgs::Imu::ConstPtr&);

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_odom_raw_, sub_imu_raw_;
  ros::Publisher  pub_odom_, pub_imu_;
  tf2_ros::TransformBroadcaster tf_br_;

  std::string odom_frame_, base_frame_;
  bool publish_tf_;

  // topic names
  std::string ugv_odom_topic_, ugv_imu_topic_;
  std::string odom_pub_topic_, imu_pub_topic_;
};

} // ns
