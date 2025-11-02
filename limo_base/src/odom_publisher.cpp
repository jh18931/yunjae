#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "limo_base/odom_publisher.h"
#include "limo_base/utils.h"

namespace limo_base {

OdomPublisher::OdomPublisher(ros::NodeHandle nh, ros::NodeHandle pnh)
: nh_(nh), pnh_(pnh)
{
  odom_frame_ = paramOr(pnh_, "odom_frame", std::string("odom"));
  base_frame_ = paramOr(pnh_, "base_link_frame", std::string("base_link"));
  publish_tf_ = paramOr(pnh_, "publish_tf", true);

  ugv_odom_topic_ = paramOr(pnh_, "ugv_odom_topic", std::string("/ugv_sdk/odom"));
  ugv_imu_topic_  = paramOr(pnh_, "ugv_imu_topic",  std::string("/ugv_sdk/imu/data"));
  odom_pub_topic_ = paramOr(pnh_, "odom_pub_topic", std::string("/odom"));
  imu_pub_topic_  = paramOr(pnh_, "imu_pub_topic",  std::string("/imu/data"));

  sub_odom_raw_ = nh_.subscribe(ugv_odom_topic_, 20, &OdomPublisher::odomCb, this);
  sub_imu_raw_  = nh_.subscribe(ugv_imu_topic_,  50, &OdomPublisher::imuCb, this);
  pub_odom_     = nh_.advertise<nav_msgs::Odometry>(odom_pub_topic_, 20);
  pub_imu_      = nh_.advertise<sensor_msgs::Imu>(imu_pub_topic_,  50);

  ROS_INFO_STREAM("[odom_publisher] " << ugv_odom_topic_ << " -> " << odom_pub_topic_
                  << ", " << ugv_imu_topic_ << " -> " << imu_pub_topic_
                  << ", tf(" << odom_frame_ << "->" << base_frame_ << ")=" << (publish_tf_?"on":"off"));
}

void OdomPublisher::odomCb(const nav_msgs::Odometry::ConstPtr& msg_in)
{
  nav_msgs::Odometry msg = *msg_in;
  msg.header.frame_id = odom_frame_;
  msg.child_frame_id  = base_frame_;
  pub_odom_.publish(msg);

  if (publish_tf_) {
    geometry_msgs::TransformStamped tf;
    tf.header = msg.header;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = msg.pose.pose.position.x;
    tf.transform.translation.y = msg.pose.pose.position.y;
    tf.transform.translation.z = msg.pose.pose.position.z;
    tf.transform.rotation      = msg.pose.pose.orientation;
    tf_br_.sendTransform(tf);
  }
}

void OdomPublisher::imuCb(const sensor_msgs::Imu::ConstPtr& msg_in)
{
  sensor_msgs::Imu msg = *msg_in;
  msg.header.frame_id = base_frame_; // IMU는 보통 base_link 기준
  pub_imu_.publish(msg);
}

} // ns

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_publisher");
  ros::NodeHandle nh, pnh("~");
  limo_base::OdomPublisher node(nh, pnh);
  ros::spin();
  return 0;
}
