#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

namespace limo_base {

class LimoBaseNode {
public:
  explicit LimoBaseNode(ros::NodeHandle nh, ros::NodeHandle pnh);
  void spin();

private:
  void cmdCb(const geometry_msgs::Twist::ConstPtr&);

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_cmd_;
  ros::Publisher  pub_ugv_cmd_;
  ros::Publisher  pub_diag_;

  // params
  double v_max_, w_max_;
  double ax_max_, aw_max_;
  double deadman_;
  std::string ugv_cmd_topic_;
  std::string cmd_sub_topic_;

  // state
  geometry_msgs::Twist last_cmd_;
  ros::Time t_last_cmd_;
};

} // ns
