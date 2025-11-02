#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include "limo_base/limo_base_node.h"
#include "limo_base/utils.h"

using diagnostic_msgs::DiagnosticArray;
using diagnostic_msgs::DiagnosticStatus;
using diagnostic_msgs::KeyValue;

namespace limo_base {

LimoBaseNode::LimoBaseNode(ros::NodeHandle nh, ros::NodeHandle pnh)
: nh_(nh), pnh_(pnh)
{
  v_max_  = paramOr(pnh_, "linear_vel_limit", 0.6);
  w_max_  = paramOr(pnh_, "angular_vel_limit", 1.2);
  ax_max_ = paramOr(pnh_, "acc_lim_x", 0.8);
  aw_max_ = paramOr(pnh_, "acc_lim_theta", 1.5);
  deadman_= paramOr(pnh_, "deadman_timeout", 0.5);

  ugv_cmd_topic_ = paramOr(pnh_, "ugv_cmd_topic", std::string("/ugv_sdk/cmd_vel"));
  cmd_sub_topic_ = paramOr(pnh_, "cmd_sub_topic", std::string("/cmd_vel"));

  sub_cmd_    = nh_.subscribe(cmd_sub_topic_, 10, &LimoBaseNode::cmdCb, this);
  pub_ugv_cmd_= nh_.advertise<geometry_msgs::Twist>(ugv_cmd_topic_, 10);
  pub_diag_   = nh_.advertise<DiagnosticArray>("/diagnostics", 1);

  t_last_cmd_ = ros::Time::now();
  ROS_INFO_STREAM("[limo_base_node] bridge " << cmd_sub_topic_ << " -> " << ugv_cmd_topic_);
}

void LimoBaseNode::cmdCb(const geometry_msgs::Twist::ConstPtr& msg)
{
  // 속도/가속 제한 (간단: 현재값만 clamp; 필요하면 가속 제한은 상위 cmd_limiter 사용)
  geometry_msgs::Twist cmd = *msg;
  cmd.linear.x  = clamp(cmd.linear.x,  -v_max_, v_max_);
  cmd.angular.z = clamp(cmd.angular.z, -w_max_, w_max_);

  pub_ugv_cmd_.publish(cmd);
  last_cmd_ = cmd;
  t_last_cmd_ = ros::Time::now();
}

void LimoBaseNode::spin()
{
  ros::Rate r(50);
  while (ros::ok())
  {
    // Deadman: 일정 시간 입력 없으면 정지 명령 1회 송신
    if ((ros::Time::now() - t_last_cmd_).toSec() > deadman_) {
      geometry_msgs::Twist stop;
      pub_ugv_cmd_.publish(stop);
      t_last_cmd_ = ros::Time::now(); // 반복 전송 방지
    }

    // Diagnostics (간단)
    DiagnosticArray arr; arr.header.stamp = ros::Time::now();
    DiagnosticStatus st;
    st.name = "limo_base_node";
    st.hardware_id = "limo";
    st.level = DiagnosticStatus::OK;
    st.message = "OK";
    diagnostic_msgs::KeyValue kv_cmd_sub;
    kv_cmd_sub.key = "cmd_sub";
    kv_cmd_sub.value = cmd_sub_topic_;
    st.values.push_back(kv_cmd_sub);
    
    diagnostic_msgs::KeyValue kv_ugv_cmd;
    kv_ugv_cmd.key = "ugv_cmd_topic";
    kv_ugv_cmd.value = ugv_cmd_topic_;
    st.values.push_back(kv_ugv_cmd);
    arr.status.push_back(st);
    pub_diag_.publish(arr);

    ros::spinOnce();
    r.sleep();
  }
}

} // ns

int main(int argc, char** argv)
{
  ros::init(argc, argv, "limo_base_node");
  ros::NodeHandle nh, pnh("~");
  limo_base::LimoBaseNode node(nh, pnh);
  node.spin();
  return 0;
}
