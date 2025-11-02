#include <ros/ros.h>
#include <thread>
#include <atomic>
#include <vector>
#include <mutex>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

#include "ugv_sdk/serial_port.h"
#include "ugv_sdk/limo_protocol.h"
#include "ugv_sdk/ugv_utils.h"

using diagnostic_msgs::DiagnosticArray;
using diagnostic_msgs::DiagnosticStatus;
using diagnostic_msgs::KeyValue;

class UgvSdkNode {
public:
  UgvSdkNode(ros::NodeHandle nh, ros::NodeHandle pnh)
  : nh_(nh), pnh_(pnh), running_(true)
  {
    // params
    port_     = getParam<std::string>("port", "/dev/ttyUSB0");
    baudrate_ = getParam<int>("baudrate", 115200);
    read_hz_  = getParam<double>("read_hz", 100.0);
    write_hz_ = getParam<double>("write_hz", 100.0);
    reconn_   = getParam<double>("reconnect_sec", 2.0);

    cmd_topic_  = getParam<std::string>("cmd_topic",  "/ugv_sdk/cmd_vel");
    odom_topic_ = getParam<std::string>("odom_topic", "/ugv_sdk/odom");
    imu_topic_  = getParam<std::string>("imu_topic",  "/ugv_sdk/imu/data");

    odom_frame_ = getParam<std::string>("odom_frame", "odom");
    base_frame_ = getParam<std::string>("base_frame", "base_link");

    pub_diag_ = getParam<bool>("publish_diag", true);
    diag_rate_= getParam<double>("diag_rate", 1.0);

    // ROS I/O
    sub_cmd_  = nh_.subscribe(cmd_topic_, 20, &UgvSdkNode::cmdCb, this);
    pub_odom_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 20);
    pub_imu_  = nh_.advertise<sensor_msgs::Imu>(imu_topic_, 50);
    pub_diag_arr_ = nh_.advertise<DiagnosticArray>("/diagnostics", 1);

    // threads
    th_read_  = std::thread(&UgvSdkNode::readLoop, this);
    th_write_ = std::thread(&UgvSdkNode::writeLoop, this);
    if (pub_diag_) th_diag_ = std::thread(&UgvSdkNode::diagLoop, this);
  }

  ~UgvSdkNode(){
    running_ = false;
    if (th_read_.joinable()) th_read_.join();
    if (th_write_.joinable()) th_write_.join();
    if (th_diag_.joinable()) th_diag_.join();
    port_serial_.close();
  }

private:
  template<typename T>
  T getParam(const std::string& key, const T& def){ T v; pnh_.param<T>(key, v, def); return v; }

  // ===== ROS Callbacks =====
  void cmdCb(const geometry_msgs::Twist::ConstPtr& msg){
    std::lock_guard<std::mutex> lk(mtx_cmd_);
    last_cmd_ = *msg;
    has_cmd_ = true;
  }

  // ===== Serial connection helpers =====
  bool ensureSerial(){
    if (port_serial_.isOpen()) return true;
    ROS_WARN_STREAM_THROTTLE(2.0, "[ugv_sdk] connecting " << port_ << " @" << baudrate_);
    if (!port_serial_.open(port_, baudrate_)){
      ROS_ERROR_STREAM_THROTTLE(2.0, "[ugv_sdk] open failed");
      return false;
    }
    buf_rx_.clear();
    return true;
  }

  // ===== Writer thread =====
  void writeLoop(){
    ros::Rate r(write_hz_ > 1 ? write_hz_ : 50);
    while (ros::ok() && running_){
      if (!ensureSerial()){ sleepSec(reconn_); r.sleep(); continue; }

      geometry_msgs::Twist cmd;
      {
        std::lock_guard<std::mutex> lk(mtx_cmd_);
        cmd = last_cmd_;
      }
      // pack twist
      ugv::TwistPayload tp{ float(cmd.linear.x), float(cmd.angular.z) };
      auto frame = ugv::packFrame(ugv::MSG_CMD_VEL, reinterpret_cast<uint8_t*>(&tp), sizeof(tp));
      int n = port_serial_.writeBytes(frame.data(), (int)frame.size());
      if (n < 0) { port_serial_.close(); }
      r.sleep();
    }
  }

  // ===== Reader thread =====
  void readLoop(){
    ros::Rate r(read_hz_ > 1 ? read_hz_ : 100);
    std::vector<uint8_t> tmp(512);
    while (ros::ok() && running_){
      if (!ensureSerial()){ sleepSec(reconn_); r.sleep(); continue; }

      int n = port_serial_.readSome(tmp.data(), (int)tmp.size());
      if (n < 0){ port_serial_.close(); sleepSec(reconn_); r.sleep(); continue; }
      if (n > 0){
        buf_rx_.insert(buf_rx_.end(), tmp.begin(), tmp.begin()+n);
        parseFrames();
      }
      r.sleep();
    }
  }

  void parseFrames(){
    size_t consumed = 0;
    while (buf_rx_.size() >= 6){
      uint8_t type = 0; std::vector<uint8_t> payload;
      if (!ugv::unpackFrame(buf_rx_.data(), buf_rx_.size(), type, payload, consumed)){
        // CRC fail or incomplete; drop consumed bytes and continue
        buf_rx_.erase(buf_rx_.begin(), buf_rx_.begin()+consumed);
        break;
      }
      // success → consume and dispatch
      std::vector<uint8_t> frame(buf_rx_.begin(), buf_rx_.begin()+consumed);
      buf_rx_.erase(buf_rx_.begin(), buf_rx_.begin()+consumed);

      if (type == ugv::MSG_ODOM && payload.size() == sizeof(ugv::OdomPayload)){
        ugv::OdomPayload od{};
        std::memcpy(&od, payload.data(), sizeof(od));
        nav_msgs::Odometry o;
        o.header.stamp = ros::Time::now();
        o.header.frame_id = odom_frame_;
        o.child_frame_id  = base_frame_;
        o.pose.pose.position.x = od.x;
        o.pose.pose.position.y = od.y;
        o.pose.pose.orientation.z = sin(od.yaw*0.5);
        o.pose.pose.orientation.w = cos(od.yaw*0.5);
        o.twist.twist.linear.x  = od.vx;
        o.twist.twist.angular.z = od.wz;
        pub_odom_.publish(o);
      }
      else if (type == ugv::MSG_IMU && payload.size() == sizeof(ugv::ImuPayload)){
        ugv::ImuPayload ip{};
        std::memcpy(&ip, payload.data(), sizeof(ip));
        sensor_msgs::Imu m;
        m.header.stamp = ros::Time::now();
        m.header.frame_id = base_frame_;
        m.linear_acceleration.x = ip.ax;
        m.linear_acceleration.y = ip.ay;
        m.linear_acceleration.z = ip.az;
        m.angular_velocity.x = ip.gx;
        m.angular_velocity.y = ip.gy;
        m.angular_velocity.z = ip.gz;
        m.orientation.x = ip.qx; m.orientation.y = ip.qy;
        m.orientation.z = ip.qz; m.orientation.w = ip.qw;
        pub_imu_.publish(m);
      }
      // heartbeat or others can be added here
    }
  }

  void diagLoop(){
    ros::Rate r(diag_rate_ > 0.1 ? diag_rate_ : 1.0);
    while (ros::ok() && running_){
      DiagnosticArray arr; arr.header.stamp = ros::Time::now();
      DiagnosticStatus st;
      st.name = "ugv_sdk_node"; st.hardware_id = "limo";
      st.level = port_serial_.isOpen() ? DiagnosticStatus::OK : DiagnosticStatus::WARN;
      st.message = port_serial_.isOpen() ? "serial ok" : "disconnected";
      st.values.push_back(KeyValue("port", port_));
      st.values.push_back(KeyValue("baudrate", std::to_string(baudrate_)));
      st.values.push_back(KeyValue("cmd_topic", cmd_topic_));
      st.values.push_back(KeyValue("odom_topic", odom_topic_));
      st.values.push_back(KeyValue("imu_topic", imu_topic_));
      arr.status.push_back(st);
      pub_diag_arr_.publish(arr);
      r.sleep();
    }
  }

  static void sleepSec(double s){
    if (s <= 0) return;
    timespec ts; ts.tv_sec = (time_t)s; ts.tv_nsec = (long)((s - ts.tv_sec)*1e9);
    nanosleep(&ts, nullptr);
  }

private:
  ros::NodeHandle nh_, pnh_;
  std::string port_; int baudrate_;
  double read_hz_, write_hz_, reconn_;
  std::string cmd_topic_, odom_topic_, imu_topic_;
  std::string odom_frame_, base_frame_;
  bool pub_diag_; double diag_rate_;

  ros::Subscriber sub_cmd_;
  ros::Publisher  pub_odom_, pub_imu_, pub_diag_arr_;

  ugv::SerialPort port_serial_;
  std::thread th_read_, th_write_, th_diag_;
  std::atomic<bool> running_;
  std::vector<uint8_t> buf_rx_;

  std::mutex mtx_cmd_;
  geometry_msgs::Twist last_cmd_; bool has_cmd_{false};
};

int main(int argc, char** argv){
  ros::init(argc, argv, "ugv_sdk_node");
  ros::NodeHandle nh, pnh("~");
  UgvSdkNode node(nh, pnh);
  ros::spin();
  return 0;
}
