#pragma once
#include <algorithm>
#include <string>

namespace limo_base {
inline double clamp(double v, double lo, double hi){ return std::max(lo, std::min(hi, v)); }

inline std::string paramOr(const ros::NodeHandle& nh, const std::string& name, const std::string& def){
  std::string v; nh.param<std::string>(name, v, def); return v;
}
inline double paramOr(const ros::NodeHandle& nh, const std::string& name, double def){
  double v; nh.param<double>(name, v, def); return v;
}
inline bool paramOr(const ros::NodeHandle& nh, const std::string& name, bool def){
  bool v; nh.param<bool>(name, v, def); return v;
}
} // namespace
