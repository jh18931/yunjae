#pragma once
#include <cstdint>
#include <vector>
#include <cstring>
#include "ugv_sdk/ugv_utils.h"

namespace ugv {

// === 실제 프로토콜에 맞게 교체하세요 ===
// 여기서는 예시로 [HEAD=0xAA 0x55][LEN][TYPE][PAYLOAD...][CRC16] 구조를 가정.

static constexpr uint8_t HEAD0 = 0xAA;
static constexpr uint8_t HEAD1 = 0x55;

enum MsgType : uint8_t {
  MSG_CMD_VEL = 0x01,    // PC → UGV: Twist 명령
  MSG_ODOM    = 0x10,    // UGV → PC: 오돔
  MSG_IMU     = 0x11,    // UGV → PC: IMU
  MSG_HEARTBEAT = 0xF0,  // 양방향 keepalive
};

struct TwistPayload {
  float vx;   // m/s
  float wz;   // rad/s
} __attribute__((packed));

struct OdomPayload {
  float x, y, yaw;     // 2D pose
  float vx, wz;        // velocities
} __attribute__((packed));

struct ImuPayload {
  float ax, ay, az;    // m/s^2
  float gx, gy, gz;    // rad/s
  float qx, qy, qz, qw;// orientation
} __attribute__((packed));

inline std::vector<uint8_t> packFrame(uint8_t type, const uint8_t* payload, uint8_t len) {
  std::vector<uint8_t> buf;
  buf.reserve(4 + len + 2);
  buf.push_back(HEAD0);
  buf.push_back(HEAD1);
  buf.push_back(len);
  buf.push_back(type);
  for (int i=0;i<len;i++) buf.push_back(payload[i]);
  uint16_t crc = crc16_x25(buf.data(), buf.size());
  buf.push_back(uint8_t(crc & 0xFF));
  buf.push_back(uint8_t((crc >> 8) & 0xFF));
  return buf;
}

inline bool unpackFrame(const uint8_t* data, size_t n, uint8_t& type, std::vector<uint8_t>& payload, size_t& consumed) {
  consumed = 0;
  if (n < 6) return false;
  size_t i = 0;
  while (i+1 < n && !(data[i]==HEAD0 && data[i+1]==HEAD1)) i++;
  if (i+4 >= n) return false;
  uint8_t len = data[i+2];
  type = data[i+3];
  size_t total = 4 + len + 2;
  if (i + total > n) return false;

  uint16_t crc_calc = crc16_x25(&data[i], 4 + len);
  uint16_t crc_in = uint16_t(data[i+4+len]) | (uint16_t(data[i+5+len])<<8);
  if (crc_calc != crc_in) { consumed = i+total; return false; }

  payload.assign(&data[i+4], &data[i+4+len]);
  consumed = i + total;
  return true;
}

} // ns
