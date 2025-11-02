#pragma once
#include <algorithm>
#include <string>
#include <vector>
#include <cstdint>

namespace ugv {

inline double clamp(double v, double lo, double hi){ return std::max(lo, std::min(hi, v)); }

inline uint16_t crc16_x25(const uint8_t* data, size_t len) {
  // 데모용 CRC(필요시 실제 프로토콜 CRC로 교체)
  uint16_t crc = 0xFFFF;
  for (size_t i=0;i<len;i++){
    crc ^= data[i];
    for (int j=0;j<8;j++) crc = (crc & 1) ? (crc>>1) ^ 0x8408 : (crc>>1);
  }
  return ~crc;
}

} // ns
