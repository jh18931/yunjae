#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <mutex>

namespace ugv {

class SerialPort {
public:
 SerialPort():fd_(-1){}
 ~SerialPort(){ close(); }

 bool open(const std::string& port, int baud);
 void close();
 bool isOpen() const { return fd_>=0; }


 int writeBytes(const uint8_t* data, int len);
 int readSome(uint8_t* buf, int maxlen);

private:
 int fd_;
 std::mutex mtx_;
};

} // ns


// serial_port.h 하단에 추가 구현 (간단 버전)
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <string.h>

namespace ugv {

inline bool SerialPort::open(const std::string& port, int baud){
 std::lock_guard<std::mutex> lk(mtx_);
 close();
 fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
 if (fd_ < 0) return false;

 termios tty{};
 if (tcgetattr(fd_, &tty) != 0) { close(); return false; }

 speed_t spd = B115200;
 if (baud == 921600) spd = B921600;
 else if (baud == 460800) spd = B460800;
 else if (baud == 230400) spd = B230400;
 else if (baud == 115200) spd = B115200;
 else if (baud == 57600) spd = B57600;
 cfsetispeed(&tty, spd); cfsetospeed(&tty, spd);

 tty.c_cflag |= (CLOCAL | CREAD);
 tty.c_cflag &= ~CSIZE; tty.c_cflag |= CS8;
 tty.c_cflag &= ~PARENB;
 tty.c_cflag &= ~CSTOPB;
 tty.c_cflag &= ~CRTSCTS;

 tty.c_lflag = 0; // raw
 tty.c_iflag &= ~(IXON | IXOFF | IXANY);
 tty.c_oflag = 0;
 tty.c_cc[VTIME] = 1; // 0.1s
 tty.c_cc[VMIN] = 0;

 tcflush(fd_, TCIFLUSH);
 if (tcsetattr(fd_, TCSANOW, &tty) != 0) { close(); return false; }


 int flags = fcntl(fd_, F_GETFL, 0);
 fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

 return true;
}

inline void SerialPort::close(){
 std::lock_guard<std::mutex> lk(mtx_);
 if (fd_>=0) { ::close(fd_); fd_=-1; }
}

inline int SerialPort::writeBytes(const uint8_t* data, int len){
 std::lock_guard<std::mutex> lk(mtx_);
 if (fd_<0) return -1;
 return ::write(fd_, data, len);
}

inline int SerialPort::readSome(uint8_t* buf, int maxlen){
 std::lock_guard<std::mutex> lk(mtx_);
 if (fd_<0) return -1;
 return ::read(fd_, buf, maxlen);
}

} // ns
