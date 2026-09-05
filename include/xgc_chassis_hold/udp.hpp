#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <cerrno>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include "xgc_chassis_hold/gate.hpp"

namespace xgc_chassis_hold {

enum { kPort = 19520, kMagic = 0x58474348u, kVersion = 1u, kRobotIdBytes = 32u };
enum { kRequestBytes = 44u, kAckBytes = 12u };

inline void writeU32LE(unsigned char *out, unsigned value) {
  out[0] = static_cast<unsigned char>(value);
  out[1] = static_cast<unsigned char>(value >> 8);
  out[2] = static_cast<unsigned char>(value >> 16);
  out[3] = static_cast<unsigned char>(value >> 24);
}

inline unsigned readU32LE(const unsigned char *in) {
  return static_cast<unsigned>(in[0]) | (static_cast<unsigned>(in[1]) << 8) |
         (static_cast<unsigned>(in[2]) << 16) |
         (static_cast<unsigned>(in[3]) << 24);
}

inline std::string lastPath(const std::string &value) {
  if (value.empty()) {
    return value;
  }
  std::string trimmed = value;
  while (!trimmed.empty() && trimmed[trimmed.size() - 1] == '/') {
    trimmed.erase(trimmed.size() - 1);
  }
  const std::string::size_type slash = trimmed.rfind('/');
  if (slash == std::string::npos) {
    return trimmed;
  }
  return trimmed.substr(slash + 1);
}

class Hub {
 public:
  static Hub &instance() {
    static Hub hub;
    return hub;
  }

  void add(Gate *gate) {
    if (gate == nullptr) return;
    start();
    registry_.add(gate);
  }

  void remove(Gate *gate) { registry_.remove(gate); }

 private:
  Hub() {}
  ~Hub() {
    stop_.store(true, std::memory_order_release);
    if (fd_ >= 0) {
      shutdown(fd_, SHUT_RDWR);
    }
    if (thread_.joinable()) {
      thread_.join();
    }
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

  void start() {
    bool expected = false;
    if (!started_.compare_exchange_strong(expected, true)) {
      return;
    }
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
      started_.store(false);
      return;
    }
    int reuse = 1;
    setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(kPort));
    if (bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
      close(fd_);
      fd_ = -1;
      started_.store(false);
      return;
    }
    thread_ = std::thread(&Hub::loop, this);
  }

  void loop() {
    unsigned char buf[kRequestBytes];
    while (!stop_.load(std::memory_order_acquire)) {
      sockaddr_in from;
      socklen_t from_len = sizeof(from);
      const ssize_t n = recvfrom(fd_, buf, sizeof(buf), 0,
                                 reinterpret_cast<sockaddr *>(&from), &from_len);
      if (n < 0) {
        if (errno == EINTR) {
          continue;
        }
        break;
      }
      if (n != static_cast<ssize_t>(kRequestBytes)) {
        continue;
      }
      if (readU32LE(buf) != kMagic || buf[4] != kVersion) {
        continue;
      }
      const bool held = buf[5] != 0;
      const unsigned request_id = readU32LE(buf + 8);
      char robot[kRobotIdBytes + 1];
      std::memcpy(robot, buf + 12, kRobotIdBytes);
      robot[kRobotIdBytes] = '\0';
      const bool matched = registry_.apply(robot, held);
      unsigned char ack[kAckBytes];
      std::memset(ack, 0, sizeof(ack));
      writeU32LE(ack, kMagic);
      ack[4] = kVersion;
      ack[5] = held ? 1 : 0;
      ack[6] = matched ? 0 : 1;
      writeU32LE(ack + 8, request_id);
      sendto(fd_, ack, sizeof(ack), 0, reinterpret_cast<sockaddr *>(&from),
             from_len);
    }
  }

  GateRegistry registry_;
  std::atomic<bool> started_{false};
  std::atomic<bool> stop_{false};
  int fd_ = -1;
  std::thread thread_;
};

}  // namespace xgc_chassis_hold
