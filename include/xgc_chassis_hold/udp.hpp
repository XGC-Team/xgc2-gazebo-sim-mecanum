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
#include <memory>
#include <stdexcept>
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

// Simulator endpoints are robot-specific. This mapping is shared with Core's
// simulation sender; physical chassis retain their separate port 19520 contract.
// Hash collisions are detected by exclusive bind, never shared or misrouted.
inline uint16_t simulationPort(const std::string& robot_id) {
  uint32_t hash = 2166136261u;
  for (unsigned char ch : robot_id) { hash ^= ch; hash *= 16777619u; }
  return static_cast<uint16_t>(20000u + hash % 20000u);
}

class Hub {
 public:
  static Hub &instance() { static Hub hub; return hub; }

  void add(Gate *gate) {
    if (!gate || gate->robotId().empty() || gate->robotId().size() >= kRobotIdBytes)
      throw std::runtime_error("HOLD requires a nonempty robot ID shorter than 32 bytes");
    std::lock_guard<std::mutex> lock(mutex_);
    if (endpoints_.count(gate)) return;
    std::unique_ptr<Endpoint> endpoint(new Endpoint);
    endpoint->fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (endpoint->fd < 0) throw std::runtime_error("HOLD socket creation failed");
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(simulationPort(gate->robotId()));
    if (bind(endpoint->fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
      const std::string message = "HOLD endpoint for " + gate->robotId() + " at port " +
          std::to_string(simulationPort(gate->robotId())) + " is unavailable: " + std::strerror(errno);
      close(endpoint->fd);
      throw std::runtime_error(message);
    }
    endpoint->robot = gate->robotId();
    registry_.add(gate);
    Endpoint* raw = endpoint.get();
    try { endpoint->thread = std::thread([this, raw] { loop(*raw); }); }
    catch (...) { registry_.remove(gate); close(endpoint->fd); throw; }
    endpoints_.emplace(gate, std::move(endpoint));
  }

  void remove(Gate *gate) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto found = endpoints_.find(gate);
    if (found == endpoints_.end()) return;
    registry_.remove(gate); // drains the owner callback before destruction
    stop(*found->second);
    endpoints_.erase(found);
  }

 private:
  struct Endpoint {
    int fd = -1;
    std::string robot;
    std::atomic<bool> stopping{false};
    std::thread thread;
  };
  Hub() {}
  ~Hub() { for (auto& entry : endpoints_) stop(*entry.second); }
  static void stop(Endpoint& endpoint) {
    endpoint.stopping.store(true);
    shutdown(endpoint.fd, SHUT_RDWR);
    if (endpoint.thread.joinable()) endpoint.thread.join();
    close(endpoint.fd);
  }

  void loop(Endpoint& endpoint) {
    // Read one extra byte: recvfrom truncation must not make an oversized
    // datagram look like an exact-length request.
    unsigned char buf[kRequestBytes + 1];
    while (!endpoint.stopping.load(std::memory_order_acquire)) {
      sockaddr_in from;
      socklen_t from_len = sizeof(from);
      const ssize_t n = recvfrom(endpoint.fd, buf, sizeof(buf), 0,
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
      const bool matched = endpoint.robot == robot && registry_.apply(robot, held);
      unsigned char ack[kAckBytes];
      std::memset(ack, 0, sizeof(ack));
      writeU32LE(ack, kMagic);
      ack[4] = kVersion;
      ack[5] = held ? 1 : 0;
      ack[6] = matched ? 0 : 1;
      writeU32LE(ack + 8, request_id);
      sendto(endpoint.fd, ack, sizeof(ack), 0, reinterpret_cast<sockaddr *>(&from),
             from_len);
    }
  }

  GateRegistry registry_;
  std::mutex mutex_;
  std::map<Gate*, std::unique_ptr<Endpoint>> endpoints_;
};

}  // namespace xgc_chassis_hold
