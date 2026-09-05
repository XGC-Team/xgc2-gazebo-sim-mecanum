#pragma once

#include <atomic>
#include <map>
#include <mutex>
#include <string>
#include <utility>

namespace xgc_chassis_hold {

// Lock order: registry -> gate -> caller's command state. Callbacks must not
// re-enter the gate or registry. Unregister before destroying the callback owner.
class Gate {
 public:
  typedef void (*ZeroFn)(void*);
  explicit Gate(std::string robot_id) : robot_id_(std::move(robot_id)) {}

  bool held() const { return held_.load(std::memory_order_acquire); }
  const std::string& robotId() const { return robot_id_; }

  void setZeroThunk(ZeroFn fn, void* self) {
    std::lock_guard<std::mutex> lock(mutex_);
    thunk_ = fn;
    thunk_self_ = self;
  }

  // The state transition and zero callback are one transaction. A successful
  // return means every previously admitted command transaction has completed.
  void setHeld(bool value) {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool was = held_.exchange(value, std::memory_order_acq_rel);
    if (value && !was) {
      fireZeroLocked();
    }
  }

  void fireZero() {
    std::lock_guard<std::mutex> lock(mutex_);
    fireZeroLocked();
  }

  // All command acceptance AND actuator writes must use this transaction.
  // The callback receives the current state; held() alone is not admission.
  template <typename Action>
  void withCommand(Action action) {
    std::lock_guard<std::mutex> lock(mutex_);
    action(held_.load(std::memory_order_relaxed));
  }

 private:
  void fireZeroLocked() {
    if (thunk_ != nullptr) {
      thunk_(thunk_self_);
    }
  }

  std::string robot_id_;
  std::atomic<bool> held_{false};
  std::mutex mutex_;
  ZeroFn thunk_ = nullptr;
  void* thunk_self_ = nullptr;
};

// No Gate pointer escapes a lookup. remove() drains an in-flight apply() before
// returning, protecting both the Gate and the object behind its zero thunk.
class GateRegistry {
 public:
  void add(Gate* gate) {
    if (gate == nullptr) return;
    std::lock_guard<std::mutex> lock(mutex_);
    gates_[gate->robotId()] = gate;
  }

  void remove(Gate* gate) {
    if (gate == nullptr) return;
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = gates_.find(gate->robotId());
    if (found != gates_.end() && found->second == gate) {
      gates_.erase(found);
    }
  }

  bool apply(const std::string& robot_id, bool held) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto found = gates_.find(robot_id);
    if (found == gates_.end()) {
      found = gates_.find(std::string());
    }
    if (found == gates_.end()) return false;
    found->second->setHeld(held);
    return true;
  }

 private:
  std::mutex mutex_;
  std::map<std::string, Gate*> gates_;
};

}  // namespace xgc_chassis_hold
