#include "xgc_chassis_hold/gate.hpp"

#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>

namespace {
void Check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
void Zero(void* value) { *static_cast<int*>(value) = 0; }

void AdmissionIsSerialized() {
  xgc_chassis_hold::Gate gate("ugv1");
  int output = 0;
  gate.setZeroThunk(&Zero, &output);
  std::promise<void> entered, release, hold_started;
  auto resume = release.get_future().share();
  auto command = std::async(std::launch::async, [&] {
    gate.withCommand([&](bool held) {
      Check(!held, "initial command must be admitted");
      entered.set_value();
      resume.wait();
      output = 42;
    });
  });
  entered.get_future().wait();
  auto hold = std::async(std::launch::async, [&] {
    hold_started.set_value();
    gate.setHeld(true);
  });
  hold_started.get_future().wait();
  const bool blocked = hold.wait_for(std::chrono::milliseconds(20)) == std::future_status::timeout;
  release.set_value();
  command.get();
  hold.get();
  Check(blocked, "HOLD must drain a previously admitted command");
  Check(output == 0, "HOLD must win after the old command exits");
  gate.withCommand([&](bool held) { if (!held) output = 99; });
  Check(output == 0, "a callback arriving after HOLD cannot submit a command");
  gate.setHeld(true);
  Check(output == 0, "repeated HOLD must preserve zero");
  gate.setHeld(false);
  Check(output == 0, "release must not replay a cached command");
}

struct BlockingZero {
  std::promise<void> entered;
  std::shared_future<void> resume;
  static void Run(void* self) {
    auto* state = static_cast<BlockingZero*>(self);
    state->entered.set_value();
    state->resume.wait();
  }
};

void UnregisterDrainsCallbackOwner() {
  xgc_chassis_hold::GateRegistry registry;
  std::unique_ptr<xgc_chassis_hold::Gate> gate(new xgc_chassis_hold::Gate("ugv1"));
  std::promise<void> release, removal_started;
  BlockingZero callback;
  callback.resume = release.get_future().share();
  gate->setZeroThunk(&BlockingZero::Run, &callback);
  registry.add(gate.get());
  auto request = std::async(std::launch::async, [&] { return registry.apply("ugv1", true); });
  callback.entered.get_future().wait();
  auto removal = std::async(std::launch::async, [&] {
    removal_started.set_value();
    registry.remove(gate.get());
    gate.reset();
  });
  removal_started.get_future().wait();
  const bool blocked = removal.wait_for(std::chrono::milliseconds(20)) == std::future_status::timeout;
  release.set_value();
  Check(request.get(), "the registered request must match");
  removal.get();
  Check(blocked, "remove must wait until the zero thunk exits");
  Check(!registry.apply("ugv1", false), "unregistered object must not be called");
}

void ReplacementAndFallback() {
  xgc_chassis_hold::GateRegistry registry;
  xgc_chassis_hold::Gate old_gate("ugv1"), replacement("ugv1"), fallback("");
  registry.add(&old_gate);
  registry.add(&replacement);
  registry.remove(&old_gate);
  Check(registry.apply("ugv1", true), "old registration cannot remove replacement");
  Check(replacement.held() && !old_gate.held(), "only the replacement is updated");
  registry.remove(&replacement);
  Check(!registry.apply("unknown", true), "unknown IDs must not match without fallback");
  registry.add(&fallback);
  Check(registry.apply("unknown", true) && fallback.held(), "preserve explicit wildcard fallback");
  registry.remove(&fallback);
}
}  // namespace

int main() {
  for (int i = 0; i < 20; ++i) {
    AdmissionIsSerialized();
    UnregisterDrainsCallbackOwner();
    ReplacementAndFallback();
  }
  std::cout << "HOLD gate: 60 deterministic checks passed\n";
}
