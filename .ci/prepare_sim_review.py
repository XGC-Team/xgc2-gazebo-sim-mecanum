from pathlib import Path
import subprocess
import urllib.request

# Copy the exact same reviewed implementation and executable tests, not a
# floating dependency. Both source repositories are public; no token is sent.
shared_commit = 'acfb1a784ff61ec2787e637256148d9a65b4ee38'
for source, target in (
    ('scout/include/xgc_chassis_hold/gate.hpp', 'include/xgc_chassis_hold/gate.hpp'),
    ('scout/test/hold_gate_test.cpp', 'test/hold_gate_test.cpp'),
):
    path = Path(target)
    if path.exists():
        raise RuntimeError(f'{target} already exists')
    url = 'https://raw.githubusercontent.com/XGC-Team/xgc2-gazebo-sim-agilex/' + shared_commit + '/' + source
    with urllib.request.urlopen(url, timeout=60) as response:
        data = response.read()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)


def replace(path, old, new, count=1):
    file = Path(path)
    text = file.read_text()
    if text.count(old) != count:
        raise RuntimeError(f'{path}: expected {count} occurrences, found {text.count(old)}')
    file.write_text(text.replace(old, new))


def replace_region(path, start, end, new):
    file = Path(path)
    text = file.read_text()
    if text.count(start) != 1 or text.count(end) != 1:
        raise RuntimeError(f'{path}: ambiguous region')
    a, b = text.index(start), text.index(end)
    if b <= a:
        raise RuntimeError(f'{path}: reversed region')
    file.write_text(text[:a] + new + text[b:])


udp = 'include/xgc_chassis_hold/udp.hpp'
replace(udp, '#include <thread>\n', '#include <thread>\n\n#include "xgc_chassis_hold/gate.hpp"\n')
replace_region(udp, 'class Gate {', 'inline void writeU32LE', '')
replace_region(udp, '  void add(Gate *gate) {', '\n private:\n  Hub()', '''  void add(Gate *gate) {
    if (gate == nullptr) return;
    start();
    registry_.add(gate);
  }

  void remove(Gate *gate) { registry_.remove(gate); }
''')
replace(udp, '''      close(fd_);
      fd_ = -1;
    }
    if (thread_.joinable()) {
      thread_.join();
    }
  }''', '''    }
    if (thread_.joinable()) {
      thread_.join();
    }
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }''')
replace(udp, '''      Gate *gate = match(robot);
      if (gate != nullptr) {
        gate->setHeld(held);
      }''', '''      const bool matched = registry_.apply(robot, held);''')
replace(udp, 'ack[6] = gate != nullptr ? 0 : 1;', 'ack[6] = matched ? 0 : 1;')
replace_region(udp, '  Gate *match(const char *robot_id) {', '  std::atomic<bool> started_', '  GateRegistry registry_;\n')

source = 'src/mecanum_contract_plugin.cpp'
init = '''    hold_gate_.reset(new xgc_chassis_hold::Gate(xgc_chassis_hold::lastPath(robot_namespace)));
    hold_gate_->setZeroThunk(&MecanumContractPlugin::HoldZeroThunk, this);
    xgc_chassis_hold::Hub::instance().add(hold_gate_.get());

'''
replace(source, init, '')
replace(source, '    ros::SubscribeOptions command_options', init + '    ros::SubscribeOptions command_options')
cleanup = '''    if (hold_gate_) {
      xgc_chassis_hold::Hub::instance().remove(hold_gate_.get());
      hold_gate_.reset();
    }
'''
replace(source, cleanup, '')
replace(source, '    command_queue_.clear();\n', '''    command_queue_.clear();

    // All ROS/Gazebo command producers are drained before releasing the Gate.
    // Unregister also waits for any in-flight UDP zero callback.
''' + cleanup)
replace(source, '''    if (hold_gate_ && hold_gate_->held()) {
      HoldZero();
      return;
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    front_velocity_command_ = linear_scale_ * ClampFinite(command->linear.x, max_front_velocity_);
    left_velocity_command_ = linear_scale_ * ClampFinite(command->linear.y, max_left_velocity_);
    yaw_velocity_command_ = angular_scale_ * ClampFinite(command->angular.z, max_yaw_velocity_);''', '''    hold_gate_->withCommand([&](bool held) {
      if (held) {
        HoldZero();
        return;
      }
      std::lock_guard<std::mutex> lock(command_mutex_);
      front_velocity_command_ = linear_scale_ * ClampFinite(command->linear.x, max_front_velocity_);
      left_velocity_command_ = linear_scale_ * ClampFinite(command->linear.y, max_left_velocity_);
      yaw_velocity_command_ = angular_scale_ * ClampFinite(command->angular.z, max_yaw_velocity_);
    });''')
replace(source, '''    if (high_fidelity_) {
      ApplyWheelDynamics(dt);
    } else {
      ApplyPlanarVelocity();
    }''', '''    // Serialize the actuator write, not only the earlier command receipt,
    // against HOLD. Zero wheel targets can still generate braking forces.
    hold_gate_->withCommand([&](bool held) {
      if (held) {
        HoldZero();
      }
      if (high_fidelity_) {
        ApplyWheelDynamics(dt);
      } else {
        ApplyPlanarVelocity();
      }
    });''')
subprocess.run(['python3', '-m', 'unittest', 'discover', '-s', 'test', '-p', 'test_hold_wiring.py', '-v'], check=True)
for suffix, flags in (('plain', ['-Wall', '-Wextra', '-Werror']),
                      ('san', ['-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer'])):
    binary = '/tmp/hold_gate_' + suffix
    subprocess.run(['g++', '-std=c++11', '-pthread', '-Iinclude', *flags,
                    'test/hold_gate_test.cpp', '-o', binary], check=True)
    subprocess.run([binary], check=True, timeout=30)
Path('/tmp/udp_header_smoke.cpp').write_text('#include "xgc_chassis_hold/udp.hpp"\nint main() {}\n')
subprocess.run(['g++', '-std=c++11', '-pthread', '-Wall', '-Wextra', '-Werror', '-Iinclude',
                '-fsyntax-only', '/tmp/udp_header_smoke.cpp'], check=True)
