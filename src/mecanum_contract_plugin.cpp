#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <boost/bind/bind.hpp>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

namespace gazebo_sim_mecanum {
namespace {

constexpr double kPi = 3.14159265358979323846;
// Preserve the deployed swarm-sync-sim ugv_sim_single.launch contract. These
// are not AgileX/Scout or legacy force-plugin limits.
constexpr double kSssMaxFrontVelocity = 1.5;
constexpr double kSssMaxLeftVelocity = 1.5;
constexpr double kSssMaxYawVelocity = kPi / 2.0;
constexpr std::size_t kWheelCount = 4;
// Wheeltec MCU publishes /imu at 20 Hz. Onboard /PowerVoltage is ~1.67 Hz
// (every 11 chassis frames); swarm-ros-bridge send max_freq=1 on :3002.
// Simulation has no bridge. Gazebo publishes the post-bridge contract:
// std_msgs/Float32 volts at 1 Hz. Core mecanum_ugv.3s_lipo is linear
// 10.5–12.6 V.
constexpr double kDefaultImuRate = 20.0;
constexpr double kDefaultBatteryVoltage = 12.348;  // 88% SOC
constexpr double kDefaultPowerVoltageRate = 1.0;
// These exact relative names stay below the slot NodeHandle. Canonical
// /<namespace>/pose and /twist belong to experiment-localization-projection.
constexpr const char* kSimulationGroundTruthPoseTopic = "simulation/ground_truth/pose";
constexpr const char* kSimulationGroundTruthTwistTopic = "simulation/ground_truth/twist";

template <typename T>
T SdfValue(const sdf::ElementPtr& sdf, const std::string& name, const T& fallback) {
  return sdf && sdf->HasElement(name) ? sdf->Get<T>(name) : fallback;
}

double ClampFinite(double value, double limit) {
  if (!std::isfinite(value)) {
    return 0.0;
  }
  return std::max(-limit, std::min(limit, value));
}

std::string TrimSlashes(std::string value) {
  while (!value.empty() && value.front() == '/') {
    value.erase(value.begin());
  }
  while (!value.empty() && value.back() == '/') {
    value.pop_back();
  }
  return value;
}

bool SdfDeclaresExactRelativeTopic(const sdf::ElementPtr& sdf, const char* element,
                                   const char* expected) {
  return !sdf || !sdf->HasElement(element) || sdf->Get<std::string>(element) == expected;
}

}  // namespace

// Gazebo always owns pose integration. The selectable ideal mode writes the
// requested body velocity, while the default high-fidelity mode closes four
// wheel-speed loops and converts wheel/ground slip into bounded Mecanum forces.
class MecanumContractPlugin final : public gazebo::ModelPlugin {
 private:
  // Gazebo disconnects future event delivery without joining a callback that
  // was already selected. The callback owns this gate and never dereferences
  // the plugin outside its lock, so Shutdown can drain that final callback.
  struct UpdateGate {
    explicit UpdateGate(MecanumContractPlugin* initial_owner) : owner(initial_owner) {}

    std::mutex mutex;
    MecanumContractPlugin* owner;
  };

 public:
  MecanumContractPlugin() : update_gate_(std::make_shared<UpdateGate>(this)) {}

  ~MecanumContractPlugin() override { Shutdown(); }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = std::move(model);
    if (!model_) {
      gzerr << "[gazebo_sim_mecanum] Missing Gazebo model\n";
      return;
    }
    if (!ros::isInitialized()) {
      gzerr << "[gazebo_sim_mecanum] gazebo_ros_api_plugin is required\n";
      return;
    }

    std::string robot_namespace = SdfValue<std::string>(sdf, "robotNamespace", "");
    if (robot_namespace.empty()) {
      robot_namespace = model_->GetName();
    }
    ros_node_.reset(new ros::NodeHandle(robot_namespace));

    const std::string command_topic = SdfValue<std::string>(sdf, "commandTopic", "cmd_vel");
    if (!SdfDeclaresExactRelativeTopic(sdf, "poseTopic", kSimulationGroundTruthPoseTopic) ||
        !SdfDeclaresExactRelativeTopic(sdf, "twistTopic", kSimulationGroundTruthTwistTopic)) {
      gzerr << "[gazebo_sim_mecanum] poseTopic/twistTopic are not configurable; they must equal "
               "the exact relative strings simulation/ground_truth/pose and "
               "simulation/ground_truth/twist. Canonical "
               "/<namespace>/pose and /twist are owned by experiment-localization-projection\n";
      return;
    }
    const std::string imu_topic = SdfValue<std::string>(sdf, "imuTopic", "imu");
    const std::string joint_states_topic =
        SdfValue<std::string>(sdf, "jointStatesTopic", "joint_states");
    const std::string power_voltage_topic =
        SdfValue<std::string>(sdf, "powerVoltageTopic", "PowerVoltage");

    state_publish_rate_ = SdfValue<double>(sdf, "statePublishRate", 100.0);
    visual_publish_rate_ = SdfValue<double>(sdf, "visualPublishRate", 20.0);
    imu_publish_rate_ = SdfValue<double>(sdf, "imuPublishRate", kDefaultImuRate);
    power_voltage_publish_rate_ =
        SdfValue<double>(sdf, "powerVoltagePublishRate", kDefaultPowerVoltageRate);
    battery_voltage_ = SdfValue<double>(sdf, "batteryVoltage", kDefaultBatteryVoltage);
    max_front_velocity_ = SdfValue<double>(sdf, "maxFrontVelocity", kSssMaxFrontVelocity);
    max_left_velocity_ = SdfValue<double>(sdf, "maxLeftVelocity", kSssMaxLeftVelocity);
    max_yaw_velocity_ = SdfValue<double>(sdf, "maxYawVelocity", kSssMaxYawVelocity);
    linear_scale_ = SdfValue<double>(sdf, "linearScale", 1.0);
    angular_scale_ = SdfValue<double>(sdf, "angularScale", 1.0);
    imu_yaw_base_ = SdfValue<double>(sdf, "imuYawBase", kPi / 4.0);
    use_imu_orientation_ = SdfValue<bool>(sdf, "useImuOrientation", false);
    map_frame_ = SdfValue<std::string>(sdf, "mapFrame", "map");
    imu_frame_ = SdfValue<std::string>(sdf, "imuFrame", "world");
    base_frame_ = SdfValue<std::string>(sdf, "baseFrame", "base_footprint");
    drive_model_ = SdfValue<std::string>(sdf, "driveModel", "high_fidelity");
    wheel_radius_ = std::abs(SdfValue<double>(sdf, "wheelRadius", 0.05));
    wheelbase_sum_ = std::abs(SdfValue<double>(sdf, "wheelbaseSum", 0.30));
    wheel_pid_p_ = std::abs(SdfValue<double>(sdf, "wheelPidP", 0.35));
    wheel_torque_limit_ = std::abs(SdfValue<double>(sdf, "wheelTorqueLimit", 1.2));
    traction_gain_ = std::abs(SdfValue<double>(sdf, "tractionGain", 18.0));
    friction_coefficient_ = std::abs(SdfValue<double>(sdf, "frictionCoefficient", 0.85));
    linear_drag_ = std::abs(SdfValue<double>(sdf, "linearDrag", 0.10));
    angular_drag_ = std::abs(SdfValue<double>(sdf, "angularDrag", 0.02));
    yaw_traction_scale_ = std::abs(SdfValue<double>(sdf, "yawTractionScale", 0.45));

    if (drive_model_ != "high_fidelity" && drive_model_ != "ideal") {
      gzerr << "[gazebo_sim_mecanum] driveModel must be 'high_fidelity' or 'ideal', got '"
            << drive_model_ << "'\n";
      return;
    }
    high_fidelity_ = drive_model_ == "high_fidelity";
    if (high_fidelity_) {
      body_link_ = model_->GetLink("base_footprint");
      const std::array<std::string, kWheelCount> joint_names = {
          "upper_left_wheel_joint", "upper_right_wheel_joint", "lower_left_wheel_joint",
          "lower_right_wheel_joint"};
      for (std::size_t index = 0; index < joint_names.size(); ++index) {
        wheel_joints_[index] = model_->GetJoint(joint_names[index]);
        if (!wheel_joints_[index]) {
          gzerr << "[gazebo_sim_mecanum] Missing high-fidelity joint '" << joint_names[index] << "'\n";
          return;
        }
      }
      if (!body_link_ || wheel_radius_ < 1.0e-6) {
        gzerr << "[gazebo_sim_mecanum] Invalid high-fidelity body or wheel radius\n";
        return;
      }
      double total_mass = 0.0;
      for (const auto& link : model_->GetLinks()) {
        total_mass += link->GetInertial()->Mass();
      }
      normal_force_per_wheel_ = total_mass * 9.80665 / static_cast<double>(kWheelCount);
    }

    ros_node_->param("state_publish_rate", state_publish_rate_, state_publish_rate_);
    ros_node_->param("visualize_max_freq", visual_publish_rate_, visual_publish_rate_);
    ros_node_->param("v_front_max_m_s", max_front_velocity_, max_front_velocity_);
    ros_node_->param("v_left_max_m_s", max_left_velocity_, max_left_velocity_);
    double max_yaw_deg_s = max_yaw_velocity_ * 180.0 / kPi;
    ros_node_->param("omega_max_deg_s", max_yaw_deg_s, max_yaw_deg_s);
    max_yaw_velocity_ = max_yaw_deg_s * kPi / 180.0;
    ros_node_->param("linear_vel_scale", linear_scale_, linear_scale_);
    ros_node_->param("angular_vel_scale", angular_scale_, angular_scale_);
    ros_node_->param("use_imu_orientation", use_imu_orientation_, use_imu_orientation_);
    ros_node_->param("imu_publish_rate", imu_publish_rate_, imu_publish_rate_);
    ros_node_->param("battery_voltage", battery_voltage_, battery_voltage_);
    ros_node_->param("power_voltage_publish_rate", power_voltage_publish_rate_,
                     power_voltage_publish_rate_);

    state_publish_rate_ = std::max(1.0, state_publish_rate_);
    visual_publish_rate_ = std::max(1.0, visual_publish_rate_);
    imu_publish_rate_ = std::max(1.0, imu_publish_rate_);
    power_voltage_publish_rate_ = std::max(0.1, power_voltage_publish_rate_);
    if (!std::isfinite(battery_voltage_)) {
      battery_voltage_ = kDefaultBatteryVoltage;
    }
    max_front_velocity_ = std::abs(max_front_velocity_);
    max_left_velocity_ = std::abs(max_left_velocity_);
    max_yaw_velocity_ = std::abs(max_yaw_velocity_);

    pose_publisher_ = ros_node_->advertise<geometry_msgs::PoseStamped>(kSimulationGroundTruthPoseTopic, 10, false);
    twist_publisher_ = ros_node_->advertise<geometry_msgs::TwistStamped>(kSimulationGroundTruthTwistTopic, 10, false);
    imu_publisher_ = ros_node_->advertise<sensor_msgs::Imu>(imu_topic, 10, false);
    joint_states_publisher_ = ros_node_->advertise<sensor_msgs::JointState>(joint_states_topic, 1, false);
    power_voltage_publisher_ =
        ros_node_->advertise<std_msgs::Float32>(power_voltage_topic, 10, false);
    ros::SubscribeOptions command_options = ros::SubscribeOptions::create<geometry_msgs::Twist>(
        command_topic, 1000,
        boost::bind(&MecanumContractPlugin::CommandCallback, this, boost::placeholders::_1), ros::VoidPtr(),
        &command_queue_);
    command_subscriber_ = ros_node_->subscribe(command_options);
    command_spinner_.reset(new ros::AsyncSpinner(1, &command_queue_));
    command_spinner_->start();

    tf_child_frame_ = TrimSlashes(robot_namespace);
    if (tf_child_frame_.empty()) {
      tf_child_frame_ = model_->GetName();
    }
    tf_child_frame_ += "/" + base_frame_;

    const auto update_gate = update_gate_;
    update_connection_ =
        gazebo::event::Events::ConnectWorldUpdateBegin([update_gate](const gazebo::common::UpdateInfo& info) {
          std::lock_guard<std::mutex> lock(update_gate->mutex);
          if (update_gate->owner) {
            update_gate->owner->OnUpdate(info);
          }
        });

    ROS_INFO_STREAM("[gazebo_sim_mecanum] model='" << model_->GetName() << "' namespace='"
                                                    << ros_node_->getNamespace() << "' drive_model='"
                                                    << (high_fidelity_ ? "high_fidelity" : "ideal")
                                                    << "' ground truth pose='" << kSimulationGroundTruthPoseTopic
                                                    << "' twist='" << kSimulationGroundTruthTwistTopic
                                                    << "' uses Gazebo-owned integration");
  }

  void Reset() override {
    next_state_publish_time_ = 0.0;
    next_visual_publish_time_ = 0.0;
    next_imu_publish_time_ = 0.0;
    next_power_voltage_publish_time_ = 0.0;
    last_visual_update_time_ = 0.0;
    last_update_time_ = 0.0;
    wheel_positions_.assign(4, 0.0);
  }

 private:
  void Shutdown() {
    if (shutting_down_.exchange(true, std::memory_order_acq_rel)) {
      return;
    }

    update_connection_.reset();
    const auto update_gate = update_gate_;
    if (update_gate) {
      std::lock_guard<std::mutex> lock(update_gate->mutex);
      update_gate->owner = nullptr;
    }

    command_queue_.disable();
    command_subscriber_.shutdown();
    if (command_spinner_) {
      command_spinner_->stop();
      command_spinner_.reset();
    }
    command_queue_.clear();

    joint_states_publisher_.shutdown();
    power_voltage_publisher_.shutdown();
    imu_publisher_.shutdown();
    twist_publisher_.shutdown();
    pose_publisher_.shutdown();
    ros_node_.reset();

    for (auto& joint : wheel_joints_) {
      joint.reset();
    }
    body_link_.reset();
    model_.reset();
    update_gate_.reset();
  }

  void CommandCallback(const geometry_msgs::Twist::ConstPtr& command) {
    if (shutting_down_.load(std::memory_order_acquire)) {
      return;
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    front_velocity_command_ = linear_scale_ * ClampFinite(command->linear.x, max_front_velocity_);
    left_velocity_command_ = linear_scale_ * ClampFinite(command->linear.y, max_left_velocity_);
    yaw_velocity_command_ = angular_scale_ * ClampFinite(command->angular.z, max_yaw_velocity_);
  }

  void OnUpdate(const gazebo::common::UpdateInfo& info) {
    if (shutting_down_.load(std::memory_order_acquire) || !ros_node_ || !ros::ok()) {
      return;
    }
    const double now = info.simTime.Double();
    double dt = 0.0;
    if (last_update_time_ > 0.0 && now >= last_update_time_) {
      dt = now - last_update_time_;
    }
    last_update_time_ = now;
    if (high_fidelity_) {
      ApplyWheelDynamics(dt);
    } else {
      ApplyPlanarVelocity();
    }
    if (next_state_publish_time_ == 0.0 || now + 1.0e-9 >= next_state_publish_time_) {
      PublishState(now);
      AdvanceDeadline(now, 1.0 / state_publish_rate_, &next_state_publish_time_);
    }
    if (next_visual_publish_time_ == 0.0 || now + 1.0e-9 >= next_visual_publish_time_) {
      PublishVisualizationState(now);
      AdvanceDeadline(now, 1.0 / visual_publish_rate_, &next_visual_publish_time_);
    }
    if (next_imu_publish_time_ == 0.0 || now + 1.0e-9 >= next_imu_publish_time_) {
      PublishImu(now);
      AdvanceDeadline(now, 1.0 / imu_publish_rate_, &next_imu_publish_time_);
    }
    if (next_power_voltage_publish_time_ == 0.0 || now + 1.0e-9 >= next_power_voltage_publish_time_) {
      PublishPowerVoltage();
      AdvanceDeadline(now, 1.0 / power_voltage_publish_rate_, &next_power_voltage_publish_time_);
    }
  }

  void ApplyPlanarVelocity() {
    double front_velocity;
    double left_velocity;
    double yaw_velocity;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      front_velocity = front_velocity_command_;
      left_velocity = left_velocity_command_;
      yaw_velocity = yaw_velocity_command_;
    }
    const double yaw = model_->WorldPose().Rot().Yaw();
    model_->SetLinearVel(ignition::math::Vector3d(front_velocity * std::cos(yaw) - left_velocity * std::sin(yaw),
                                                  front_velocity * std::sin(yaw) + left_velocity * std::cos(yaw),
                                                  0.0));
    model_->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, yaw_velocity));
  }

  void ApplyWheelDynamics(double dt) {
    double front_velocity;
    double left_velocity;
    double yaw_velocity;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      front_velocity = front_velocity_command_;
      left_velocity = left_velocity_command_;
      yaw_velocity = yaw_velocity_command_;
    }

    const std::array<double, kWheelCount> lateral_sign = {-1.0, 1.0, 1.0, -1.0};
    const std::array<double, kWheelCount> yaw_sign = {-1.0, 1.0, -1.0, 1.0};
    const std::array<double, kWheelCount> joint_sign = {1.0, -1.0, 1.0, -1.0};
    const ignition::math::Pose3d pose = model_->WorldPose();
    const ignition::math::Vector3d body_velocity = pose.Rot().RotateVectorReverse(model_->WorldLinearVel());
    const double body_yaw_rate = model_->WorldAngularVel().Z();
    const double max_traction = friction_coefficient_ * normal_force_per_wheel_;
    double body_force_x = -linear_drag_ * body_velocity.X();
    double body_force_y = -linear_drag_ * body_velocity.Y();
    double body_torque_z = -angular_drag_ * body_yaw_rate;

    for (std::size_t index = 0; index < kWheelCount; ++index) {
      const double requested_ground_speed = front_velocity + lateral_sign[index] * left_velocity +
                                            yaw_sign[index] * wheelbase_sum_ * yaw_velocity;
      const double target_joint_speed = joint_sign[index] * requested_ground_speed / wheel_radius_;
      const double joint_speed = wheel_joints_[index]->GetVelocity(0);
      const double effective_wheel_speed = joint_sign[index] * joint_speed;
      const double contact_ground_speed = body_velocity.X() + lateral_sign[index] * body_velocity.Y() +
                                          yaw_sign[index] * wheelbase_sum_ * body_yaw_rate;
      const double slip_speed = wheel_radius_ * effective_wheel_speed - contact_ground_speed;
      const double traction = ClampFinite(traction_gain_ * slip_speed, max_traction);
      const double motor_torque =
          ClampFinite(wheel_pid_p_ * (target_joint_speed - joint_speed), wheel_torque_limit_);
      const double reaction_torque = joint_sign[index] * wheel_radius_ * traction;
      wheel_joints_[index]->SetForce(0, motor_torque - reaction_torque);

      // The 1/2 factor is the virtual-work mapping for the 45-degree roller
      // directions used by the standard four-wheel Mecanum Jacobian.
      body_force_x += 0.5 * traction;
      body_force_y += 0.5 * lateral_sign[index] * traction;
      body_torque_z += 0.5 * yaw_traction_scale_ * yaw_sign[index] * wheelbase_sum_ * traction;
    }

    if (dt > 0.0 && std::isfinite(body_force_x) && std::isfinite(body_force_y) &&
        std::isfinite(body_torque_z)) {
      body_link_->AddRelativeForce(ignition::math::Vector3d(body_force_x, body_force_y, 0.0));
      body_link_->AddRelativeTorque(ignition::math::Vector3d(0.0, 0.0, body_torque_z));
    }
  }

  static void AdvanceDeadline(double now, double period, double* deadline) {
    if (*deadline <= 0.0 || now + period < *deadline) {
      *deadline = now + period;
      return;
    }
    do {
      *deadline += period;
    } while (*deadline <= now + 1.0e-9);
  }

  void PublishState(double sim_time) {
    const ignition::math::Pose3d pose = model_->WorldPose();
    const ignition::math::Vector3d linear_velocity = model_->WorldLinearVel();
    const ignition::math::Vector3d angular_velocity = model_->WorldAngularVel();
    ros::Time stamp;
    stamp.fromSec(sim_time);

    tf2::Quaternion pose_quaternion;
    const double pose_yaw = pose.Rot().Yaw();
    pose_quaternion.setRPY(0.0, 0.0, use_imu_orientation_ ? pose_yaw - imu_yaw_base_ : pose_yaw);

    geometry_msgs::PoseStamped pose_message;
    pose_message.header.stamp = stamp;
    pose_message.header.frame_id = map_frame_;
    pose_message.pose.position.x = pose.Pos().X();
    pose_message.pose.position.y = pose.Pos().Y();
    pose_message.pose.position.z = pose.Pos().Z();
    pose_message.pose.orientation.x = pose_quaternion.x();
    pose_message.pose.orientation.y = pose_quaternion.y();
    pose_message.pose.orientation.z = pose_quaternion.z();
    pose_message.pose.orientation.w = pose_quaternion.w();
    pose_publisher_.publish(pose_message);

    geometry_msgs::TwistStamped twist_message;
    twist_message.header.stamp = stamp;
    twist_message.header.frame_id = map_frame_;
    twist_message.twist.linear.x = linear_velocity.X();
    twist_message.twist.linear.y = linear_velocity.Y();
    twist_message.twist.linear.z = linear_velocity.Z();
    twist_message.twist.angular.x = angular_velocity.X();
    twist_message.twist.angular.y = angular_velocity.Y();
    twist_message.twist.angular.z = angular_velocity.Z();
    twist_publisher_.publish(twist_message);
  }

  void PublishImu(double sim_time) {
    const ignition::math::Pose3d pose = model_->WorldPose();
    const ignition::math::Vector3d angular_velocity = model_->WorldAngularVel();
    ros::Time stamp;
    stamp.fromSec(sim_time);

    tf2::Quaternion imu_quaternion;
    imu_quaternion.setRPY(0.0, 0.0, pose.Rot().Yaw() - imu_yaw_base_);
    sensor_msgs::Imu imu_message;
    imu_message.header.stamp = stamp;
    imu_message.header.frame_id = imu_frame_;
    imu_message.orientation.x = imu_quaternion.x();
    imu_message.orientation.y = imu_quaternion.y();
    imu_message.orientation.z = imu_quaternion.z();
    imu_message.orientation.w = imu_quaternion.w();
    imu_message.angular_velocity.x = angular_velocity.X();
    imu_message.angular_velocity.y = angular_velocity.Y();
    imu_message.angular_velocity.z = angular_velocity.Z();
    imu_publisher_.publish(imu_message);
  }

  void PublishPowerVoltage() {
    std_msgs::Float32 message;
    message.data = static_cast<float>(battery_voltage_);
    power_voltage_publisher_.publish(message);
  }

  void PublishVisualizationState(double sim_time) {
    const ignition::math::Pose3d pose = model_->WorldPose();
    const ignition::math::Vector3d world_velocity = model_->WorldLinearVel();
    const double yaw = pose.Rot().Yaw();
    const double front_velocity = std::cos(yaw) * world_velocity.X() + std::sin(yaw) * world_velocity.Y();
    const double left_velocity = -std::sin(yaw) * world_velocity.X() + std::cos(yaw) * world_velocity.Y();
    const double yaw_velocity = model_->WorldAngularVel().Z();

    // joint_states is a renderer contract, not a leak of simulator-internal
    // wheel speeds. Derive animation from measured body motion so the same
    // visualization path also works for a physical robot.
    double dt = 0.0;
    if (last_visual_update_time_ > 0.0 && sim_time >= last_visual_update_time_) {
      dt = sim_time - last_visual_update_time_;
    }
    constexpr double wheel_radius = 0.05;
    constexpr double half_wheelbase_plus_half_track = 0.30;
    const double wheel_velocity[4] = {
        (front_velocity - left_velocity - yaw_velocity * half_wheelbase_plus_half_track) / wheel_radius,
        -(front_velocity + left_velocity + yaw_velocity * half_wheelbase_plus_half_track) / wheel_radius,
        (front_velocity + left_velocity - yaw_velocity * half_wheelbase_plus_half_track) / wheel_radius,
        -(front_velocity - left_velocity + yaw_velocity * half_wheelbase_plus_half_track) / wheel_radius,
    };
    for (std::size_t index = 0; index < wheel_positions_.size(); ++index) {
      wheel_positions_[index] =
          std::remainder(wheel_positions_[index] + wheel_velocity[index] * dt, 2.0 * kPi);
    }
    last_visual_update_time_ = sim_time;

    ros::Time stamp;
    stamp.fromSec(sim_time);
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = stamp;
    joint_state.name = {"upper_left_wheel_joint", "upper_right_wheel_joint", "lower_left_wheel_joint",
                        "lower_right_wheel_joint"};
    joint_state.position = wheel_positions_;
    joint_states_publisher_.publish(joint_state);

    geometry_msgs::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = tf_child_frame_;
    transform.transform.translation.x = pose.Pos().X();
    transform.transform.translation.y = pose.Pos().Y();
    transform.transform.translation.z = pose.Pos().Z();
    transform.transform.rotation.x = pose.Rot().X();
    transform.transform.rotation.y = pose.Rot().Y();
    transform.transform.rotation.z = pose.Rot().Z();
    transform.transform.rotation.w = pose.Rot().W();
    tf_broadcaster_.sendTransform(transform);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr body_link_;
  std::array<gazebo::physics::JointPtr, kWheelCount> wheel_joints_{};
  std::atomic<bool> shutting_down_{false};
  std::shared_ptr<UpdateGate> update_gate_;
  gazebo::event::ConnectionPtr update_connection_;
  std::unique_ptr<ros::NodeHandle> ros_node_;
  ros::CallbackQueue command_queue_;
  std::unique_ptr<ros::AsyncSpinner> command_spinner_;
  ros::Subscriber command_subscriber_;
  ros::Publisher pose_publisher_;
  ros::Publisher twist_publisher_;
  ros::Publisher imu_publisher_;
  ros::Publisher joint_states_publisher_;
  ros::Publisher power_voltage_publisher_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  std::mutex command_mutex_;
  double front_velocity_command_{0.0};
  double left_velocity_command_{0.0};
  double yaw_velocity_command_{0.0};

  double state_publish_rate_{100.0};
  double visual_publish_rate_{20.0};
  double imu_publish_rate_{kDefaultImuRate};
  double power_voltage_publish_rate_{kDefaultPowerVoltageRate};
  double battery_voltage_{kDefaultBatteryVoltage};
  double max_front_velocity_{kSssMaxFrontVelocity};
  double max_left_velocity_{kSssMaxLeftVelocity};
  double max_yaw_velocity_{kSssMaxYawVelocity};
  double linear_scale_{1.0};
  double angular_scale_{1.0};
  double imu_yaw_base_{kPi / 4.0};
  bool use_imu_orientation_{false};
  bool high_fidelity_{true};
  std::string drive_model_{"high_fidelity"};
  double wheel_radius_{0.05};
  double wheelbase_sum_{0.30};
  double wheel_pid_p_{0.35};
  double wheel_torque_limit_{1.2};
  double traction_gain_{18.0};
  double friction_coefficient_{0.85};
  double normal_force_per_wheel_{0.0};
  double linear_drag_{0.10};
  double angular_drag_{0.02};
  double yaw_traction_scale_{0.45};
  std::string map_frame_{"map"};
  std::string imu_frame_{"world"};
  std::string base_frame_{"base_footprint"};
  std::string tf_child_frame_;
  double next_state_publish_time_{0.0};
  double next_visual_publish_time_{0.0};
  double next_imu_publish_time_{0.0};
  double next_power_voltage_publish_time_{0.0};
  double last_visual_update_time_{0.0};
  double last_update_time_{0.0};
  std::vector<double> wheel_positions_{0.0, 0.0, 0.0, 0.0};
};

GZ_REGISTER_MODEL_PLUGIN(MecanumContractPlugin)

}  // namespace gazebo_sim_mecanum
