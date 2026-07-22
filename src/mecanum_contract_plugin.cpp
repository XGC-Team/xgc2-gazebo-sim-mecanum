#include <algorithm>
#include <cmath>
#include <functional>
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

}  // namespace

// This plugin sets an ideal planar velocity and publishes the ROS contract. It
// never integrates or writes a pose and never applies a force. Gazebo remains
// the sole owner of velocity-to-pose integration, pause, and reset semantics.
class MecanumContractPlugin final : public gazebo::ModelPlugin {
 public:
  MecanumContractPlugin() = default;

  ~MecanumContractPlugin() override {
    update_connection_.reset();
    command_subscriber_.shutdown();
    if (command_spinner_) {
      command_spinner_->stop();
      command_spinner_.reset();
    }
    ros_node_.reset();
  }

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
    const std::string pose_topic = SdfValue<std::string>(sdf, "poseTopic", "pose");
    const std::string twist_topic = SdfValue<std::string>(sdf, "twistTopic", "twist");
    const std::string imu_topic = SdfValue<std::string>(sdf, "imuTopic", "imu");
    const std::string joint_states_topic =
        SdfValue<std::string>(sdf, "jointStatesTopic", "joint_states");

    state_publish_rate_ = SdfValue<double>(sdf, "statePublishRate", 100.0);
    visual_publish_rate_ = SdfValue<double>(sdf, "visualPublishRate", 20.0);
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

    state_publish_rate_ = std::max(1.0, state_publish_rate_);
    visual_publish_rate_ = std::max(1.0, visual_publish_rate_);
    max_front_velocity_ = std::abs(max_front_velocity_);
    max_left_velocity_ = std::abs(max_left_velocity_);
    max_yaw_velocity_ = std::abs(max_yaw_velocity_);

    pose_publisher_ = ros_node_->advertise<geometry_msgs::PoseStamped>(pose_topic, 10, false);
    twist_publisher_ = ros_node_->advertise<geometry_msgs::TwistStamped>(twist_topic, 10, false);
    imu_publisher_ = ros_node_->advertise<sensor_msgs::Imu>(imu_topic, 10, false);
    joint_states_publisher_ = ros_node_->advertise<sensor_msgs::JointState>(joint_states_topic, 1, false);
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

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&MecanumContractPlugin::OnUpdate, this, std::placeholders::_1));

    ROS_INFO_STREAM("[gazebo_sim_mecanum] model='" << model_->GetName() << "' namespace='"
                                                    << ros_node_->getNamespace()
                                                    << "' uses Gazebo-owned planar integration");
  }

  void Reset() override {
    next_state_publish_time_ = 0.0;
    next_visual_publish_time_ = 0.0;
    last_visual_update_time_ = 0.0;
    wheel_positions_.assign(4, 0.0);
  }

 private:
  void CommandCallback(const geometry_msgs::Twist::ConstPtr& command) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    front_velocity_command_ = linear_scale_ * ClampFinite(command->linear.x, max_front_velocity_);
    left_velocity_command_ = linear_scale_ * ClampFinite(command->linear.y, max_left_velocity_);
    yaw_velocity_command_ = angular_scale_ * ClampFinite(command->angular.z, max_yaw_velocity_);
  }

  void OnUpdate(const gazebo::common::UpdateInfo& info) {
    if (!ros_node_ || !ros::ok()) {
      return;
    }
    const double now = info.simTime.Double();
    ApplyPlanarVelocity();
    if (next_state_publish_time_ == 0.0 || now + 1.0e-9 >= next_state_publish_time_) {
      PublishState(now);
      AdvanceDeadline(now, 1.0 / state_publish_rate_, &next_state_publish_time_);
    }
    if (next_visual_publish_time_ == 0.0 || now + 1.0e-9 >= next_visual_publish_time_) {
      PublishVisualizationState(now);
      AdvanceDeadline(now, 1.0 / visual_publish_rate_, &next_visual_publish_time_);
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

    tf2::Quaternion imu_quaternion;
    imu_quaternion.setRPY(0.0, 0.0, pose_yaw - imu_yaw_base_);
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

  void PublishVisualizationState(double sim_time) {
    const ignition::math::Pose3d pose = model_->WorldPose();
    const ignition::math::Vector3d world_velocity = model_->WorldLinearVel();
    const double yaw = pose.Rot().Yaw();
    const double front_velocity = std::cos(yaw) * world_velocity.X() + std::sin(yaw) * world_velocity.Y();
    const double left_velocity = -std::sin(yaw) * world_velocity.X() + std::cos(yaw) * world_velocity.Y();
    const double yaw_velocity = model_->WorldAngularVel().Z();

    double dt = 0.0;
    if (last_visual_update_time_ > 0.0 && sim_time >= last_visual_update_time_) {
      dt = sim_time - last_visual_update_time_;
    }
    last_visual_update_time_ = sim_time;

    constexpr double wheel_radius = 0.05;
    constexpr double half_wheelbase_plus_half_track = 0.30;
    const double wheel_velocity[4] = {
        (front_velocity - left_velocity - yaw_velocity * half_wheelbase_plus_half_track) / wheel_radius,
        -(front_velocity + left_velocity + yaw_velocity * half_wheelbase_plus_half_track) / wheel_radius,
        (front_velocity + left_velocity - yaw_velocity * half_wheelbase_plus_half_track) / wheel_radius,
        -(front_velocity - left_velocity + yaw_velocity * half_wheelbase_plus_half_track) / wheel_radius,
    };
    for (std::size_t index = 0; index < wheel_positions_.size(); ++index) {
      wheel_positions_[index] = std::remainder(wheel_positions_[index] + wheel_velocity[index] * dt, 2.0 * kPi);
    }

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
  gazebo::event::ConnectionPtr update_connection_;
  std::unique_ptr<ros::NodeHandle> ros_node_;
  ros::CallbackQueue command_queue_;
  std::unique_ptr<ros::AsyncSpinner> command_spinner_;
  ros::Subscriber command_subscriber_;
  ros::Publisher pose_publisher_;
  ros::Publisher twist_publisher_;
  ros::Publisher imu_publisher_;
  ros::Publisher joint_states_publisher_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  std::mutex command_mutex_;
  double front_velocity_command_{0.0};
  double left_velocity_command_{0.0};
  double yaw_velocity_command_{0.0};

  double state_publish_rate_{100.0};
  double visual_publish_rate_{20.0};
  double max_front_velocity_{kSssMaxFrontVelocity};
  double max_left_velocity_{kSssMaxLeftVelocity};
  double max_yaw_velocity_{kSssMaxYawVelocity};
  double linear_scale_{1.0};
  double angular_scale_{1.0};
  double imu_yaw_base_{kPi / 4.0};
  bool use_imu_orientation_{false};
  std::string map_frame_{"map"};
  std::string imu_frame_{"world"};
  std::string base_frame_{"base_footprint"};
  std::string tf_child_frame_;
  double next_state_publish_time_{0.0};
  double next_visual_publish_time_{0.0};
  double last_visual_update_time_{0.0};
  std::vector<double> wheel_positions_{0.0, 0.0, 0.0, 0.0};
};

GZ_REGISTER_MODEL_PLUGIN(MecanumContractPlugin)

}  // namespace gazebo_sim_mecanum
