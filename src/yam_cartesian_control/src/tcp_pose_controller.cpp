#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>


class TcpPoseController : public rclcpp::Node
{
public:
  TcpPoseController()
  : Node("tcp_pose_controller")
  {
    planning_group_ = declare_parameter<std::string>(
      "planning_group", "yam_arm");

    base_frame_ = declare_parameter<std::string>(
      "base_frame", "base");

    ee_link_ = declare_parameter<std::string>(
      "ee_link", "grasp_point");

    position_kp_ = declare_parameter<double>(
      "position_kp", 2.0);

    orientation_kp_ = declare_parameter<double>(
      "orientation_kp", 2.0);

    control_rate_ = declare_parameter<double>(
      "control_rate", 100.0);

    target_timeout_ = declare_parameter<double>(
      "target_timeout", 0.25);

    joint_state_sub_ =
      create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        50,
        std::bind(
          &TcpPoseController::jointStateCallback,
          this,
          std::placeholders::_1));

    target_pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseStamped>(
        "/yam_follower/cartesian_pose_cmd",
        20,
        std::bind(
          &TcpPoseController::targetPoseCallback,
          this,
          std::placeholders::_1));

    twist_pub_ =
      create_publisher<geometry_msgs::msg::TwistStamped>(
        "/yam_follower/cartesian_twist_cmd",
        20);

    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_),
      std::bind(&TcpPoseController::controlLoop, this));

    RCLCPP_INFO(
      get_logger(),
      "TcpPoseController created. pose_cmd=/yam_follower/cartesian_pose_cmd "
      "twist_cmd=/yam_follower/cartesian_twist_cmd");
  }


  void initialize(const rclcpp::Node::SharedPtr & node)
  {
    robot_model_loader_ =
      std::make_shared<robot_model_loader::RobotModelLoader>(
        node,
        "robot_description");

    robot_model_ = robot_model_loader_->getModel();

    if (!robot_model_) {
      throw std::runtime_error("Failed to load robot model");
    }

    robot_state_ =
      std::make_shared<moveit::core::RobotState>(robot_model_);

    robot_state_->setToDefaultValues();

    joint_model_group_ =
      robot_model_->getJointModelGroup(planning_group_);

    if (!joint_model_group_) {
      throw std::runtime_error(
        "Planning group not found: " + planning_group_);
    }

    base_link_model_ =
      robot_model_->getLinkModel(base_frame_);

    if (!base_link_model_) {
      throw std::runtime_error(
        "Base link not found: " + base_frame_);
    }

    ee_link_model_ =
      robot_model_->getLinkModel(ee_link_);

    if (!ee_link_model_) {
      throw std::runtime_error(
        "EE link not found: " + ee_link_);
    }

    joint_names_ = joint_model_group_->getVariableNames();

    have_model_ = true;

    RCLCPP_INFO(
      get_logger(),
      "MoveIt model initialized | group=%s base=%s ee=%s "
      "Kp_pos=%.3f Kp_rot=%.3f rate=%.1fHz timeout=%.3fs",
      planning_group_.c_str(),
      base_frame_.c_str(),
      ee_link_.c_str(),
      position_kp_,
      orientation_kp_,
      control_rate_,
      target_timeout_);
  }


private:
  static bool finitePose(const geometry_msgs::msg::Pose & pose)
  {
    const double values[] = {
      pose.position.x,
      pose.position.y,
      pose.position.z,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w
    };

    for (double value : values) {
      if (!std::isfinite(value)) {
        return false;
      }
    }

    return true;
  }


  void jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!have_model_ || !robot_state_) {
      return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);

    try {
      robot_state_->setVariableValues(*msg);
      robot_state_->update();

      for (const auto & joint_name : joint_names_) {
        auto it = std::find(
          msg->name.begin(),
          msg->name.end(),
          joint_name);

        if (it == msg->name.end()) {
          return;
        }
      }

      have_joint_state_ = true;
    }
    catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Failed to update RobotState from /joint_states: %s",
        e.what());
    }
  }


  void targetPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!have_model_) {
      return;
    }

    if (!msg->header.frame_id.empty() &&
        msg->header.frame_id != base_frame_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Pose frame must be '%s', received '%s'",
        base_frame_.c_str(),
        msg->header.frame_id.c_str());
      return;
    }

    if (!finitePose(msg->pose)) {
      RCLCPP_WARN(get_logger(), "Rejected non-finite TCP pose command");
      return;
    }

    Eigen::Quaterniond q_des(
      msg->pose.orientation.w,
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z);

    if (q_des.norm() < 1e-9) {
      RCLCPP_WARN(get_logger(), "Rejected zero-norm TCP quaternion");
      return;
    }

    q_des.normalize();

    const Eigen::Vector3d p_des(
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z);

    const rclcpp::Time receipt_time = now();

    std::lock_guard<std::mutex> lock(target_mutex_);

    Eigen::Vector3d new_linear_ff = Eigen::Vector3d::Zero();
    Eigen::Vector3d new_angular_ff = Eigen::Vector3d::Zero();

    if (have_target_) {
      const double dt = (receipt_time - last_target_time_).seconds();

      const Eigen::Vector3d position_step =
        p_des - target_position_;

      const Eigen::Matrix3d R_delta =
        q_des.toRotationMatrix() *
        target_orientation_.toRotationMatrix().transpose();

      Eigen::AngleAxisd step_aa(R_delta);

      double rotation_step = 0.0;

      if (std::isfinite(step_aa.angle()) &&
          step_aa.axis().allFinite())
      {
        rotation_step = std::abs(step_aa.angle());
      }

      if (dt > 1e-3 && dt < 0.2) {
        new_linear_ff = position_step / dt;

        if (std::isfinite(step_aa.angle()) &&
            step_aa.axis().allFinite())
        {
          new_angular_ff =
            step_aa.axis() * step_aa.angle() / dt;
        }
      }

      constexpr double RAD_TO_DEG =
        57.29577951308232;

      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        500,
        "[POSE RX] dt=%.2f ms rate=%.2f Hz "
        "step=%.3f mm %.3f deg "
        "ff=%.4f m/s %.4f rad/s",
        dt * 1000.0,
        dt > 1e-9 ? 1.0 / dt : 0.0,
        position_step.norm() * 1000.0,
        rotation_step * RAD_TO_DEG,
        new_linear_ff.norm(),
        new_angular_ff.norm());
    }

    target_position_ = p_des;
    target_orientation_ = q_des;
    target_linear_ff_ = new_linear_ff;
    target_angular_ff_ = new_angular_ff;
    last_target_time_ = receipt_time;
    have_target_ = true;
  }


  void publishZero()
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = base_frame_;
    twist_pub_->publish(msg);
  }


  void controlLoop()
  {
    if (!have_model_ || !have_joint_state_) {
      publishZero();
      return;
    }

    Eigen::Vector3d p_des;
    Eigen::Quaterniond q_des;
    Eigen::Vector3d linear_ff;
    Eigen::Vector3d angular_ff;
    rclcpp::Time last_target_time(0, 0, get_clock()->get_clock_type());

    {
      std::lock_guard<std::mutex> lock(target_mutex_);

      if (!have_target_) {
        publishZero();
        return;
      }

      p_des = target_position_;
      q_des = target_orientation_;
      linear_ff = target_linear_ff_;
      angular_ff = target_angular_ff_;
      last_target_time = last_target_time_;
    }

    const double target_age = (now() - last_target_time).seconds();

    if (target_age > target_timeout_) {
      publishZero();

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "TCP pose command timeout: %.3f s > %.3f s; publishing zero Twist",
        target_age,
        target_timeout_);

      return;
    }

    Eigen::Isometry3d T_base_ee;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);

      const Eigen::Isometry3d & T_model_base =
        robot_state_->getGlobalLinkTransform(base_link_model_);

      const Eigen::Isometry3d & T_model_ee =
        robot_state_->getGlobalLinkTransform(ee_link_model_);

      T_base_ee = T_model_base.inverse() * T_model_ee;
    }

    const Eigen::Vector3d p_current = T_base_ee.translation();
    const Eigen::Matrix3d R_current = T_base_ee.rotation();
    const Eigen::Matrix3d R_desired = q_des.toRotationMatrix();

    const Eigen::Vector3d position_error =
      p_des - p_current;

    // Spatial/base-frame orientation error.
    const Eigen::Matrix3d R_error =
      R_desired * R_current.transpose();

    Eigen::AngleAxisd aa(R_error);

    Eigen::Vector3d orientation_error =
      Eigen::Vector3d::Zero();

    if (std::isfinite(aa.angle()) &&
        aa.axis().allFinite())
    {
      orientation_error = aa.axis() * aa.angle();
    }

    const Eigen::Vector3d linear_command =
      linear_ff + position_kp_ * position_error;

    const Eigen::Vector3d angular_command =
      angular_ff + orientation_kp_ * orientation_error;

    geometry_msgs::msg::TwistStamped output;
    output.header.stamp = now();
    output.header.frame_id = base_frame_;

    output.twist.linear.x = linear_command.x();
    output.twist.linear.y = linear_command.y();
    output.twist.linear.z = linear_command.z();

    output.twist.angular.x = angular_command.x();
    output.twist.angular.y = angular_command.y();
    output.twist.angular.z = angular_command.z();

    twist_pub_->publish(output);

    const Eigen::Vector3d linear_fb =
      position_kp_ * position_error;

    const Eigen::Vector3d angular_fb =
      orientation_kp_ * orientation_error;

    constexpr double RAD_TO_DEG =
      57.29577951308232;

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      500,
      "[TCP CTRL] age=%.1f ms "
      "err=%.2f mm %.2f deg | "
      "ff=%.4f %.4f | "
      "fb=%.4f %.4f | "
      "cmd=%.4f %.4f",
      target_age * 1000.0,
      1000.0 * position_error.norm(),
      RAD_TO_DEG * orientation_error.norm(),
      linear_ff.norm(),
      angular_ff.norm(),
      linear_fb.norm(),
      angular_fb.norm(),
      linear_command.norm(),
      angular_command.norm());
  }


  // MoveIt
  std::shared_ptr<robot_model_loader::RobotModelLoader>
    robot_model_loader_;

  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;

  const moveit::core::JointModelGroup *
    joint_model_group_{nullptr};

  const moveit::core::LinkModel *
    base_link_model_{nullptr};

  const moveit::core::LinkModel *
    ee_link_model_{nullptr};

  std::vector<std::string> joint_names_;

  std::mutex state_mutex_;
  bool have_model_{false};
  bool have_joint_state_{false};

  // Target
  std::mutex target_mutex_;

  Eigen::Vector3d target_position_ =
    Eigen::Vector3d::Zero();

  Eigen::Quaterniond target_orientation_ =
    Eigen::Quaterniond::Identity();

  Eigen::Vector3d target_linear_ff_ =
    Eigen::Vector3d::Zero();

  Eigen::Vector3d target_angular_ff_ =
    Eigen::Vector3d::Zero();

  rclcpp::Time last_target_time_{0, 0, RCL_ROS_TIME};
  bool have_target_{false};

  // ROS
  rclcpp::Subscription<
    sensor_msgs::msg::JointState>::SharedPtr
    joint_state_sub_;

  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    target_pose_sub_;

  rclcpp::Publisher<
    geometry_msgs::msg::TwistStamped>::SharedPtr
    twist_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  // Parameters
  std::string planning_group_;
  std::string base_frame_;
  std::string ee_link_;

  double position_kp_;
  double orientation_kp_;
  double control_rate_;
  double target_timeout_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TcpPoseController>();

  node->initialize(node);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}