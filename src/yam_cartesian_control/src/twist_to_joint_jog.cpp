#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <mutex>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cmath>


class TwistToJointJog : public rclcpp::Node
{
public:
  TwistToJointJog()
  : Node("twist_to_joint_jog")
  {
    // ==================================================
    // Parameters
    // ==================================================
    planning_group_ = this->declare_parameter<std::string>(
      "planning_group", "yam_arm");

    base_frame_ = this->declare_parameter<std::string>(
      "base_frame", "base");

    ee_link_ = this->declare_parameter<std::string>(
      "ee_link", "grasp_point");

    max_joint_velocity_ = this->declare_parameter<double>(
      "max_joint_velocity", 0.30);

    max_linear_velocity_ = this->declare_parameter<double>(
      "max_linear_velocity", 0.10);

    max_angular_velocity_ = this->declare_parameter<double>(
      "max_angular_velocity", 0.50);

    max_linear_acceleration_ = this->declare_parameter<double>(
      "max_linear_acceleration", 0.30);

    max_angular_acceleration_ = this->declare_parameter<double>(
      "max_angular_acceleration", 1.00);

    joint_limit_slow_margin_ =
      this->declare_parameter<double>("joint_limit_slow_margin", 0.15);

    joint_limit_stop_margin_ =
      this->declare_parameter<double>("joint_limit_stop_margin", 0.05);
            

    damping_ = this->declare_parameter<double>("damping", 0.01);


    // ==================================================
    // Subscriber: /joint_states
    // ==================================================
    joint_state_sub_ =
      create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        50,
        std::bind(
          &TwistToJointJog::jointStateCallback,
          this,
          std::placeholders::_1));


    // ==================================================
    // Subscriber: Cartesian Twist
    // ==================================================
    twist_sub_ =
      create_subscription<geometry_msgs::msg::TwistStamped>(
        "/yam_follower/cartesian_twist_cmd",
        20,
        std::bind(
          &TwistToJointJog::twistCallback,
          this,
          std::placeholders::_1));


    // ==================================================
    // Publisher: raw joint velocity command
    // ==================================================
    joint_jog_pub_ =
      create_publisher<control_msgs::msg::JointJog>(
        "/yam_follower/raw_joint_velocity_cmd",
        20);

    diagnostics_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/yam_follower/dls_diagnostics",
        20);


    RCLCPP_INFO(
      get_logger(),
      "TwistToJointJog node created.");
  }


  // ==================================================
  // Initialize MoveIt RobotModel
  // ==================================================
  void initialize(const rclcpp::Node::SharedPtr & node)
  {
    robot_model_loader_ =
      std::make_shared<robot_model_loader::RobotModelLoader>(
        node,
        "robot_description");

    robot_model_ =
      robot_model_loader_->getModel();

    if (!robot_model_)
    {
      throw std::runtime_error(
        "Failed to load robot model");
    }


    robot_state_ =
      std::make_shared<moveit::core::RobotState>(
        robot_model_);

    robot_state_->setToDefaultValues();


    joint_model_group_ =
      robot_model_->getJointModelGroup(
        planning_group_);

    if (!joint_model_group_)
    {
      throw std::runtime_error(
        "Planning group not found: " +
        planning_group_);
    }


    ee_link_model_ =
      robot_model_->getLinkModel(
        ee_link_);

    if (!ee_link_model_)
    {
      throw std::runtime_error(
        "EE link not found: " +
        ee_link_);
    }


    joint_names_ =
      joint_model_group_->getVariableNames();

    have_model_ = true;


    RCLCPP_INFO(
      get_logger(),
      "MoveIt model initialized.");

    RCLCPP_INFO(
      get_logger(),
      "Planning group: %s",
      planning_group_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Base frame: %s",
      base_frame_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "EE link: %s",
      ee_link_.c_str());


    RCLCPP_INFO(
      get_logger(),
      "damping_: %.4f",
      damping_);

    RCLCPP_INFO(
      get_logger(),
      "Max joint velocity: %.3f rad/s",
      max_joint_velocity_);

    for (const auto & name : joint_names_)
    {
      RCLCPP_INFO(
        get_logger(),
        "Joint: %s",
        name.c_str());
    }
  }


private:

  // ==================================================
  // JointState callback
  // ==================================================
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
		have_joint_state_ = true;

		std::vector<double> current_positions(joint_names_.size());

		for (size_t i = 0; i < joint_names_.size(); ++i) {
		  auto it = std::find(
		    msg->name.begin(),
		    msg->name.end(),
		    joint_names_[i]
		  );

		  if (it == msg->name.end()) {
		    return;
		  }

		  size_t index = std::distance(msg->name.begin(), it);
		  current_positions[i] = msg->position[index];
		}

		rclcpp::Time current_time = now();

		if (!have_prev_joint_state_) {
		  prev_joint_positions_ = current_positions;
		  prev_joint_time_ = current_time;
		  have_prev_joint_state_ = true;
		  return;
		}

		double dt = (current_time - prev_joint_time_).seconds();

		if (dt >= qdot_window_) {
		  actual_qdot_.resize(joint_names_.size());

		  for (size_t i = 0; i < joint_names_.size(); ++i) {
		    actual_qdot_[i] =
		      (current_positions[i] - prev_joint_positions_[i]) / dt;
		  }

		  prev_joint_positions_ = current_positions;
		  prev_joint_time_ = current_time;
		  have_actual_qdot_ = true;
		}
	  }
	  catch (const std::exception & e) {
		RCLCPP_WARN_THROTTLE(
		  get_logger(),
		  *get_clock(),
		  1000,
		  "Failed to update RobotState from /joint_states: %s",
		  e.what()
		);
	  }
	}


  // ==================================================
  // Twist callback
  // ==================================================
  void twistCallback(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    if (!have_model_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Robot model is not initialized yet.");

      return;
    }


    if (!have_joint_state_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Waiting for /joint_states...");

      return;
    }


    // --------------------------------------------------
    // Only accept Twist expressed in base frame
    // --------------------------------------------------
    if (!msg->header.frame_id.empty() &&
        msg->header.frame_id != base_frame_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Twist frame must be '%s', received '%s'",
        base_frame_.c_str(),
        msg->header.frame_id.c_str());

      return;
    }


    // ==================================================
    // Compute Jacobian
    // ==================================================
    Eigen::MatrixXd jacobian;

    {
      std::lock_guard<std::mutex> lock(
        state_mutex_);

      bool ok =
        robot_state_->getJacobian(
          joint_model_group_,
          ee_link_model_,
          Eigen::Vector3d::Zero(),
          jacobian);

      if (!ok)
      {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to compute Jacobian");

        return;
      }
    }


    // ==================================================
    // Singular value analysis
    // ==================================================
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      jacobian,
      Eigen::ComputeThinU |
      Eigen::ComputeThinV);

    Eigen::VectorXd singular_values =
      svd.singularValues();

    double sigma_max =
      singular_values.maxCoeff();

    double sigma_min =
      singular_values.minCoeff();

    double condition_number =
      sigma_max /
      std::max(sigma_min, 1e-9);


    // ==================================================
    // Adaptive damping + velocity scaling
    // ==================================================
    double damping = damping_;

	  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "sigma_min=%.6f sigma_max=%.6f condition=%.2f damping=%.5f",
      sigma_min, sigma_max, condition_number, damping_
    );

    // ==================================================
    // Build Cartesian Twist vector
    // ==================================================
    Eigen::Matrix<double, 6, 1> twist;

    twist <<
      msg->twist.linear.x,
      msg->twist.linear.y,
      msg->twist.linear.z,
      msg->twist.angular.x,
      msg->twist.angular.y,
      msg->twist.angular.z;

  

    // ==================================================
    // Cartesian velocity limit
    // ==================================================
    Eigen::Vector3d linear = twist.head<3>();
    Eigen::Vector3d angular = twist.tail<3>();

    double linear_norm = linear.norm();
    double angular_norm = angular.norm();
    double linear_velocity_scale = 1.0;
    double angular_velocity_scale = 1.0;

    if (linear_norm > max_linear_velocity_) {
      linear_velocity_scale = max_linear_velocity_ / linear_norm;
      linear *= linear_velocity_scale;
    }

    if (angular_norm > max_angular_velocity_) {
      angular_velocity_scale = max_angular_velocity_ / angular_norm;
      angular *= angular_velocity_scale;
    }

    twist.head<3>() = linear;
    twist.tail<3>() = angular;

    // ==================================================
    // Cartesian acceleration limit
    // ==================================================
    rclcpp::Time current_twist_time = now();

    double twist_dt = 0.02;

    if (have_previous_twist_) {
      double measured_dt = (current_twist_time - previous_twist_time_).seconds();

      if (measured_dt > 0.0 && measured_dt < 0.2) {
        twist_dt = measured_dt;
      } else {
        // Command stream was interrupted.
        // Restart acceleration limiting from zero.
        previous_twist_.setZero();
      }
    } else {
      // First command starts from zero velocity.
      previous_twist_.setZero();
      have_previous_twist_ = true;
    }

    Eigen::Vector3d delta_linear =
      twist.head<3>() - previous_twist_.head<3>();

    Eigen::Vector3d delta_angular =
      twist.tail<3>() - previous_twist_.tail<3>();

    double max_delta_linear = max_linear_acceleration_ * twist_dt;
    double max_delta_angular = max_angular_acceleration_ * twist_dt;

    double delta_linear_norm = delta_linear.norm();
    double delta_angular_norm = delta_angular.norm();
    double linear_acceleration_scale = 1.0;
    double angular_acceleration_scale = 1.0;

    if (delta_linear_norm > max_delta_linear) {
      linear_acceleration_scale = max_delta_linear / delta_linear_norm;
      delta_linear *= linear_acceleration_scale;
    }

    if (delta_angular_norm > max_delta_angular) {
      angular_acceleration_scale = max_delta_angular / delta_angular_norm;
      delta_angular *= angular_acceleration_scale;
    }

    twist.head<3>() = previous_twist_.head<3>() + delta_linear;
    twist.tail<3>() = previous_twist_.tail<3>() + delta_angular;

    previous_twist_ = twist;
    previous_twist_time_ = current_twist_time;


    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(
      jacobian.rows(), jacobian.rows()
    );

    Eigen::MatrixXd lhs =
      jacobian * jacobian.transpose() +
      damping_ * damping_ * identity;
      
    Eigen::VectorXd qdot =
      jacobian.transpose() * lhs.ldlt().solve(twist);

    // ==================================================
    // Joint limit protection
    // ==================================================
    std::vector<double> current_positions;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      robot_state_->copyJointGroupPositions(
        joint_model_group_,
        current_positions
      );
    }

    double joint_limit_scale = 1.0;

    for (int i = 0; i < qdot.size(); ++i) {
      const auto & bounds =
        robot_model_->getVariableBounds(joint_names_[i]);

      if (!bounds.position_bounded_ || std::abs(qdot[i]) < 1e-4) {
        continue;
      }

      double distance_to_limit = 0.0;

      if (qdot[i] > 0.0) {
        distance_to_limit =
          bounds.max_position_ - current_positions[i];
      } else {
        distance_to_limit =
          current_positions[i] - bounds.min_position_;
      }

      double scale = 1.0;

      if (distance_to_limit <= joint_limit_stop_margin_) {
        scale = 0.0;
      } else if (distance_to_limit < joint_limit_slow_margin_) {
        scale =
          (distance_to_limit - joint_limit_stop_margin_) /
          (joint_limit_slow_margin_ - joint_limit_stop_margin_);
      }

      joint_limit_scale = std::min(joint_limit_scale, scale);
      if (scale < 1.0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 500,
          "%s near limit: q=%.4f qdot=%.4f "
          "limits=[%.4f, %.4f] dist=%.4f scale=%.3f",
          joint_names_[i].c_str(),
          current_positions[i],
          qdot[i],
          bounds.min_position_,
          bounds.max_position_,
          distance_to_limit,
          scale
        );
      }
    }

    if (joint_limit_scale < 1.0) {
      qdot *= joint_limit_scale;

      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 500,
        "Joint limit protection active: scale=%.3f",
        joint_limit_scale
      );
    }


    // ==================================================
    // Global joint velocity scaling
    // Preserve qdot direction while respecting max velocity
    // ==================================================
    double max_abs_qdot = qdot.cwiseAbs().maxCoeff();
    double velocity_scale = 1.0;

    if (max_abs_qdot > max_joint_velocity_) {
      velocity_scale = max_joint_velocity_ / max_abs_qdot;
      qdot *= velocity_scale;

      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 500,
        "Joint velocity scaling active: max=%.4f rad/s scale=%.3f",
        max_abs_qdot, velocity_scale
      );
    }


    // ==================================================
    // Cartesian velocity generated by final qdot
    // ==================================================
    Eigen::VectorXd achieved_twist = jacobian * qdot;
    Eigen::VectorXd actual_twist;

    bool have_actual_twist = false;
    double actual_qdot_norm = std::nan("");

    {
      std::lock_guard<std::mutex> lock(state_mutex_);

      if (have_actual_qdot_ &&
          actual_qdot_.size() == jacobian.cols())
      {
        actual_twist = jacobian * actual_qdot_;
        actual_qdot_norm = actual_qdot_.norm();
        have_actual_twist = true;
      }
    }
    
    if (have_actual_twist) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        500,
        "CMD=[%.6f %.6f %.6f | %.6f %.6f %.6f] "
        "J*qdot_cmd=[%.6f %.6f %.6f | %.6f %.6f %.6f] "
        "J*qdot_actual=[%.6f %.6f %.6f | %.6f %.6f %.6f]",
        twist[0], twist[1], twist[2],
        twist[3], twist[4], twist[5],
        achieved_twist[0], achieved_twist[1], achieved_twist[2],
        achieved_twist[3], achieved_twist[4], achieved_twist[5],
        actual_twist[0], actual_twist[1], actual_twist[2],
        actual_twist[3], actual_twist[4], actual_twist[5]
      );
    }
    else {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        500,
        "CMD=[%.6f %.6f %.6f | %.6f %.6f %.6f] "
        "J*qdot_cmd=[%.6f %.6f %.6f | %.6f %.6f %.6f] "
        "actual qdot waiting...",
        twist[0], twist[1], twist[2],
        twist[3], twist[4], twist[5],
        achieved_twist[0], achieved_twist[1], achieved_twist[2],
        achieved_twist[3], achieved_twist[4], achieved_twist[5]
      );
    }
    if (have_actual_qdot_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        500,
        "qdot_cmd=[%.6f %.6f %.6f %.6f %.6f %.6f] "
        "qdot_actual=[%.6f %.6f %.6f %.6f %.6f %.6f]",
        qdot[0], qdot[1], qdot[2],
        qdot[3], qdot[4], qdot[5],
        actual_qdot_[0], actual_qdot_[1], actual_qdot_[2],
        actual_qdot_[3], actual_qdot_[4], actual_qdot_[5]
      );
    }


    // ==================================================
    // Publish JointJog
    // ==================================================
    control_msgs::msg::JointJog output;

    output.header.stamp =
      now();

    output.header.frame_id =
      base_frame_;

    output.joint_names =
      joint_names_;

    output.velocities.resize(
      qdot.size());


    for (int i = 0;
         i < qdot.size();
         ++i)
    {
      output.velocities[i] =
        qdot[i];
    }


    output.duration = 0.0;

    joint_jog_pub_->publish(
      output);

    // Publish synchronized DLS diagnostics. This is observation only and
    // does not alter the command computed above.
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = output.header.stamp;

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "yam_follower/dls";
    status.hardware_id = "yam_follower";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "normal";

    if (joint_limit_scale < 1.0 || velocity_scale < 1.0 ||
        linear_velocity_scale < 1.0 || angular_velocity_scale < 1.0 ||
        linear_acceleration_scale < 1.0 || angular_acceleration_scale < 1.0)
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "command scaling active";
    }

    const auto add_value = [&status](const std::string & key, double value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = std::to_string(value);
      status.values.push_back(item);
    };

    add_value("sigma_min", sigma_min);
    add_value("sigma_max", sigma_max);
    add_value("condition_number", condition_number);
    add_value("damping", damping);
    add_value("joint_limit_scale", joint_limit_scale);
    add_value("joint_velocity_scale", velocity_scale);
    add_value("linear_velocity_scale", linear_velocity_scale);
    add_value("angular_velocity_scale", angular_velocity_scale);
    add_value("linear_acceleration_scale", linear_acceleration_scale);
    add_value("angular_acceleration_scale", angular_acceleration_scale);
    add_value("commanded_twist_linear_norm", twist.head<3>().norm());
    add_value("commanded_twist_angular_norm", twist.tail<3>().norm());
    add_value("achieved_twist_linear_norm", achieved_twist.head<3>().norm());
    add_value("achieved_twist_angular_norm", achieved_twist.tail<3>().norm());
    add_value("qdot_norm", qdot.norm());
    add_value("qdot_max_abs", qdot.cwiseAbs().maxCoeff());
    add_value(
      "actual_qdot_norm",
      actual_qdot_norm);

    diagnostics.status.push_back(status);
    diagnostics_pub_->publish(diagnostics);


    // ==================================================
    // qdot debug
    // ==================================================
    if (qdot.size() >= 6)
    {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        500,
        "qdot=[%.6f %.6f %.6f "
        "%.6f %.6f %.6f]",
        qdot[0],
        qdot[1],
        qdot[2],
        qdot[3],
        qdot[4],
        qdot[5]);
    }
  }


  // ==================================================
  // MoveIt
  // ==================================================
  std::shared_ptr<
    robot_model_loader::RobotModelLoader>
    robot_model_loader_;

  moveit::core::RobotModelPtr
    robot_model_;

  moveit::core::RobotStatePtr
    robot_state_;

  const moveit::core::JointModelGroup *
    joint_model_group_{nullptr};

  const moveit::core::LinkModel *
    ee_link_model_{nullptr};


  std::mutex state_mutex_;
  
  std::vector<double> prev_joint_positions_;
  rclcpp::Time prev_joint_time_;

  Eigen::VectorXd actual_qdot_;

  bool have_prev_joint_state_{false};
  bool have_actual_qdot_{false};

  double qdot_window_ = 0.10;

  bool have_model_{false};
  bool have_joint_state_{false};


  // ==================================================
  // ROS
  // ==================================================
  rclcpp::Subscription<
    sensor_msgs::msg::JointState>::SharedPtr
    joint_state_sub_;

  rclcpp::Subscription<
    geometry_msgs::msg::TwistStamped>::SharedPtr
    twist_sub_;

  rclcpp::Publisher<
    control_msgs::msg::JointJog>::SharedPtr
    joint_jog_pub_;

  rclcpp::Publisher<
    diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_pub_;


  // ==================================================
  // Parameters
  // ==================================================
  std::string planning_group_;
  std::string base_frame_;
  std::string ee_link_;

  double damping_;
  double max_joint_velocity_;
  double max_linear_velocity_;
  double max_angular_velocity_;
  double max_linear_acceleration_;
  double max_angular_acceleration_;

  double joint_limit_slow_margin_;
  double joint_limit_stop_margin_;

  Eigen::Matrix<double, 6, 1> previous_twist_ =
    Eigen::Matrix<double, 6, 1>::Zero();

  rclcpp::Time previous_twist_time_{0, 0, RCL_ROS_TIME};

  bool have_previous_twist_{false};
  
  std::vector<std::string>
    joint_names_;
};


// ====================================================
// Main
// ====================================================
int main(
  int argc,
  char ** argv)
{
  rclcpp::init(
    argc,
    argv);

  auto node =
    std::make_shared<TwistToJointJog>();

  node->initialize(
    node);

  rclcpp::spin(
    node);

  rclcpp::shutdown();

  return 0;
}
