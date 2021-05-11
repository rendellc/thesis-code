
#include <chrono>
#include <cmath>
#include <control/PID.hpp>
#include <control/softsign.hpp>
#include <control/ssa.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_interface/msg/wheel_command.hpp>
#include <vehicle_interface/msg/wheel_controller_info.hpp>
#include <vehicle_interface/msg/wheel_load.hpp>
#include <vehicle_interface/msg/wheel_state.hpp>

using control::softsign;
using control::ssa;
using std::placeholders::_1;
using namespace vehicle_interface::msg;

class WheelControllerNode : public rclcpp::Node {
 public:
  WheelControllerNode(const rclcpp::NodeOptions &options)
      : Node("wheel_controller_node", options) {
    command_pub_p = this->create_publisher<WheelCommand>("command", 10);
    state_sub_p = this->create_subscription<WheelState>(
        "state", 1, std::bind(&WheelControllerNode::state_callback, this, _1));
    reference_sub_p = this->create_subscription<WheelState>(
        "reference", 1,
        std::bind(&WheelControllerNode::reference_callback, this, _1));
    wheel_load_sub_p = this->create_subscription<WheelLoad>(
        "load", 1, std::bind(&WheelControllerNode::load_callback, this, _1));

    info_pub_p =
        this->create_publisher<WheelControllerInfo>("controller_info", 10);

    this->declare_parameter("update_rate");
    this->get_parameter("update_rate", update_rate);
    timer_p = this->create_wall_timer(
        std::chrono::duration<double>(1 / update_rate),
        std::bind(&WheelControllerNode::update_command, this));

    this->declare_parameter("P_omega");
    this->get_parameter("P_omega", angular_velocity_pid.P);

    this->declare_parameter("I_omega");
    this->get_parameter("I_omega", angular_velocity_pid.I);

    this->declare_parameter("P_delta");
    this->get_parameter("P_delta", steering_rate_pid.P);
    this->declare_parameter("steering_rate_limit");
    this->get_parameter("steering_rate_limit", steering_rate_limit);

    this->declare_parameter("wheel_mass");
    this->get_parameter("wheel_mass", wheel_mass);

    this->declare_parameter("wheel_radius");
    this->get_parameter("wheel_radius", wheel_radius);

    this->declare_parameter("wheel_width");
    this->get_parameter("wheel_width", wheel_width);

    this->declare_parameter("use_sliding_mode");
    this->get_parameter("use_sliding_mode", use_sliding_mode);

    this->declare_parameter("sliding_mode_eigenvalue");
    this->get_parameter("sliding_mode_eigenvalue", sliding_mode_eigenvalue);

    this->declare_parameter("steer_resistance_factor");
    this->get_parameter("steer_resistance_factor", steer_resistance_factor);

    this->declare_parameter("beta_0");
    this->get_parameter("beta_0", beta_0);

    this->declare_parameter("sliding_mode_softregion");
    this->get_parameter("sliding_mode_softregion", sliding_mode_softregion);

    this->declare_parameter("use_robust_rate");
    this->get_parameter("use_robust_rate", use_robust_rate);

    this->declare_parameter("max_steering_accel");
    this->get_parameter("max_steering_accel", max_steering_accel);

    this->declare_parameter("robust_rate_softregion");
    this->get_parameter("robust_rate_softregion", robust_rate_softregion);

    RCLCPP_INFO(this->get_logger(), "%s initialized", this->get_name());
  }

 private:
  rclcpp::Publisher<WheelCommand>::SharedPtr command_pub_p;
  rclcpp::Publisher<WheelControllerInfo>::SharedPtr info_pub_p;
  rclcpp::Subscription<WheelState>::SharedPtr state_sub_p, reference_sub_p;
  rclcpp::Subscription<WheelLoad>::SharedPtr wheel_load_sub_p;
  rclcpp::TimerBase::SharedPtr timer_p;

  double update_rate;
  double wheel_mass;
  double wheel_radius;
  double wheel_width;
  bool use_sliding_mode;
  double sliding_mode_eigenvalue;
  double steer_resistance_factor;
  double beta_0;
  double sliding_mode_softregion;
  double x1_integral = 0.0;
  double steering_rate_limit;
  bool use_robust_rate;
  double robust_rate_softregion;
  double max_steering_accel;

  WheelState::SharedPtr wheel_state_p;
  WheelState::SharedPtr wheel_reference_p;
  WheelLoad::SharedPtr wheel_load_p;

  WheelCommand command_msg;
  PID angular_velocity_pid;
  PID steering_rate_pid;

  WheelControllerInfo info_msg;

  void sliding_mode_steering_angle() {
    const auto &x_p = wheel_state_p;
    const auto &xr_p = wheel_reference_p;
    const auto &F_z = wheel_load_p->load;

    // Steering angle control law
    // calculation in goodnotes
    const auto x1 = ssa(x_p->steering_angle - xr_p->steering_angle);
    x1_integral = x1_integral + x1 / update_rate;
    const auto x2 = x_p->steering_angle_rate - xr_p->steering_angle_rate;

    const double steer_inertia =
        wheel_mass *
        (3 * wheel_radius * wheel_radius + wheel_width * wheel_width) / 12;
    // const double max_steer_resistance = 15000.0; // found by experimenting
    // in gazebo, should actually be different for each wheel and load
    // dependent
    // const double steer_resistance_factor = 2.5;
    const double max_steer_resistance = steer_resistance_factor * F_z;
    // const double beta_0 = 0.1;
    const double rho = steer_inertia * sliding_mode_eigenvalue * fabs(x2) +
                       max_steer_resistance;
    const double beta = rho + beta_0;
    // const double s = x1 + (1 / sliding_mode_eigenvalue) * x2;
    const double s = sliding_mode_eigenvalue * x1 + x2;

    info_msg.sliding_mode_s = s;
    info_msg.sliding_mode_rho = rho;
    info_msg.sliding_mode_beta = beta;

    command_msg.steer_torque = -beta * softsign(s, sliding_mode_softregion);
  }

  void robust_rate(double desired_rate) {
    const auto &x = wheel_state_p;
    // const auto &xr = wheel_reference_p;
    const auto &F_z = wheel_load_p->load;

    // Steering angle rate control law
    // calculation in goodnotes
    const double steer_inertia =
        wheel_mass *
        (3 * wheel_radius * wheel_radius + wheel_width * wheel_width) / 12;

    const double rate_error = desired_rate - x->steering_angle_rate;
    const double max_steer_resistance = steer_resistance_factor * F_z;

    info_msg.robust_rate_error_rate = rate_error;

    command_msg.steer_torque =
        (steer_inertia * max_steering_accel + max_steer_resistance + beta_0) *
        softsign(rate_error, robust_rate_softregion);
  }

  void update_command() {
    if (wheel_state_p && wheel_reference_p && wheel_load_p) {
      // Angular velocity control law
      const auto time_now = this->now();
      const auto omega_error =
          wheel_reference_p->angular_velocity - wheel_state_p->angular_velocity;
      command_msg.drive_torque =
          angular_velocity_pid.update(omega_error, time_now);

      if (use_sliding_mode) {
        sliding_mode_steering_angle();
        info_msg.mode = "sliding_mode";
      }
      if (use_robust_rate) {
        const double delta = wheel_state_p->steering_angle;
        const double delta_r = wheel_reference_p->steering_angle;

        // compute desired rate
        double desired_angular_rate =
            steering_rate_pid.update(ssa(delta_r - delta), time_now);
        if (fabs(desired_angular_rate) >= steering_rate_limit) {
          desired_angular_rate = desired_angular_rate /
                                 fabs(desired_angular_rate) *
                                 steering_rate_limit;
        }
        robust_rate(desired_angular_rate);

        info_msg.mode = "robust_rate";
        info_msg.robust_rate_angular_rate_reference = desired_angular_rate;
      }

      info_msg.header.frame_id = this->get_namespace();
      info_msg.header.stamp = this->get_clock()->now();
      info_msg.state = *wheel_state_p;
      info_msg.reference = *wheel_reference_p;
      info_msg.steer_torque = command_msg.steer_torque;
      info_msg.drive_torque = command_msg.drive_torque;
      command_pub_p->publish(command_msg);
      info_pub_p->publish(info_msg);
    }
  }

  void state_callback(WheelState::SharedPtr msg_p) {
    RCLCPP_INFO_ONCE(this->get_logger(), "state message recieved");
    wheel_state_p = msg_p;
  }
  void reference_callback(WheelState::SharedPtr msg_p) {
    RCLCPP_INFO_ONCE(this->get_logger(), "reference message recieved");
    wheel_reference_p = msg_p;
  }
  void load_callback(WheelLoad::SharedPtr msg_p) {
    RCLCPP_INFO_ONCE(this->get_logger(), "load message recieved");
    wheel_load_p = msg_p;
  }
};

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(WheelControllerNode)
