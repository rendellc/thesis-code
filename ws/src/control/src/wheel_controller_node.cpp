
#include <chrono>
#include <cmath>
#include <control/PID.hpp>
#include <control/reference_model.hpp>
#include <control/softsign.hpp>
#include <control/ssa.hpp>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_interface/msg/wheel_command.hpp>
#include <vehicle_interface/msg/wheel_controller_info.hpp>
#include <vehicle_interface/msg/wheel_load.hpp>
#include <vehicle_interface/msg/wheel_state.hpp>

using control::softsign;
using control::ssa;
using Eigen::Matrix2d;
using Eigen::Vector2d;
using std::placeholders::_1;
using namespace vehicle_interface::msg;
using control::reference_model::FirstOrder;
using control::reference_model::SecondOrder;

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

    this->declare_parameter("D_omega");
    this->get_parameter("D_omega", angular_velocity_pid.D);

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

    this->declare_parameter("use_reference_optimization");
    this->get_parameter("use_reference_optimization",
                        use_reference_optimization);

    this->declare_parameter("use_reference_model");
    this->get_parameter("use_reference_model", use_reference_model);

    double steer_damping_ratio;
    this->declare_parameter("steer_damping_ratio");
    this->get_parameter("steer_damping_ratio", steer_damping_ratio);

    double steer_natural_frequency, drive_time_constant;
    this->declare_parameter("steer_natural_frequency");
    this->get_parameter("steer_natural_frequency", steer_natural_frequency);
    this->declare_parameter("drive_time_constant");
    this->get_parameter("drive_time_constant", drive_time_constant);

    steer_model = std::make_shared<SecondOrder>(steer_damping_ratio,
                                                steer_natural_frequency, 0, 0);
    drive_model = std::make_shared<FirstOrder>(drive_time_constant, 0);

    RCLCPP_INFO(this->get_logger(), "%s initialized", this->get_name());
  }

 private:
  rclcpp::Publisher<WheelCommand>::SharedPtr command_pub_p;
  rclcpp::Publisher<WheelControllerInfo>::SharedPtr info_pub_p;
  rclcpp::Subscription<WheelState>::SharedPtr state_sub_p, reference_sub_p;
  rclcpp::Subscription<WheelLoad>::SharedPtr wheel_load_sub_p;
  rclcpp::TimerBase::SharedPtr timer_p;

  // Reference models
  std::shared_ptr<FirstOrder> drive_model;
  std::shared_ptr<SecondOrder> steer_model;

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
  bool use_reference_optimization;
  bool use_reference_model;

  bool state_valid = false;
  bool reference_valid = false;
  bool load_valid = false;
  // WheelState::SharedPtr wheel_state_p;
  // WheelState::SharedPtr wheel_reference_p;
  // WheelLoad::SharedPtr wheel_load_p;
  // WheelState reference_optimal;
  // WheelState reference;

  PID angular_velocity_pid;
  PID steering_rate_pid;

  WheelControllerInfo info;

  void sliding_mode_steering_angle() {
    // Steering angle control law
    // calculation in goodnotes
    // const auto x1 = ssa(x_p->steering_angle - xr_p->steering_angle);
    const auto x1 = info.state.steering_angle - info.reference.steering_angle;
    x1_integral = x1_integral + x1 / update_rate;
    const auto x2 =
        info.state.steering_angle_rate - info.reference.steering_angle_rate;

    const double steer_inertia =
        wheel_mass *
        (3 * wheel_radius * wheel_radius + wheel_width * wheel_width) / 12;
    // const double max_steer_resistance = 15000.0; // found by experimenting
    // in gazebo, should actually be different for each wheel and load
    // dependent
    // const double steer_resistance_factor = 2.5;
    const double &Fz = info.load.load;
    const double max_steer_resistance = steer_resistance_factor * Fz;
    // const double beta_0 = 0.1;
    info.sliding_mode_rho = steer_inertia * sliding_mode_eigenvalue * fabs(x2) +
                            max_steer_resistance;
    const double &rho = info.sliding_mode_rho;
    info.sliding_mode_beta = rho + beta_0;
    // const double s = x1 + (1 / sliding_mode_eigenvalue) * x2;
    info.sliding_mode_s = sliding_mode_eigenvalue * x1 + x2;

    const double &s = info.sliding_mode_s;
    const double &beta = info.sliding_mode_beta;
    info.cmd.steer_torque = -beta * softsign(s, sliding_mode_softregion);
  }

  void robust_rate() {
    info.mode = "robust_rate";
    // const auto &xr = wheel_reference_p;
    const auto &Fz = info.load.load;

    // Steering angle rate control law
    // calculation in goodnotes
    const double steer_inertia =
        wheel_mass *
        (3 * wheel_radius * wheel_radius + wheel_width * wheel_width) / 12;

    info.robust_rate_error_rate = info.robust_rate_angular_rate_reference -
                                  info.state.steering_angle_rate;
    const double max_steer_resistance = steer_resistance_factor * Fz;

    const double &rate_error = info.robust_rate_error_rate;
    info.cmd.steer_torque =
        (steer_inertia * max_steering_accel + max_steer_resistance + beta_0) *
        softsign(rate_error, robust_rate_softregion);
  }

  void reference_model() {
    // Use opt to filter this->reference
    const auto dt = 1 / update_rate;
    // Steering angle model
    // deltaddot + 2*zeta*omega0* deltadot + omega0^2 delta = delta_r
    steer_model->update(info.reference_optimal.steering_angle, dt);
    drive_model->update(info.reference_optimal.angular_velocity, dt);

    info.reference.steering_angle = steer_model->getReference();
    info.reference.steering_angle_rate = steer_model->getReferenceDerivative();
    info.reference.angular_velocity = drive_model->getReference();
  }

  void update_command() {
    if (state_valid && reference_valid && load_valid) {
      // Angular velocity control law
      const auto time_now = this->now();

      if (use_reference_optimization) {
        info.reference_optimal =
            find_closest_equivalent_reference(info.state, info.reference_raw);
      } else {
        info.reference_optimal = info.reference_raw;
      }

      if (use_reference_model) {
        reference_model();
      } else {
        info.reference = info.reference_optimal;
      }

      const auto omega_error =
          info.reference.angular_velocity - info.state.angular_velocity;
      info.cmd.drive_torque =
          angular_velocity_pid.update(omega_error, time_now);

      if (use_sliding_mode) {
        sliding_mode_steering_angle();
        info.mode = "sliding_mode";
      }
      if (use_robust_rate) {
        const double delta = info.state.steering_angle;
        const double delta_r = info.reference.steering_angle;
        const double ddelta_r = info.reference.steering_angle_rate;

        // compute desired rate
        double desired_angular_rate =
            ddelta_r + steering_rate_pid.update((delta_r - delta), time_now);
        if (fabs(desired_angular_rate) >= steering_rate_limit) {
          desired_angular_rate = desired_angular_rate /
                                 fabs(desired_angular_rate) *
                                 steering_rate_limit;
        }
        info.robust_rate_angular_rate_reference = desired_angular_rate;
        robust_rate();
      }

      info.header.frame_id = this->get_namespace();
      info.header.stamp = this->get_clock()->now();
      const auto &x = info.state;
      const auto &xr = info.reference;
      auto &e = info.error;
      e.steering_angle = xr.steering_angle - x.steering_angle;
      e.steering_angle_rate = xr.steering_angle_rate - x.steering_angle_rate;
      e.angular_velocity = xr.angular_velocity - x.angular_velocity;
      info_pub_p->publish(info);

      command_pub_p->publish(info.cmd);
    }
  }

  double reference_cost(const WheelState &state,
                        const WheelState &reference) const {
    const double steer_cost =
        fabs(state.steering_angle - reference.steering_angle);
    const double drive_cost =
        fabs(state.angular_velocity - reference.angular_velocity);

    return steer_cost + 0.2 * drive_cost;
  }

  WheelState find_closest_equivalent_reference(
      const WheelState &state, const WheelState &reference) const {
    // std::vector<WheelState> options;
    const double PI = 2 * atan2(1.0, 0);

    const double angle_change =
        ssa(reference.steering_angle - state.steering_angle);

    WheelState unchanged;
    unchanged.steering_angle = state.steering_angle + angle_change;
    unchanged.steering_angle_rate = reference.steering_angle_rate;
    unchanged.angular_velocity = reference.angular_velocity;

    // WheelState fullturn_positive = unchanged;
    // fullturn_positive.steering_angle = unchanged.steering_angle + 2 * PI;
    // WheelState fullturn_negative = unchanged;
    // fullturn_negative.steering_angle = unchanged.steering_angle - 2 * PI;

    WheelState halfturn_positive = unchanged;
    halfturn_positive.steering_angle = unchanged.steering_angle + PI;
    halfturn_positive.angular_velocity = -unchanged.angular_velocity;

    WheelState halfturn_negative = unchanged;
    halfturn_negative.steering_angle = unchanged.steering_angle - PI;
    halfturn_negative.angular_velocity = -unchanged.angular_velocity;

    std::array<WheelState, 3> options = {
        // fullturn_positive, fullturn_negative,
        unchanged, halfturn_positive, halfturn_negative};

    double lowest_cost = std::numeric_limits<double>::infinity();
    size_t lowest_index = 0;

    // [[sign-compare]]
    for (size_t i = 0; i < options.size(); i++) {
      const double cost = reference_cost(state, options[i]);
      if (cost < lowest_cost) {
        lowest_cost = cost;
        lowest_index = i;
      }
    }
    return options[lowest_index];
  }

  void state_callback(WheelState::SharedPtr msg_p) {
    state_valid = true;
    info.state = *msg_p;
    RCLCPP_INFO_ONCE(this->get_logger(), "state message recieved");
  }
  void reference_callback(WheelState::SharedPtr msg_p) {
    reference_valid = true;
    info.reference_raw = *msg_p;
    RCLCPP_INFO_ONCE(this->get_logger(), "reference message recieved");
  }
  void load_callback(WheelLoad::SharedPtr msg_p) {
    load_valid = true;
    info.load = *msg_p;
    RCLCPP_INFO_ONCE(this->get_logger(), "load message recieved");
  }
};

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(WheelControllerNode)
