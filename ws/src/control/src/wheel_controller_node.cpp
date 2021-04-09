
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include "control/PID.hpp"
#include "vehicle_interface/msg/wheel_command.hpp"
#include "vehicle_interface/msg/wheel_load.hpp"
#include "vehicle_interface/msg/wheel_state.hpp"

using std::placeholders::_1;

class WheelControllerNode : public rclcpp::Node {
 public:
  WheelControllerNode(const rclcpp::NodeOptions &options)
      : Node("wheel_controller_node", options) {
    command_pub_p =
        this->create_publisher<vehicle_interface::msg::WheelCommand>("command",
                                                                     10);
    state_sub_p = this->create_subscription<vehicle_interface::msg::WheelState>(
        "state", 1, std::bind(&WheelControllerNode::state_callback, this, _1));
    reference_sub_p =
        this->create_subscription<vehicle_interface::msg::WheelState>(
            "reference", 1,
            std::bind(&WheelControllerNode::reference_callback, this, _1));
    wheel_load_sub_p =
        this->create_subscription<vehicle_interface::msg::WheelLoad>(
            "load", 1,
            std::bind(&WheelControllerNode::load_callback, this, _1));

    this->declare_parameter<double>("update_rate", 100);
    this->get_parameter("update_rate", update_rate);
    timer_p = this->create_wall_timer(
        std::chrono::duration<double>(1 / update_rate),
        std::bind(&WheelControllerNode::update_command, this));

    this->declare_parameter<double>("P_omega", 100);
    this->declare_parameter<double>("P_delta", 100);

    this->get_parameter("P_omega", angular_velocity_pid.P);
    this->get_parameter("P_delta", steering_angle_pid.P);

    RCLCPP_INFO(this->get_logger(), "%s initialized", this->get_name());
  }

 private:
  rclcpp::Publisher<vehicle_interface::msg::WheelCommand>::SharedPtr
      command_pub_p;
  rclcpp::Subscription<vehicle_interface::msg::WheelState>::SharedPtr
      state_sub_p,
      reference_sub_p;
  rclcpp::Subscription<vehicle_interface::msg::WheelLoad>::SharedPtr
      wheel_load_sub_p;
  rclcpp::TimerBase::SharedPtr timer_p;

  vehicle_interface::msg::WheelState::SharedPtr wheel_state_p;
  vehicle_interface::msg::WheelState::SharedPtr wheel_reference_p;
  vehicle_interface::msg::WheelLoad::SharedPtr wheel_load_p;

  vehicle_interface::msg::WheelCommand command_msg;
  double update_rate;
  PID angular_velocity_pid;
  PID steering_angle_pid;

  void update_command() {
    if (wheel_state_p && wheel_reference_p && wheel_load_p) {
      const auto time_now = this->now();

      // Shorter names
      const auto &x_p = wheel_state_p;
      const auto &xr_p = wheel_reference_p;
      const auto &F_z = wheel_load_p->load;

      // Angular velocity control law
      const auto omega_error = xr_p->angular_velocity - x_p->angular_velocity;
      command_msg.drive_torque =
          angular_velocity_pid.update(omega_error, time_now);

      // Steering angle control law
      // calculation in goodnotes
      const std::function<double(double)> ssa = [](double dtheta) {
        return atan2(sin(dtheta), cos(dtheta));
      };

      const auto x1 = x_p->steering_angle - xr_p->steering_angle;
      const auto x2 = x_p->steering_angle_rate - xr_p->steering_angle_rate;

      const double wheel_mass = 200.0, wheel_radius = 0.505, wheel_width = 0.4;
      const double a1 = 0.5;
      const double steer_inertia =
          wheel_mass *
          (3 * wheel_radius * wheel_radius + wheel_width * wheel_width) / 12;
      // const double max_steer_resistance = 15000.0; // found by experimenting
      // in gazebo, should actually be different for each wheel and load
      // dependent
      const double max_steer_resistance = 2.5 * F_z;
      const double beta_0 = 0.1;
      const double rho = steer_inertia * a1 * fabs(x2) + max_steer_resistance;
      const double beta = rho + beta_0;
      const double s = a1 * x2 + x1;

      const auto sign = [](double x) {
        double eps = 2.5;
        if (x < -eps) {
          return -1.0;
        } else if (fabs(x) <= eps) {
          return x / eps;
        } else {
          return 1.0;
        }
      };

      command_msg.steer_torque = -beta * sign(s);

      command_pub_p->publish(command_msg);
    }
  }

  void state_callback(vehicle_interface::msg::WheelState::SharedPtr msg_p) {
    RCLCPP_INFO_ONCE(this->get_logger(), "state message recieved");
    wheel_state_p = msg_p;
  }
  void reference_callback(vehicle_interface::msg::WheelState::SharedPtr msg_p) {
    RCLCPP_INFO_ONCE(this->get_logger(), "reference message recieved");
    wheel_reference_p = msg_p;
  }
  void load_callback(vehicle_interface::msg::WheelLoad::SharedPtr msg_p) {
    RCLCPP_INFO_ONCE(this->get_logger(), "load message recieved");
    wheel_load_p = msg_p;
  }
};

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(WheelControllerNode)
