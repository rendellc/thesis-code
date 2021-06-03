#include <eigen3/Eigen/Dense>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_interface/msg/wheel_state.hpp>

using Eigen::Matrix2d;
using Eigen::Vector2d;
using vehicle_interface::msg::WheelState;

class WheelAngvelSmoothStepper : public rclcpp::Node {
 public:
  explicit WheelAngvelSmoothStepper(const rclcpp::NodeOptions& options)
      : Node("wheel_angvel_stepper", options) {
    this->declare_parameter("steptime");
    this->get_parameter("steptime", steptime);

    this->declare_parameter("stepvalue");
    this->get_parameter("stepvalue", stepvalue);

    this->declare_parameter("damping_ratio");
    this->get_parameter("damping_ratio", damping_ratio);

    this->declare_parameter("undamped_frequency");
    this->get_parameter("undamped_frequency", undamped_frequency);

    timer_p = this->create_wall_timer(
        std::chrono::duration<double>(0.01),
        std::bind(&WheelAngvelSmoothStepper::update, this));

    starttime = this->get_clock()->now();
    prevtime = starttime;

    angular_velocity_desired = 0.0;
    wheelstate.steering_angle = 0.0;
    wheelstate.steering_angle_rate = 0.0;
    wheelstate.angular_velocity = 0.0;

    A(0, 0) = 0;
    A(0, 1) = 1;

    A(1, 0) = -undamped_frequency * undamped_frequency;
    A(1, 1) = -2 * damping_ratio * undamped_frequency;
    b << 0, undamped_frequency * undamped_frequency;
    x << 0, 0;

    fl_pub = this->create_publisher<WheelState>("wheel_fl/reference", 1);
    rl_pub = this->create_publisher<WheelState>("wheel_rl/reference", 1);
    rr_pub = this->create_publisher<WheelState>("wheel_rr/reference", 1);
    fr_pub = this->create_publisher<WheelState>("wheel_fr/reference", 1);
  }

 private:
  rclcpp::Time starttime;
  rclcpp::Time prevtime;
  double steptime;
  double stepvalue;
  double damping_ratio;
  double undamped_frequency;
  Matrix2d A;
  Vector2d b;
  Vector2d x;

  double angular_velocity_desired;
  WheelState wheelstate;

  rclcpp::Publisher<WheelState>::SharedPtr fl_pub, rl_pub, rr_pub, fr_pub;
  rclcpp::TimerBase::SharedPtr timer_p;

  void update() {
    const auto time = this->get_clock()->now();
    const double dt = (time - prevtime).seconds();
    const double time_elapsed = (time - starttime).seconds();

    if (time_elapsed > steptime) {
      angular_velocity_desired = stepvalue;
    }

    const Matrix2d I = Matrix2d::Identity(2, 2);
    // x = (I + dt * A) * (I - dt * A).inverse() * x +
    //     dt * b * angular_velocity_desired;
    x = (I + dt * A) * x + dt * b * angular_velocity_desired;

    wheelstate.angular_velocity = x(0);
    prevtime = time;

    fl_pub->publish(wheelstate);
    rl_pub->publish(wheelstate);
    rr_pub->publish(wheelstate);
    fr_pub->publish(wheelstate);
  }
};

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(WheelAngvelSmoothStepper)
