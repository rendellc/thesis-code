#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_interface/msg/wheel_state.hpp>

using vehicle_interface::msg::WheelState;

class WheelTurner : public rclcpp::Node {
 public:
  explicit WheelTurner(const rclcpp::NodeOptions& options)
      : Node("wheel_turner", options) {
    this->declare_parameter("slopetimeshift");
    this->get_parameter("slopetimeshift", slopetimeshift);

    this->declare_parameter("slope");
    this->get_parameter("slope", slope);

    timer_p = this->create_wall_timer(std::chrono::duration<double>(0.1),
                                      std::bind(&WheelTurner::update, this));

    starttime = this->get_clock()->now();

    wheelstate.steering_angle = 0.0;

    fl_pub = this->create_publisher<WheelState>("wheel_fl/reference", 1);
    rl_pub = this->create_publisher<WheelState>("wheel_rl/reference", 1);
    rr_pub = this->create_publisher<WheelState>("wheel_rr/reference", 1);
    fr_pub = this->create_publisher<WheelState>("wheel_fr/reference", 1);
  }

 private:
  rclcpp::Time starttime;
  double slopetimeshift;
  double slope;

  WheelState wheelstate;

  rclcpp::Publisher<WheelState>::SharedPtr fl_pub, rl_pub, rr_pub, fr_pub;

  rclcpp::TimerBase::SharedPtr timer_p;

  void update() {
    const auto time = this->get_clock()->now();
    const auto t = (time - starttime).seconds();

    if (t > slopetimeshift) {
      wheelstate.steering_angle = slope * (t - slopetimeshift);
      wheelstate.steering_angle_rate = slope;
    }

    fl_pub->publish(wheelstate);
    rl_pub->publish(wheelstate);
    rr_pub->publish(wheelstate);
    fr_pub->publish(wheelstate);
  }
};

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(WheelTurner)
