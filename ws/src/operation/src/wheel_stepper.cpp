#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_interface/msg/wheel_state.hpp>

using vehicle_interface::msg::WheelState;

class WheelStepper : public rclcpp::Node {
 public:
  explicit WheelStepper(const rclcpp::NodeOptions& options)
      : Node("wheel_stepper", options) {
    this->declare_parameter("steptime");
    this->get_parameter("steptime", steptime);

    this->declare_parameter("stepvalue");
    this->get_parameter("stepvalue", stepvalue);

    timer_p = this->create_wall_timer(std::chrono::duration<double>(0.1),
                                      std::bind(&WheelStepper::update, this));

    starttime = this->get_clock()->now();

    wheelstate.steering_angle = 0.0;

    fl_pub = this->create_publisher<WheelState>("wheel_fl/reference", 1);
    rl_pub = this->create_publisher<WheelState>("wheel_rl/reference", 1);
    rr_pub = this->create_publisher<WheelState>("wheel_rr/reference", 1);
    fr_pub = this->create_publisher<WheelState>("wheel_fr/reference", 1);
  }

 private:
  rclcpp::Time starttime;
  double steptime;
  double stepvalue;

  WheelState wheelstate;

  rclcpp::Publisher<WheelState>::SharedPtr fl_pub, rl_pub, rr_pub, fr_pub;

  rclcpp::TimerBase::SharedPtr timer_p;

  void update() {
    const auto time = this->get_clock()->now();

    if ((time - starttime).seconds() > steptime) {
      wheelstate.steering_angle = stepvalue;
    }

    fl_pub->publish(wheelstate);
    rl_pub->publish(wheelstate);
    rr_pub->publish(wheelstate);
    fr_pub->publish(wheelstate);
  }
};

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(WheelStepper)
