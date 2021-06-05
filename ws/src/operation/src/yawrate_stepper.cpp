#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_interface/msg/yaw_reference.hpp>

using vehicle_interface::msg::YawReference;

class YawrateStepper : public rclcpp::Node {
 public:
  explicit YawrateStepper(const rclcpp::NodeOptions& options)
      : Node("yawrate_stepper", options) {
    this->declare_parameter("steptime");
    this->get_parameter("steptime", steptime);

    this->declare_parameter("stepvalue");
    this->get_parameter("stepvalue", stepvalue);

    timer_p = this->create_wall_timer(std::chrono::duration<double>(0.1),
                                      std::bind(&YawrateStepper::update, this));

    starttime = this->get_clock()->now();

    yaw_reference.source = YawReference::YAWRATE;

    pub = this->create_publisher<YawReference>("yaw_reference", 1);
  }

 private:
  rclcpp::Time starttime;
  double steptime;
  double stepvalue;

  YawReference yaw_reference;

  rclcpp::Publisher<YawReference>::SharedPtr pub;

  rclcpp::TimerBase::SharedPtr timer_p;

  void update() {
    const auto time = this->get_clock()->now();

    if ((time - starttime).seconds() > steptime) {
      yaw_reference.yawrate = stepvalue;
    }

    pub->publish(yaw_reference);
  }
};

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(YawrateStepper)
