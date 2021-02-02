
#include <rclcpp/rclcpp.hpp>

#include "simulator/msg/wheel_command.hpp"
#include "simulator/msg/wheel_state.hpp"

#include <chrono>

using std::placeholders::_1;

class WheelControllerNode : public rclcpp::Node
{
public:
    WheelControllerNode()
    : Node("wheel_controller_node") 
    {
        // const rclcpp::Qos& qos = this->get_qos();
        command_pub_p = this->create_publisher<simulator::msg::WheelCommand>("command", 10);
        state_sub_p = this->create_subscription<simulator::msg::WheelState>(
            "state", 10, std::bind(&WheelControllerNode::state_callback, this, _1)
        );
        
        const double pub_period = 0.1; // TODO: parameter
        timer_p = this->create_wall_timer(
                std::chrono::duration<double>(pub_period),
                std::bind(&WheelControllerNode::update_command, this)
        );
    }
    
private:
    rclcpp::Publisher<simulator::msg::WheelCommand>::SharedPtr command_pub_p;
    rclcpp::Subscription<simulator::msg::WheelState>::SharedPtr state_sub_p;
    rclcpp::TimerBase::SharedPtr timer_p;

    simulator::msg::WheelState::SharedPtr wheel_state_p;
    
    void update_command() {
        simulator::msg::WheelCommand command;

        // TODO: control implementation
        command.drive_torque = 0.0;
        command.steer_torque = 0.0;

        command_pub_p->publish(command);
    }
    
    void state_callback(simulator::msg::WheelState::SharedPtr msg_p) 
    {
        wheel_state_p = msg_p;
    }
};


int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelControllerNode>());
    rclcpp::shutdown();
    return 0;
}