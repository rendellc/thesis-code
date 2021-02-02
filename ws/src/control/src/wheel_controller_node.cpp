
#include <rclcpp/rclcpp.hpp>

#include "vehicle_interface/msg/wheel_command.hpp"
#include "vehicle_interface/msg/wheel_state.hpp"

#include <chrono>

using std::placeholders::_1;

class WheelControllerNode : public rclcpp::Node
{
public:
    rclcpp::Publisher<vehicle_interface::msg::WheelCommand>::SharedPtr command_pub_p;
    rclcpp::Subscription<vehicle_interface::msg::WheelState>::SharedPtr state_sub_p, reference_sub_p;
    rclcpp::TimerBase::SharedPtr timer_p;

    WheelControllerNode() 
    : Node("wheel_controller_node") 
    {
        // const rclcpp::Qos& qos = this->get_qos();
        command_pub_p = this->create_publisher<vehicle_interface::msg::WheelCommand>("command", 10);
        state_sub_p = this->create_subscription<vehicle_interface::msg::WheelState>(
            "state", 1, std::bind(&WheelControllerNode::state_callback, this, _1)
        );
        reference_sub_p = this->create_subscription<vehicle_interface::msg::WheelState>(
            "reference", 1, std::bind(&WheelControllerNode::reference_callback, this, _1)
        );
        
        this->declare_parameter<double>("update_rate", 60);
        this->get_parameter("update_rate", update_rate);
        timer_p = this->create_wall_timer(
                std::chrono::duration<double>(1/update_rate),
                std::bind(&WheelControllerNode::update_command, this)
        );
        
         this->declare_parameter<double>("P_omega", 100);
         this->declare_parameter<double>("P_delta", 100);
         this->get_parameter("P_omega", P_omega);
         this->get_parameter("P_delta", P_delta);
        
        RCLCPP_INFO(this->get_logger(), "wheel_controller_node initialized");
    }
    
private:
    vehicle_interface::msg::WheelState::SharedPtr wheel_state_p;
    vehicle_interface::msg::WheelState::SharedPtr wheel_reference_p;
    
    vehicle_interface::msg::WheelCommand command_msg;
    double update_rate; 
    double P_omega; 
    double P_delta;
    
    void update_command() 
    {
        if (wheel_state_p && wheel_reference_p) {
            const double omega = wheel_state_p->angular_velocity;
            const double omega_ref = wheel_reference_p->angular_velocity;
            const double delta = wheel_state_p->steering_angle;
            const double delta_ref = wheel_reference_p->steering_angle;
            command_msg.drive_torque = P_omega*(omega_ref - omega);
            command_msg.steer_torque = P_delta*(delta_ref - delta);

            command_pub_p->publish(command_msg);
        }
    }
    
    void state_callback(vehicle_interface::msg::WheelState::SharedPtr msg_p) 
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "state message recieved");
        wheel_state_p = msg_p;
    }
    void reference_callback(vehicle_interface::msg::WheelState::SharedPtr msg_p) 
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "reference message recieved");
        wheel_reference_p = msg_p;
    }
};

class TestNode : public rclcpp::Node
{
public:
    TestNode(const std::string& name) : Node(name) {}
};

int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelControllerNode>());
    rclcpp::shutdown();
    return 0;
}