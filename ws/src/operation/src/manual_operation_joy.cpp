#include <rclcpp/rclcpp.hpp>
#include <memory>

#include <joy/joy.hpp>

class ManualOperationJoyNode : public rclcpp::Node
{
public:
    ManualOperationJoyNode()
    : Node("manual_operation_joy_node")
    {
        RCLCPP_INFO(this->get_logger(), "created joy relay node");
        

    }

};




int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<ManualOperationJoyNode>());
        
    rclcpp::shutdown();

    return 0;
}




// joy_node publishes on /joy
// sensor_msgs::msg::Joy

// need to convert this to an appropriate reference signal for the vehicle
// for instance geometry_msgs::msg::Twist
