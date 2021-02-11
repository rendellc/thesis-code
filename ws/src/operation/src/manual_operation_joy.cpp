// joy_node publishes on /joy
// sensor_msgs::msg::Joy

// need to convert this to an appropriate reference signal for the vehicle
// for instance geometry_msgs::msg::Twist



#include <rclcpp/rclcpp.hpp>
#include <memory>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "vehicle_interface/msg/drive_mode.hpp"

using std::placeholders::_1;

class ManualOperationJoyNode : public rclcpp::Node
{
public:
    ManualOperationJoyNode(const rclcpp::NodeOptions& options)
    : Node("manual_operation_joy_node", options)
    {
        joy_sub_p = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 1, std::bind(&ManualOperationJoyNode::joy_callback, this, _1)
        );
        
        reference_pub_p = this->create_publisher<vehicle_interface::msg::DriveMode>(
            "reference", 1
        );
        
        timer_p = this->create_wall_timer(
                std::chrono::duration<double>(1.0/30.0),
                std::bind(&ManualOperationJoyNode::update, this)
        );
    }

private:
    // Joy message indices
    enum AXES : size_t {
        LEFT_LR = 0,
        LEFT_UD = 1,
        RIGHT_LR = 3,
        RIGHT_UD = 4,
        PAD_LR = 6
    };
    enum BUTTONS : size_t {
        TRIANGLE = 2
    };
    bool triangle_pressed = false;
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_p;
    sensor_msgs::msg::Joy::SharedPtr joy_p;
    
    //rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr reference_pub_p;
    rclcpp::Publisher<vehicle_interface::msg::DriveMode>::SharedPtr reference_pub_p;
    // geometry_msgs::msg::Twist reference;
    vehicle_interface::msg::DriveMode reference;
    
    rclcpp::TimerBase::SharedPtr timer_p;

    void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg_p)
    {
        joy_p = msg_p;
        RCLCPP_INFO_ONCE(this->get_logger(), "joy message recieved");
    }
    
    void update()
    {
        using std::to_string;
        if (joy_p)
        {
            if (joy_p->buttons[TRIANGLE] == 1)
            {
                triangle_pressed = true;
            }
            if (joy_p->buttons[TRIANGLE] == 0 && triangle_pressed)
            {
                // TODO: implement mode transition better
                reference.mode = (reference.mode + 1) % (vehicle_interface::msg::DriveMode::CRAB + 1);
                triangle_pressed = false;
            }

            reference.speed = joy_p->axes[LEFT_UD]* 15.0/3.6;
            reference.turn = 1.57*joy_p->axes[RIGHT_LR];
            
            reference_pub_p->publish(reference);
        }
    }
};


#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ManualOperationJoyNode)
