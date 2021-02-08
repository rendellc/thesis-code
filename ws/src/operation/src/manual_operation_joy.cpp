// joy_node publishes on /joy
// sensor_msgs::msg::Joy

// need to convert this to an appropriate reference signal for the vehicle
// for instance geometry_msgs::msg::Twist



#include <rclcpp/rclcpp.hpp>
#include <memory>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

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
        
        reference_pub_p = this->create_publisher<geometry_msgs::msg::Twist>(
            "reference", 1
        );
        
        timer_p = this->create_wall_timer(
                std::chrono::duration<double>(1.0/30.0),
                std::bind(&ManualOperationJoyNode::update, this)
        );
    }

private:
    // Joy message indices
    enum JOY : size_t {
        LEFT_LR = 0,
        LEFT_UD = 1,
        RIGHT_LR = 3,
        RIGHT_UD = 4
    };
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_p;
    sensor_msgs::msg::Joy::SharedPtr joy_p;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr reference_pub_p;
    geometry_msgs::msg::Twist reference;
    
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
            // const std::string message = 
            //     "LEFT(" + to_string(joy_p->axes[LEFT_LR]) + "," + to_string(joy_p->axes[LEFT_UD]) + "), " + 
            //     "RIGHT(" + to_string(joy_p->axes[RIGHT_LR]) + "," + to_string(joy_p->axes[RIGHT_UD]) + ")";
            // RCLCPP_INFO(this->get_logger(), message);
            reference.linear.x = joy_p->axes[LEFT_UD]* 15.0/3.6;
            reference.angular.z = joy_p->axes[RIGHT_LR];
            
            reference_pub_p->publish(reference);
        }
    }
};


#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ManualOperationJoyNode)
