// joy_node publishes on /joy
// sensor_msgs::msg::Joy

// need to convert this to an appropriate reference signal for the vehicle
// for instance geometry_msgs::msg::Twist



#include <rclcpp/rclcpp.hpp>
#include <rcpputils/asserts.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "vehicle_interface/msg/drive_mode.hpp"

#include <memory>
#include <algorithm>

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
        
        reference.mode = vehicle_interface::msg::DriveMode::ACKERMANN;
        
        timer_p = this->create_wall_timer(
                std::chrono::duration<double>(1.0/update_rate),
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
        CIRCLE = 1,
        TRIANGLE = 2,
        NUM_BUTTONS = 13
    };
    
    std::array<bool,NUM_BUTTONS> button_pressed;
    std::array<bool,NUM_BUTTONS> button_released;
    
    double update_rate = 30.0;
    double turn_angle = 0.0;
    
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
        update_buttons(joy_p);
        RCLCPP_INFO_ONCE(this->get_logger(), "joy message recieved");

        // Store pressed buttons
        for (size_t i = 0; i < NUM_BUTTONS; i++)
        {
            if (joy_p->buttons[i] == 1)
            {
                button_pressed[i] = true;
            }
        }
        // std::copy_if(joy_p->buttons.cbegin(), joy_p->buttons.cend(), 
        //     button_pressed.begin(),
        //     [](int32_t button){ return button == 1; }
        // );

        // Check for released buttons
        for (size_t i = 0; i < NUM_BUTTONS; i++)
        {
            if (joy_p->buttons[i] == 0 && button_pressed[i])
            {
                button_released[i] = true;
                button_pressed[i] = false;
            }
        }

    }
    
    std::string next_drive_mode(const std::string& mode)
    {
        if (mode == vehicle_interface::msg::DriveMode::ACKERMANN)
        {
            return vehicle_interface::msg::DriveMode::AFAR;
        } else if (mode == vehicle_interface::msg::DriveMode::AFAR)
        {
            return vehicle_interface::msg::DriveMode::CRAB;
        } else if (mode == vehicle_interface::msg::DriveMode::CRAB)
        {
            return vehicle_interface::msg::DriveMode::SPIN;
        } else 
        {
            return vehicle_interface::msg::DriveMode::ACKERMANN;
        }
    }
    
    void update_buttons(sensor_msgs::msg::Joy::ConstSharedPtr joy_p)
    {
        rcpputils::assert_true(joy_p->buttons.size() == NUM_BUTTONS, "Size mismatch between joy msg and NUM_BUTTONS");
        
        // Remember all pressed buttons
    }

    void update()
    {
        using std::to_string;
        if (joy_p)
        {
            if (button_released[TRIANGLE])
            {
                const auto previous_mode = reference.mode;
                reference.mode = next_drive_mode(reference.mode);
                
                RCLCPP_INFO(this->get_logger(), 
                    "Changed drive mode from " + previous_mode + " to " + reference.mode
                );
            }
            if (button_released[CIRCLE])
            {
                reference.turn = 0.0;
                RCLCPP_INFO(this->get_logger(), "Reset turn to zero");
            }

            // reset released buttons
            button_released.fill(false);

            // update reference message
            reference.speed = joy_p->axes[LEFT_UD]* 15.0/3.6;
            reference.turn = reference.turn + joy_p->axes[RIGHT_LR]/update_rate;
            
            reference_pub_p->publish(reference);
        }
    }
};


#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ManualOperationJoyNode)
