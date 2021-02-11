
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include "vehicle_interface/msg/wheel_state.hpp"
#include "vehicle_interface/msg/drive_mode.hpp"

#include <memory>
#include <cmath>

using std::placeholders::_1;

class VehicleControllerNode : public rclcpp::Node
{
public:
    VehicleControllerNode(const rclcpp::NodeOptions& options)
    : Node("vehicle_controller_node", options)
    {
        reference_sub_p = this->create_subscription<vehicle_interface::msg::DriveMode>(
            "reference", 1, std::bind(&VehicleControllerNode::reference_callback, this, _1)
        );

        fl_pub_p = this->create_publisher<vehicle_interface::msg::WheelState>("wheel_fl/reference", 1);
        rl_pub_p = this->create_publisher<vehicle_interface::msg::WheelState>("wheel_rl/reference", 1);
        rr_pub_p = this->create_publisher<vehicle_interface::msg::WheelState>("wheel_rr/reference", 1);
        fr_pub_p = this->create_publisher<vehicle_interface::msg::WheelState>("wheel_fr/reference", 1);

        this->declare_parameter<double>("update_rate", 20.0);
        this->get_parameter("update_rate", update_rate);
        timer_p = this->create_wall_timer(
                std::chrono::duration<double>(1/update_rate),
                std::bind(&VehicleControllerNode::update_command, this)
        );
        

        // TODO: remove this
        // reference_p = std::make_shared<geometry_msgs::msg::Twist>();
    }

private:
    rclcpp::Subscription<vehicle_interface::msg::DriveMode>::SharedPtr reference_sub_p;
    //rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr reference_sub_p;
    rclcpp::Publisher<vehicle_interface::msg::WheelState>::SharedPtr fl_pub_p, rl_pub_p, rr_pub_p, fr_pub_p;
    vehicle_interface::msg::WheelState fl_msg, rl_msg, rr_msg, fr_msg;
    rclcpp::TimerBase::SharedPtr timer_p;
    
    // geometry_msgs::msg::Twist::SharedPtr reference_p;
    vehicle_interface::msg::DriveMode::SharedPtr reference_p;
    
    double update_rate;
    

    void update_command()
    {
        if (reference_p)
        {
            // Wheel layout
            constexpr double L = 3.2;
            constexpr double W = 2.0;

            double delta_fl = 0.0;
            double delta_rl = 0.0;
            double delta_rr = 0.0;
            double delta_fr = 0.0;
            switch (reference_p->mode)
            {
            case vehicle_interface::msg::DriveMode::ACKERMANN:{
                const auto& delta_f = reference_p->turn;
                delta_rl = delta_rr = 0.0;
                delta_fl = atan2(
                    2*L*sin(delta_f), 2*L*cos(delta_f) - W*sin(delta_f)
                );
                delta_fr = atan2(
                    2*L*sin(delta_f), 2*L*cos(delta_f) + W*sin(delta_f)
                );
                break;
            } case vehicle_interface::msg::DriveMode::AFAR:{
                const auto& delta = reference_p->turn;
                delta_fl = atan2(
                    2*L*sin(delta), 2*L*cos(delta) - W*sin(delta)
                )/2;
                delta_rl = -atan2(
                    2*L*sin(delta), 2*L*cos(delta) - W*sin(delta)
                )/2;
                delta_rr = -atan2(
                    2*L*sin(delta), 2*L*cos(delta) + W*sin(delta)
                )/2;
                delta_fr = atan2(
                    2*L*sin(delta), 2*L*cos(delta) + W*sin(delta)
                )/2;
                break;
             } case vehicle_interface::msg::DriveMode::SPIN:{
                constexpr double delta = 3.1415/2;
                delta_fl = atan2(
                    2*L*sin(delta), 2*L*cos(delta) - W*sin(delta)
                )/2;
                delta_rl = -atan2(
                    2*L*sin(delta), 2*L*cos(delta) - W*sin(delta)
                )/2;
                delta_rr = -atan2(
                    2*L*sin(delta), 2*L*cos(delta) + W*sin(delta)
                )/2;
                delta_fr = atan2(
                    2*L*sin(delta), 2*L*cos(delta) + W*sin(delta)
                )/2;
                break;
             } case vehicle_interface::msg::DriveMode::CRAB:{
                const auto& delta = reference_p->turn;
                delta_fl = delta_rl = delta_rr = delta_fr = delta;
                break;
            } 
            default:
                RCLCPP_WARN(this->get_logger(), "unknown mode supplied: %i", reference_p->mode);
            }
                
            fl_msg.steering_angle = delta_fl;
            fl_msg.steering_angle_rate = 0.0;
            rl_msg.steering_angle = delta_rl;
            rl_msg.steering_angle_rate = 0.0;
            fr_msg.steering_angle = delta_fr;
            fr_msg.steering_angle_rate = 0.0;
            rr_msg.steering_angle = delta_rr;
            rr_msg.steering_angle_rate = 0.0;
            

            // TODO: scale speeds depending on wheel radius and distance to ICR
            constexpr double R = 0.505;
            const double omega = R*reference_p->speed; 

            fl_msg.angular_velocity = omega;
            rl_msg.angular_velocity = omega;
            rr_msg.angular_velocity = omega;
            fr_msg.angular_velocity = omega;
            

            
            fl_pub_p->publish(fl_msg);
            rl_pub_p->publish(rl_msg);
            rr_pub_p->publish(rr_msg);
            fr_pub_p->publish(fr_msg);
        }
    }
    
    void reference_callback(vehicle_interface::msg::DriveMode::SharedPtr msg_p)
    {
        reference_p = msg_p;
        RCLCPP_INFO_ONCE(this->get_logger(), "reference message recieved");
    }
};

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(VehicleControllerNode)

// int main(int argc, char* argv[]) 
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<VehicleControllerNode>());
//     rclcpp::shutdown();
//     return 0;
// }