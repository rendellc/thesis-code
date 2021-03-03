
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include "vehicle_interface/msg/wheel_state.hpp"
#include "vehicle_interface/msg/drive_mode.hpp"

#include <eigen3/Eigen/Dense>

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
        using Eigen::MatrixXd;
        using Eigen::Vector2d;
        using Eigen::VectorXd;

        if (reference_p)
        {
            // Wheel layout
            constexpr double L = 3.2;
            constexpr double W = 2.0;
            
            const std::array<Vector2d,4> wheel_positions = {
                Vector2d( L/2, W/2),
                Vector2d(-L/2, W/2),
                Vector2d(-L/2,-W/2),
                Vector2d( L/2,-W/2)
            };
            std::array<double,4> steering_angles = {};
            auto& delta_fl = steering_angles[0];
            auto& delta_rl = steering_angles[1];
            auto& delta_rr = steering_angles[2];
            auto& delta_fr = steering_angles[3];

            if (reference_p->mode == vehicle_interface::msg::DriveMode::ACKERMANN) 
            {
                const auto& delta_f = reference_p->turn;
                delta_rl = delta_rr = 0.0;
                delta_fl = atan2(
                    2*L*sin(delta_f), 2*L*cos(delta_f) - W*sin(delta_f)
                );
                delta_fr = atan2(
                    2*L*sin(delta_f), 2*L*cos(delta_f) + W*sin(delta_f)
                );
            } else if (reference_p->mode == vehicle_interface::msg::DriveMode::AFAR) 
            {
                const auto& delta = reference_p->turn;
                delta_fl = atan2(
                    L*sin(delta), L*cos(delta) - W*sin(delta)
                );
                delta_rl = -delta_fl;

                delta_fr = atan2(
                    L*sin(delta), L*cos(delta) + W*sin(delta)
                );
                delta_rr = -delta_fr;
            } else if (reference_p->mode == vehicle_interface::msg::DriveMode::SPIN) 
            {
                constexpr double delta = 3.1415/2;
                delta_fl = atan2(
                    L*sin(delta), L*cos(delta) - W*sin(delta)
                );
                delta_rl = -delta_fl;

                delta_fr = atan2(
                    L*sin(delta), L*cos(delta) + W*sin(delta)
                );
                delta_rr = -delta_fr;
            } else if (reference_p->mode == vehicle_interface::msg::DriveMode::CRAB) 
            {
                const auto& delta = reference_p->turn;
                delta_fl = delta_rl = delta_rr = delta_fr = delta;
            } else 
            {
                RCLCPP_WARN(this->get_logger(), "unknown mode supplied: " + reference_p->mode);
            }
            

            // compute instantaneous center of rotation
            MatrixXd line_normals(4,2);
            VectorXd line_positions(4);
            for (size_t i = 0; i < wheel_positions.size(); i++)
            {
                line_normals(i,0) = cos(steering_angles[i]);
                line_normals(i,1) = sin(steering_angles[i]);
                line_positions(i) = line_normals.row(i).dot(wheel_positions[i]);
            }
            const Vector2d icr = line_normals.colPivHouseholderQr().solve(line_positions);
            
            const double vehicle_icr_radius = icr.norm();
            std::array<double, 4> wheel_icr_radii;
            for (size_t i = 0; i < wheel_positions.size(); i++)
            {
                wheel_icr_radii[i] = (icr - wheel_positions[i]).norm();
            }
            
            
            // update steering angle reference message
            fl_msg.steering_angle = delta_fl;
            fl_msg.steering_angle_rate = 0.0;
            rl_msg.steering_angle = delta_rl;
            rl_msg.steering_angle_rate = 0.0;
            fr_msg.steering_angle = delta_fr;
            fr_msg.steering_angle_rate = 0.0;
            rr_msg.steering_angle = delta_rr;
            rr_msg.steering_angle_rate = 0.0;
            

            // TODO: scale speeds depending on wheel radius and distance to ICR
            constexpr double wheel_radius = 0.505;
            const auto& v = reference_p->speed; 

            fl_msg.angular_velocity = /* wheel_icr_radii[0]/vehicle_icr_radius * */ v/wheel_radius;
            rl_msg.angular_velocity = /* wheel_icr_radii[1]/vehicle_icr_radius * */ v/wheel_radius;
            rr_msg.angular_velocity = /* wheel_icr_radii[2]/vehicle_icr_radius * */ v/wheel_radius;
            fr_msg.angular_velocity = /* wheel_icr_radii[3]/vehicle_icr_radius * */ v/wheel_radius;

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