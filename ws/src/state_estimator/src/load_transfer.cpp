
#include <rclcpp/rclcpp.hpp>
#include <rcpputils/asserts.hpp>

#include <vehicle_interface/msg/wheel_loads.hpp>
#include <vehicle_interface/msg/wheel_load.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

using std::placeholders::_1;


class LoadTransferNode : public rclcpp::Node
{
public:
    LoadTransferNode(const rclcpp::NodeOptions& options)
    : Node("load_transfer_node", options)
    {
        const auto load_double = [this](const std::string& name, double& variable){
            this->declare_parameter<double>(name, variable);
            rcpputils::require_true(
                this->get_parameter(name, variable),
                "parameter " + name + " not found"
            );
        };
        load_double("vehicle_mass", mass);
        
        // TODO: these parameters can be computed using vehicle and baselink pose
        // But this approach is simpler and less prone to error
        load_double("cog_to_rear", cog_to_rear);
        load_double("cog_to_front", cog_to_front);
        load_double("width_rear", width_rear);
        load_double("width_front", width_front);
        load_double("height_cog", height_cog);
        
        twist_sub_p = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "twist", 1, std::bind(&LoadTransferNode::twist_callback, this, _1)
        );
        
        load_pub_p = this->create_publisher<vehicle_interface::msg::WheelLoads>("wheel_loads", 1);
        load_fl_pub_p = this->create_publisher<vehicle_interface::msg::WheelLoad>("wheel_fl/load", 1);
        load_rl_pub_p = this->create_publisher<vehicle_interface::msg::WheelLoad>("wheel_rl/load", 1);
        load_rr_pub_p = this->create_publisher<vehicle_interface::msg::WheelLoad>("wheel_rr/load", 1);
        load_fr_pub_p = this->create_publisher<vehicle_interface::msg::WheelLoad>("wheel_fr/load", 1);
        
        double update_rate;
        load_double("update_rate", update_rate);
        timer_p = this->create_wall_timer(
                std::chrono::duration<double>(1/update_rate),
                std::bind(&LoadTransferNode::update_load, this)
        );
    }

private:
    double mass, cog_to_rear, cog_to_front, width_rear, width_front, height_cog;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_p;
    rclcpp::Publisher<vehicle_interface::msg::WheelLoads>::SharedPtr load_pub_p;
    rclcpp::Publisher<vehicle_interface::msg::WheelLoad>::SharedPtr load_fl_pub_p;
    rclcpp::Publisher<vehicle_interface::msg::WheelLoad>::SharedPtr load_rl_pub_p;
    rclcpp::Publisher<vehicle_interface::msg::WheelLoad>::SharedPtr load_rr_pub_p;
    rclcpp::Publisher<vehicle_interface::msg::WheelLoad>::SharedPtr load_fr_pub_p;

    rclcpp::TimerBase::SharedPtr timer_p;

    geometry_msgs::msg::TwistStamped::SharedPtr twist_p;
    vehicle_interface::msg::WheelLoads wheel_loads;
    
    void twist_callback(geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        // TODO: need to estimate acceleration as well
        twist_p = msg;
    }
    

    void update_load()
    {
        constexpr double g = 9.81;
        constexpr double ax_ch = 0.0;
        constexpr double ay_ch = 0.0;

        // concise variable names
        const auto& m = mass;
        const auto& lr = cog_to_rear;
        const auto& lf = cog_to_front;
        const auto& hcg = height_cog;
        const auto& bf = width_front;
        const auto& br = width_rear;
        const auto l = cog_to_front + cog_to_rear;
        
        wheel_loads.front_left.load = m*g*(lr/l - hcg*ax_ch/(l*g))*(0.5 - hcg*ay_ch/(bf*g));
        wheel_loads.rear_left.load = m*g*(lf/l + hcg*ax_ch/(l*g))*(0.5 - hcg*ay_ch/(br*g));
        wheel_loads.rear_right.load = m*g*(lf/l + hcg*ax_ch/(l*g))*(0.5 + hcg*ay_ch/(br*g));
        wheel_loads.front_right.load = m*g*(lr/l - hcg*ax_ch/(l*g))*(0.5 + hcg*ay_ch/(bf*g));
        
        load_pub_p->publish(wheel_loads);
        load_fl_pub_p->publish(wheel_loads.front_left);
        load_rl_pub_p->publish(wheel_loads.rear_left);
        load_rr_pub_p->publish(wheel_loads.rear_right);
        load_fr_pub_p->publish(wheel_loads.front_right);
    }
};





#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(LoadTransferNode)
