#include <rclcpp/rclcpp.hpp>
#include "vehicle_interface/msg/waypoints.hpp"
#include <geometry_msgs/msg/point.hpp>

#include <memory>

class WaypointPublisherNode : public rclcpp::Node
{
public:
    WaypointPublisherNode(const rclcpp::NodeOptions& options)
    : Node("waypoint_publisher_node", options)
    {
        waypoints_pub_p = this->create_publisher<typeof(waypoints)>(
            "waypoints", 1
        );
        geometry_msgs::msg::Point wp;

        wp.x = wp.y = wp.z = 0;
        waypoints.points.push_back(wp);
        wp.x += 50;
        waypoints.points.push_back(wp);
        wp.y += 50;
        waypoints.points.push_back(wp);
        wp.x += -50;
        waypoints.points.push_back(wp);
        wp.y += -50;
        waypoints.points.push_back(wp);

        timer_p = this->create_wall_timer(
                std::chrono::duration<double>(1.0),
                std::bind(&WaypointPublisherNode::update, this)
        );
    }
    
private:
    vehicle_interface::msg::Waypoints waypoints;

    rclcpp::Publisher<typeof(waypoints)>::SharedPtr waypoints_pub_p;
    
    rclcpp::TimerBase::SharedPtr timer_p;
    
    void update()
    {
        waypoints_pub_p->publish(waypoints);
    }
};


#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(WaypointPublisherNode)