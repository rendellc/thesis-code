#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "vehicle_interface/msg/waypoints.hpp"

class WaypointPublisherNode : public rclcpp::Node {
 public:
  explicit WaypointPublisherNode(const rclcpp::NodeOptions& options)
      : Node("waypoint_publisher_node", options) {
    waypoints_pub_p = this->create_publisher<typeof(waypoints)>("waypoints", 1);
    waypoint_markers_pub_p =
        this->create_publisher<typeof(waypoint_markers)>("waypoint_markers", 1);

    this->declare_parameter("waypoint_xs");
    std::vector<double> waypoint_xs =
        this->get_parameter("waypoint_xs").as_double_array();

    this->declare_parameter("waypoint_ys");
    std::vector<double> waypoint_ys =
        this->get_parameter("waypoint_ys").as_double_array();

    if (waypoint_xs.size() != waypoint_ys.size()) {
      RCLCPP_WARN_STREAM(this->get_logger(), "different number of x ("
                                                 << waypoint_xs.size()
                                                 << ") and y ("
                                                 << waypoint_ys.size()
                                                 << ") waypoints" << std::endl);
      return;
    }

    const int num_waypoints = waypoint_xs.size();

    for (int i = 0; i < num_waypoints; i++) {
      geometry_msgs::msg::Point wp;
      wp.x = waypoint_xs[i];
      wp.y = waypoint_ys[i];
      waypoints.points.push_back(wp);
    }

    waypoint_markers.header.frame_id = "map";
    waypoint_markers.ns = "vehicle";
    waypoint_markers.id = 3;
    using Marker = visualization_msgs::msg::Marker;
    waypoint_markers.type = Marker::SPHERE_LIST;
    waypoint_markers.action = Marker::ADD;

    waypoint_markers.points = waypoints.points;
    waypoint_markers.scale.x = 0.5;
    waypoint_markers.scale.y = 0.5;
    waypoint_markers.scale.z = 0.5;
    waypoint_markers.color.r = 0.0;
    waypoint_markers.color.g = 1.0;
    waypoint_markers.color.b = 0.0;
    waypoint_markers.color.a = 0.5;

    timer_p = this->create_wall_timer(
        std::chrono::duration<double>(1.0),
        std::bind(&WaypointPublisherNode::update, this));
  }

 private:
  vehicle_interface::msg::Waypoints waypoints;
  visualization_msgs::msg::Marker waypoint_markers;

  rclcpp::Publisher<typeof(waypoints)>::SharedPtr waypoints_pub_p;
  rclcpp::Publisher<typeof(waypoint_markers)>::SharedPtr waypoint_markers_pub_p;

  rclcpp::TimerBase::SharedPtr timer_p;

  void update() {
    waypoints_pub_p->publish(waypoints);

    waypoint_markers.header.stamp = this->get_clock()->now();
    waypoint_markers_pub_p->publish(waypoint_markers);
  }
};

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(WaypointPublisherNode)
