
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "vehicle_interface/msg/wheel_state.hpp"
#include "vehicle_interface/msg/drive_mode.hpp"
#include "vehicle_interface/msg/waypoints.hpp"

#include <eigen3/Eigen/Dense>

#include <control/path/path.hpp>
#include <ignition/math/Vector2.hh>

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
        waypoints_sub_p = this->create_subscription<vehicle_interface::msg::Waypoints>(
            "waypoints", 1, std::bind(&VehicleControllerNode::waypoints_callback, this, _1)
        );
        
        path_marker_pub_p = this->create_publisher<visualization_msgs::msg::Marker>("path_markers",1);
        
        
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

        this->declare_parameter<double>("maximum_curvature", 0.25);
        this->get_parameter("maximum_curvature", maximum_curvature);
    }

private:
    rclcpp::Subscription<vehicle_interface::msg::DriveMode>::SharedPtr reference_sub_p;
    rclcpp::Subscription<vehicle_interface::msg::Waypoints>::SharedPtr waypoints_sub_p;

    rclcpp::Publisher<vehicle_interface::msg::WheelState>::SharedPtr fl_pub_p, rl_pub_p, rr_pub_p, fr_pub_p;
    vehicle_interface::msg::WheelState fl_msg, rl_msg, rr_msg, fr_msg;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_p;
    visualization_msgs::msg::Marker marker_msg;

    rclcpp::TimerBase::SharedPtr timer_p;
    
    vehicle_interface::msg::DriveMode::SharedPtr reference_p;
    vehicle_interface::msg::Waypoints::SharedPtr waypoints_p;
    
    std::shared_ptr<Path> path_p;
    
    double update_rate;
    double maximum_curvature;
    
    void do_reference_control()
    {
#if 0
        using Eigen::MatrixXd;
        using Eigen::Vector2d;
        using Eigen::VectorXd;

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
#endif
    }

    void update_command()
    {

        if (reference_p)
        {
            // prefer reference if supplied
            do_reference_control();

            // NOTE: if update_command is called more frequently than
            // reference_p are receieved, then this doesn't work.
            // Solution: keep track of when messages are received (prefered),
            // or add timestamp to message.
            reference_p.reset();
            
        } else if (waypoints_p)
        {
            // use waypoints if reference not supplied
            // path_p->cross_track_error()
        }
        

        publish_markers();

        fl_pub_p->publish(fl_msg);
        rl_pub_p->publish(rl_msg);
        rr_pub_p->publish(rr_msg);
        fr_pub_p->publish(fr_msg);
        
    }
    
    void reference_callback(vehicle_interface::msg::DriveMode::SharedPtr msg_p)
    {
        reference_p = msg_p;
        RCLCPP_INFO_ONCE(this->get_logger(), "reference message recieved");
    }
    
    void waypoints_callback(vehicle_interface::msg::Waypoints::SharedPtr msg_p)
    {
        waypoints_p = msg_p;
        RCLCPP_INFO_ONCE(this->get_logger(), "waypoints message recieved");
        
        std::vector<ignition::math::Vector2d> points;
        for (const auto& wp : waypoints_p->points)
        {
            points.emplace_back(wp.x, wp.y);
        }

        // TODO: create and update path here
        path_p = Path::fermat_smoothing(points, maximum_curvature);
    }
    
    void publish_markers()
    {
        if (path_p)
        {
            constexpr int num_samples = 100;
            const auto path_points = path_p->sample(num_samples);
            std::vector<geometry_msgs::msg::Point> points;
            points.reserve(num_samples);
            
            geometry_msgs::msg::Point p_geom;
            for (const auto& p : path_points)
            {
                p_geom.x = p.X();
                p_geom.y = p.Y();
                p_geom.z = 1.0;

                points.push_back(p_geom);
            }
            
            marker_msg.header.frame_id = "map";
            marker_msg.header.stamp = this->get_clock()->now();

            marker_msg.ns = "vehicle";
            marker_msg.id = 1;

            // TODO: change this to nav_msgs::msg::Path
            marker_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker_msg.action = visualization_msgs::msg::Marker::ADD;
            marker_msg.points = points;
            marker_msg.scale.x = 0.1;

            marker_msg.color.r = 0.0;
            marker_msg.color.g = 1.0;
            marker_msg.color.b = 0.0;
            marker_msg.color.a = 1.0;
            
            path_marker_pub_p->publish(marker_msg);
        }
    }
};

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(VehicleControllerNode)