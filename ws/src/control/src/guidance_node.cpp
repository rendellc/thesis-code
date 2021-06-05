
#include <control/PID.hpp>
#include <control/path/path.hpp>
#include <control/ssa.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_interface/msg/guidance_info.hpp>
#include <vehicle_interface/msg/guide.hpp>
#include <vehicle_interface/msg/waypoints.hpp>
#include <vehicle_interface/msg/yaw_reference.hpp>
#include <visualization_msgs/msg/marker.hpp>

using control::ssa;
using ignition::math::Quaterniond;
using ignition::math::Vector2d;
using ignition::math::Vector3d;
using std::placeholders::_1;
using vehicle_interface::msg::GuidanceInfo;
using vehicle_interface::msg::Guide;
using vehicle_interface::msg::YawReference;

class GuidanceNode : public rclcpp::Node {
 public:
  explicit GuidanceNode(const rclcpp::NodeOptions &options)
      : Node("vehicle_controller_node", options) {
    pose_sub_p = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "pose", 1, std::bind(&GuidanceNode::pose_callback, this, _1));
    waypoints_sub_p =
        this->create_subscription<vehicle_interface::msg::Waypoints>(
            "waypoints", 1,
            std::bind(&GuidanceNode::waypoints_callback, this, _1));

    path_marker_pub_p =
        this->create_publisher<decltype(path_marker_msg)>("path_markers", 1);

    guidance_pub_p =
        this->create_publisher<vehicle_interface::msg::Guide>("guide", 1);

    info_pub_p = this->create_publisher<GuidanceInfo>("guidance_info", 1);
    yaw_reference_pub_p =
        this->create_publisher<YawReference>("yaw_reference", 1);

    // this->declare_parameter<double>("update_rate", 20.0);
    this->declare_parameter("update_rate");
    this->get_parameter("update_rate", update_rate);

    timer_p =
        this->create_wall_timer(std::chrono::duration<double>(1 / update_rate),
                                std::bind(&GuidanceNode::update_command, this));

    this->declare_parameter("maximum_curvature");
    this->get_parameter<double>("maximum_curvature", maximum_curvature);

    this->declare_parameter("use_fermat_smoothing");
    this->get_parameter<bool>("use_fermat_smoothing", use_fermat_smoothing);

    this->declare_parameter("use_circular_smoothing");
    this->get_parameter<bool>("use_circular_smoothing", use_circular_smoothing);

    this->declare_parameter("use_braking");
    this->get_parameter<bool>("use_braking", use_braking);

    this->declare_parameter("approach_angle");
    this->get_parameter<double>("approach_angle", approach_angle);

    this->declare_parameter("P_approach");
    this->get_parameter<double>("P_approach", approach_pid.P);
    this->declare_parameter("I_approach");
    this->get_parameter<double>("I_approach", approach_pid.I);
    this->declare_parameter("D_approach");
    this->get_parameter<double>("D_approach", approach_pid.D);

    this->declare_parameter("speed_desired");
    this->get_parameter<double>("speed_desired", speed_desired);
  }

 private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_p;
  rclcpp::Subscription<vehicle_interface::msg::Waypoints>::SharedPtr
      waypoints_sub_p;

  rclcpp::Publisher<Guide>::SharedPtr guidance_pub_p;
  rclcpp::Publisher<GuidanceInfo>::SharedPtr info_pub_p;
  rclcpp::Publisher<YawReference>::SharedPtr yaw_reference_pub_p;

  visualization_msgs::msg::Marker path_marker_msg;
  rclcpp::Publisher<decltype(path_marker_msg)>::SharedPtr path_marker_pub_p;

  rclcpp::TimerBase::SharedPtr timer_p;

  geometry_msgs::msg::PoseStamped::SharedPtr pose_p;
  vehicle_interface::msg::Waypoints::SharedPtr waypoints_p;

  std::shared_ptr<Path> path_p;

  static constexpr double PI_HALF = 1.57079632679;

  // parameters
  double update_rate;
  double maximum_curvature;
  double approach_angle;
  bool use_fermat_smoothing;
  bool use_circular_smoothing;
  bool use_braking;
  double speed_desired;

  PID approach_pid;

  // Reference
  rclcpp::Time time_now;
  GuidanceInfo info_msg;

  void update_command() {
    time_now = this->get_clock()->now();
    info_msg.header.stamp = time_now;
    if (path_p && pose_p) {
      const Vector2d position(pose_p->pose.position.x, pose_p->pose.position.y);
      const Vector2d path_position = path_p->closest_point(position);
      const Vector2d path_direction = path_p->closest_direction(position);

      info_msg.path_position.x = path_position.X();
      info_msg.path_position.y = path_position.Y();
      info_msg.path_course = atan2(path_direction.Y(), path_direction.X());
      info_msg.path_courserate = path_p->closest_courserate(position);

      const Vector2d path_error = position - path_position;
      info_msg.path_error.x = path_error.X();
      info_msg.path_error.y = path_error.Y();
      info_msg.cross_track_error = Quaterniond(0, 0, info_msg.path_course)
                                       .RotateVectorReverse(Vector3d(
                                           path_error.X(), path_error.Y(), 0.0))
                                       .Y();
      info_msg.approach_error =
          approach_pid.update(info_msg.cross_track_error, time_now);
      info_msg.guide.course =
          info_msg.path_course -
          approach_angle * atan(info_msg.approach_error) / PI_HALF;

      if (use_braking) {
        // Step path position a couple seconds into the future to see if
        // movement direction will change
        Vector2d path_position_future = path_position;
        Vector2d path_direction_future = path_direction;
        const int n_steps = 20;
        const double dt = 0.1;
        for (int i = 0; i < n_steps; i++) {
          path_position_future += dt * speed_desired * path_direction_future;
          path_direction_future =
              path_p->closest_direction(path_position_future);
        }
        const double path_course_future =
            atan2(path_direction_future.Y(), path_direction_future.X());
        const double direction_difference =
            fabs(ssa(info_msg.path_course - path_course_future));
        const double direction_difference_normalized =
            direction_difference / (2 * PI_HALF);

        const double brake_factor = direction_difference_normalized;
        info_msg.guide.speed = speed_desired * (1 - brake_factor);
      } else {
        info_msg.guide.speed = speed_desired;
      }

      guidance_pub_p->publish(info_msg.guide);

      info_msg.yaw_reference.source = YawReference::PATH;
      info_msg.yaw_reference.yaw = info_msg.path_course;
      info_msg.yaw_reference.yawrate = info_msg.path_courserate;

      yaw_reference_pub_p->publish(info_msg.yaw_reference);
    }
    info_pub_p->publish(info_msg);
    publish_markers();
  }

  void pose_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg_p) {
    pose_p = msg_p;
    RCLCPP_INFO_ONCE(this->get_logger(), "pose message recieved");
  }

  void waypoints_callback(vehicle_interface::msg::Waypoints::SharedPtr msg_p) {
    waypoints_p = msg_p;
    RCLCPP_INFO_ONCE(this->get_logger(), "waypoints message recieved");

    std::vector<ignition::math::Vector2d> points;
    for (const auto &wp : waypoints_p->points) {
      points.emplace_back(wp.x, wp.y);
    }

    if (use_fermat_smoothing) {
      path_p = Path::fermat_smoothing(points, maximum_curvature);
    } else if (use_circular_smoothing) {
      path_p = Path::circular_smoothing(points, 1 / maximum_curvature);
    } else {
      path_p = Path::straight_line_path(points);
    }
    // path_p = Path::straight_line_path(points);
    // path_p = std::make_shared<PathSpiral>(
    //     Vector2d(10 * sqrt(1.5) * cos(1.5), 10 * sqrt(1.5) * sin(1.5)), 0,
    //     10, 1.5, 0);

    // Update marker message
    constexpr int num_samples = 500;
    const auto path_points = path_p->sample(num_samples);
    path_marker_msg.points.resize(path_points.size());
    geometry_msgs::msg::Point p_geom;
    for (int i = 0; i < path_points.size(); i++) {
      const auto &p = path_points[i];

      p_geom.x = p.X();
      p_geom.y = p.Y();
      p_geom.z = pose_p ? pose_p->pose.position.z : 0;

      path_marker_msg.points[i] = p_geom;
    }
  }

  void publish_markers() {
    if (path_p) {
      path_marker_msg.header.frame_id = "map";
      path_marker_msg.header.stamp = this->get_clock()->now();

      path_marker_msg.ns = "vehicle";
      path_marker_msg.id = 1;

      // TODO: change this to nav_msgs::msg::Path
      path_marker_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
      path_marker_msg.action = visualization_msgs::msg::Marker::ADD;
      path_marker_msg.scale.x = 0.1;

      path_marker_msg.color.r = 0.0;
      path_marker_msg.color.g = 1.0;
      path_marker_msg.color.b = 0.0;
      path_marker_msg.color.a = 1.0;
      path_marker_pub_p->publish(path_marker_msg);
    }
  }
};

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(GuidanceNode)