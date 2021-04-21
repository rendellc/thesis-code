

#include <cmath>
#include <control/PID.hpp>
#include <control/path/path.hpp>
#include <control/path/path_spiral.hpp>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "vehicle_interface/msg/drive_mode.hpp"
#include "vehicle_interface/msg/vehicle_controller_info.hpp"
#include "vehicle_interface/msg/waypoints.hpp"
#include "vehicle_interface/msg/wheel_state.hpp"

using ignition::math::Quaterniond;
using ignition::math::Vector2d;
using ignition::math::Vector3d;
using std::placeholders::_1;
using vehicle_interface::msg::VehicleControllerInfo;

class VehicleControllerNode : public rclcpp::Node {
 public:
  explicit VehicleControllerNode(const rclcpp::NodeOptions &options)
      : Node("vehicle_controller_node", options) {
    pose_sub_p = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "pose", 1, std::bind(&VehicleControllerNode::pose_callback, this, _1));
    twist_sub_p = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "twist", 1,
        std::bind(&VehicleControllerNode::twist_callback, this, _1));
    reference_sub_p =
        this->create_subscription<vehicle_interface::msg::DriveMode>(
            "reference", 1,
            std::bind(&VehicleControllerNode::reference_callback, this, _1));
    waypoints_sub_p =
        this->create_subscription<vehicle_interface::msg::Waypoints>(
            "waypoints", 1,
            std::bind(&VehicleControllerNode::waypoints_callback, this, _1));

    path_marker_pub_p =
        this->create_publisher<decltype(path_marker_msg)>("path_markers", 1);
    controller_markers_pub_p =
        this->create_publisher<decltype(controller_markers_msg)>(
            "controller_markers", 1);
    controller_markers_msg.markers.resize(1);

    info_pub_p =
        this->create_publisher<VehicleControllerInfo>("controller_info", 1);

    fl_pub_p = this->create_publisher<vehicle_interface::msg::WheelState>(
        "wheel_fl/reference", 1);
    rl_pub_p = this->create_publisher<vehicle_interface::msg::WheelState>(
        "wheel_rl/reference", 1);
    rr_pub_p = this->create_publisher<vehicle_interface::msg::WheelState>(
        "wheel_rr/reference", 1);
    fr_pub_p = this->create_publisher<vehicle_interface::msg::WheelState>(
        "wheel_fr/reference", 1);

    // this->declare_parameter<double>("update_rate", 20.0);
    this->declare_parameter("update_rate");
    const bool update_rate_set =
        this->get_parameter("update_rate", update_rate);
    if (!update_rate_set) {
      RCLCPP_WARN(this->get_logger(), "update_rate not set");
    }

    // this->get_parameter("update_rate", update_rate);
    timer_p = this->create_wall_timer(
        std::chrono::duration<double>(1 / update_rate.as_double()),
        std::bind(&VehicleControllerNode::update_command, this));

    this->declare_parameter("maximum_curvature");
    this->get_parameter<double>("maximum_curvature", maximum_curvature);

    this->declare_parameter("P_heading");
    this->get_parameter<double>("P_heading", heading_pid.P);

    this->declare_parameter("P_speed");
    this->get_parameter<double>("P_speed", speed_pid.P);

    this->declare_parameter("I_speed");
    this->get_parameter<double>("I_speed", speed_pid.I);

    this->declare_parameter("approach_angle");
    this->get_parameter<double>("approach_angle", approach_angle);

    this->declare_parameter("P_approach");
    this->get_parameter<double>("P_approach", P_approach);
  }

 private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_p;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_p;
  rclcpp::Subscription<vehicle_interface::msg::DriveMode>::SharedPtr
      reference_sub_p;
  rclcpp::Subscription<vehicle_interface::msg::Waypoints>::SharedPtr
      waypoints_sub_p;

  rclcpp::Publisher<VehicleControllerInfo>::SharedPtr info_pub_p;
  VehicleControllerInfo info_msg;

  rclcpp::Publisher<vehicle_interface::msg::WheelState>::SharedPtr fl_pub_p,
      rl_pub_p, rr_pub_p, fr_pub_p;
  vehicle_interface::msg::WheelState fl_msg, rl_msg, rr_msg, fr_msg;

  visualization_msgs::msg::Marker path_marker_msg;
  rclcpp::Publisher<decltype(path_marker_msg)>::SharedPtr path_marker_pub_p;

  visualization_msgs::msg::MarkerArray controller_markers_msg;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      controller_markers_pub_p;

  rclcpp::TimerBase::SharedPtr timer_p;

  geometry_msgs::msg::PoseStamped::SharedPtr pose_p;
  geometry_msgs::msg::TwistStamped::SharedPtr twist_p;
  vehicle_interface::msg::DriveMode::SharedPtr reference_p;
  vehicle_interface::msg::Waypoints::SharedPtr waypoints_p;

  std::shared_ptr<Path> path_p;

  PID speed_pid;
  PID heading_pid;

  rclcpp::Parameter update_rate;
  double maximum_curvature;
  // double maximum_curvature;
  double approach_angle;
  double P_approach;

  void do_reference_control() {
    // #if 0
    using Eigen::MatrixXd;
    using Eigen::Vector2d;
    using Eigen::VectorXd;

    // Wheel layout
    constexpr double L = 3.2;
    constexpr double W = 2.0;

    const std::array<Vector2d, 4> wheel_positions = {
        Vector2d(L / 2, W / 2), Vector2d(-L / 2, W / 2),
        Vector2d(-L / 2, -W / 2), Vector2d(L / 2, -W / 2)};

    std::array<double, 4> steering_angles = {};
    auto &delta_fl = steering_angles[0];
    auto &delta_rl = steering_angles[1];
    auto &delta_rr = steering_angles[2];
    auto &delta_fr = steering_angles[3];

    if (reference_p->mode == vehicle_interface::msg::DriveMode::ACKERMANN) {
      const auto &delta_f = reference_p->turn;
      delta_rl = delta_rr = 0.0;
      delta_fl =
          atan2(2 * L * sin(delta_f), 2 * L * cos(delta_f) - W * sin(delta_f));
      delta_fr =
          atan2(2 * L * sin(delta_f), 2 * L * cos(delta_f) + W * sin(delta_f));
    } else if (reference_p->mode == vehicle_interface::msg::DriveMode::AFAR) {
      const auto &delta = reference_p->turn;
      delta_fl = atan2(L * sin(delta), L * cos(delta) - W * sin(delta));
      delta_rl = -delta_fl;

      delta_fr = atan2(L * sin(delta), L * cos(delta) + W * sin(delta));
      delta_rr = -delta_fr;
    } else if (reference_p->mode == vehicle_interface::msg::DriveMode::SPIN) {
      constexpr double delta = 3.1415 / 2;
      delta_fl = atan2(L * sin(delta), L * cos(delta) - W * sin(delta));
      delta_rl = -delta_fl;

      delta_fr = atan2(L * sin(delta), L * cos(delta) + W * sin(delta));
      delta_rr = -delta_fr;
    } else if (reference_p->mode == vehicle_interface::msg::DriveMode::CRAB) {
      const auto &delta = reference_p->turn;
      delta_fl = delta_rl = delta_rr = delta_fr = delta;
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "unknown mode supplied: " + reference_p->mode);
    }

    // compute instantaneous center of rotation
    MatrixXd line_normals(4, 2);
    VectorXd line_positions(4);
    for (size_t i = 0; i < wheel_positions.size(); i++) {
      line_normals(i, 0) = cos(steering_angles[i]);
      line_normals(i, 1) = sin(steering_angles[i]);
      line_positions(i) = line_normals.row(i).dot(wheel_positions[i]);
    }
    const Vector2d icr =
        line_normals.colPivHouseholderQr().solve(line_positions);

    std::array<double, 4> wheel_icr_radii;
    for (size_t i = 0; i < wheel_positions.size(); i++) {
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

    // TODO(rendellc): scale speeds depending on wheel radius and distance to
    // ICR
    constexpr double wheel_radius = 0.505;
    const auto &v = reference_p->speed;

    fl_msg.angular_velocity =
        /* wheel_icr_radii[0]/vehicle_icr_radius * */ v / wheel_radius;
    rl_msg.angular_velocity =
        /* wheel_icr_radii[1]/vehicle_icr_radius * */ v / wheel_radius;
    rr_msg.angular_velocity =
        /* wheel_icr_radii[2]/vehicle_icr_radius * */ v / wheel_radius;
    fr_msg.angular_velocity =
        /* wheel_icr_radii[3]/vehicle_icr_radius * */ v / wheel_radius;
    // #endif
  }

  void update_command() {
    if (pose_p && twist_p && reference_p) {
      // prefer reference if supplied
      do_reference_control();

      // NOTE: if update_command is called more frequently than
      // reference_p are receieved, then this doesn't work.
      // Solution: keep track of when messages are received (prefered),
      // or add timestamp to message.
      reference_p.reset();
    }
    if (pose_p && twist_p && waypoints_p) {
      // use waypoints if reference not supplied
      // path_p->cross_track_error()
      const auto &q = pose_p->pose.orientation;
      const auto quat = Quaterniond(q.w, q.x, q.y, q.z);
      const auto yaw = quat.Yaw();
      const auto position =
          Vector2d(pose_p->pose.position.x, pose_p->pose.position.y);
      const auto velocity_body =
          Vector2d(twist_p->twist.linear.x, twist_p->twist.linear.y);
      const auto velocity_inertial =
          quat.RotateVector(Vector3d(velocity_body.X(), velocity_body.Y(), 0));
      const double sideslip = atan2(velocity_body.Y(), velocity_body.X());
      const double speed = velocity_body.Length();
      const double course = yaw + sideslip;
      const double &yawrate = twist_p->twist.angular.z;

      const auto path_position = path_p->closest_point(position);
      const auto path_direction = path_p->closest_direction(position);
      const auto path_error = position - path_position;

      const double path_course = atan2(path_direction.Y(), path_direction.X());

      const auto cross_track_error =
          Quaterniond(0, 0, path_course)
              .RotateVectorReverse(
                  Vector3d(path_error.X(), path_error.Y(), 0.0))
              .Y();

      // TODO(rendellc): need to rotate to vehicle coordinates

      using Marker = visualization_msgs::msg::Marker;
      controller_markers_msg.markers[0].header.frame_id = "map";
      controller_markers_msg.markers[0].header.stamp = this->get_clock()->now();
      controller_markers_msg.markers[0].ns = "vehicle/controller";
      controller_markers_msg.markers[0].id = 0;
      controller_markers_msg.markers[0].type = Marker::ARROW;
      controller_markers_msg.markers[0].action = Marker::ADD;
      controller_markers_msg.markers[0].color.r = 1.0;
      controller_markers_msg.markers[0].color.g = 1.0;
      controller_markers_msg.markers[0].color.b = 1.0;
      controller_markers_msg.markers[0].color.a = 1.0;
      controller_markers_msg.markers[0].scale.x = 0.1;
      controller_markers_msg.markers[0].scale.y = 0.2;
      controller_markers_msg.markers[0].scale.z = 0.0;
      controller_markers_msg.markers[0].points.resize(2);
      controller_markers_msg.markers[0].points[0] = pose_p->pose.position;
      controller_markers_msg.markers[0].points[1].x = path_position.X();
      controller_markers_msg.markers[0].points[1].y = path_position.Y();
      controller_markers_msg.markers[0].points[1].z = pose_p->pose.position.z;

      controller_markers_pub_p->publish(controller_markers_msg);

      if (!reference_p) {
        // only use waypoints if refernce_p not supplied
        const auto time_now = this->get_clock()->now();
        constexpr double wheel_radius = 0.505;

        // const double vx_reference = 2.0;
        const double speed_desired = 2.0;
        // const double &vx = twist_p->twist.linear.x;
        // const double speed_cg_u = speed_pid.update(vx_reference - vx,
        // time_now);

        // TODO(rendellc): transform speed_cg_u to speed signals for each wheel

        // fl_msg.angular_velocity = speed_cg_u / wheel_radius;
        // rl_msg.angular_velocity = speed_cg_u / wheel_radius;
        // rr_msg.angular_velocity = speed_cg_u / wheel_radius;
        // fr_msg.angular_velocity = speed_cg_u / wheel_radius;

        const double PI_HALF = atan2(1, 0);
        const double DEG_TO_RAD = PI_HALF / 90;

        const double max_steering_angle = 45 * DEG_TO_RAD;
        // const double approach_angle = 30 * DEG_TO_RAD;
        // const double approach_gain = 2;

        const double yaw_reference = 0.0;
        const double yaw_error = yaw_reference - yaw;

        // atan path following controller
        const double course_reference =
            path_course -
            approach_angle * atan(P_approach * cross_track_error) / PI_HALF;

        const auto ssa = [](double angle) {
          return atan2(sin(angle), cos(angle));
        };

        const double sideslip_desired = course_reference - yaw_reference;
        const double yawrate_desired = 1.0 * yaw_error;

        constexpr double cg_to_front = 1.0, cg_to_rear = 3.2, front_width = 2.0,
                         rear_width = 2.0;
        const auto steering_angle_front =
            atan2(speed * sin(sideslip_desired) + yawrate_desired * cg_to_front,
                  speed * cos(sideslip_desired));
        const auto steering_angle_rear =
            atan2(speed * sin(sideslip_desired) - yawrate_desired * cg_to_rear,
                  speed * cos(sideslip_desired));
        const double course_error = ssa(course_reference - course);

        // TODO(rendell): relate (course_error, yaw_error) to (steering_angles)

        // double steering_angle_front =
        //     heading_pid.update(course_error, time_now);

        // afar_steering(steering_angle_front, steering_angle_rear);

        const Vector2d pos_fl(cg_to_front, front_width / 2);
        const Vector2d pos_rl(-cg_to_rear, rear_width / 2);
        const Vector2d pos_rr(-cg_to_rear, -rear_width / 2);
        const Vector2d pos_fr(cg_to_front, -front_width / 2);

        const Vector2d velocity_body_desired(
            speed_desired * cos(sideslip_desired),
            speed_desired * sin(sideslip_desired));

        const Vector2d vel_fl =
            velocity_body_desired + yawrate_desired * pos_fl;
        const Vector2d vel_rl =
            velocity_body_desired + yawrate_desired * pos_rl;
        const Vector2d vel_rr =
            velocity_body_desired + yawrate_desired * pos_rr;
        const Vector2d vel_fr =
            velocity_body_desired + yawrate_desired * pos_fr;

        const double speed_fl = vel_fl.Length();
        const double speed_rl = vel_rl.Length();
        const double speed_rr = vel_rr.Length();
        const double speed_fr = vel_fr.Length();

        fl_msg.angular_velocity = speed_fl / wheel_radius;
        fl_msg.steering_angle = atan2(vel_fl.Y(), vel_fl.X());
        rl_msg.angular_velocity = speed_rl / wheel_radius;
        rl_msg.steering_angle = atan2(vel_rl.Y(), vel_rl.X());
        rr_msg.angular_velocity = speed_rr / wheel_radius;
        rr_msg.steering_angle = atan2(vel_rr.Y(), vel_rr.X());
        fr_msg.angular_velocity = speed_fr / wheel_radius;
        fr_msg.steering_angle = atan2(vel_fr.Y(), vel_fr.X());

        info_msg.path_course = path_course;
        info_msg.cross_track_error = cross_track_error;
        info_msg.yaw_error = yaw_error;
        info_msg.course_error = course_error;
        info_msg.steering_angle_front = steering_angle_front;
      }
    }

    publish_markers();

    info_pub_p->publish(info_msg);

    fl_pub_p->publish(fl_msg);
    rl_pub_p->publish(rl_msg);
    rr_pub_p->publish(rr_msg);
    fr_pub_p->publish(fr_msg);
  }

  void speed_yaw_controller_kimetal2018() {}

  void ackermann_steering(double steering_angle) {
    constexpr double L = 3.2;
    constexpr double W = 2.0;

    fl_msg.steering_angle =
        atan2(2 * L * sin(steering_angle),
              2 * L * cos(steering_angle) - W * sin(steering_angle));
    fl_msg.steering_angle_rate = 0.0;
    fr_msg.steering_angle =
        atan2(2 * L * sin(steering_angle),
              2 * L * cos(steering_angle) + W * sin(steering_angle));
    fr_msg.steering_angle_rate = 0.0;
    rr_msg.steering_angle = 0.0;
    rr_msg.steering_angle_rate = 0.0;
    rl_msg.steering_angle = 0.0;
    rl_msg.steering_angle_rate = 0.0;
  }

  void afar_steering(double steering_angle_front, double steering_angle_rear) {
    constexpr double L = 3.2;
    constexpr double W = 2.0;

    fl_msg.steering_angle = atan2(
        2 * L * sin(steering_angle_front),
        2 * L * cos(steering_angle_front) - W * sin(steering_angle_front));
    fl_msg.steering_angle_rate = 0.0;
    fr_msg.steering_angle = atan2(
        2 * L * sin(steering_angle_front),
        2 * L * cos(steering_angle_front) + W * sin(steering_angle_front));
    fr_msg.steering_angle_rate = 0.0;
    rr_msg.steering_angle =
        atan2(2 * L * sin(steering_angle_rear),
              2 * L * cos(steering_angle_rear) + W * sin(steering_angle_rear));
    rr_msg.steering_angle_rate = 0.0;
    rl_msg.steering_angle =
        atan2(2 * L * sin(steering_angle_rear),
              2 * L * cos(steering_angle_rear) - W * sin(steering_angle_rear));
    rl_msg.steering_angle_rate = 0.0;
  }

  void pose_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg_p) {
    pose_p = msg_p;
    RCLCPP_INFO_ONCE(this->get_logger(), "pose message recieved");
  }

  void twist_callback(geometry_msgs::msg::TwistStamped::SharedPtr msg_p) {
    twist_p = msg_p;
    RCLCPP_INFO_ONCE(this->get_logger(), "twist message recieved");
  }

  void reference_callback(vehicle_interface::msg::DriveMode::SharedPtr msg_p) {
    reference_p = msg_p;
    RCLCPP_INFO_ONCE(this->get_logger(), "reference message recieved");
  }

  void waypoints_callback(vehicle_interface::msg::Waypoints::SharedPtr msg_p) {
    waypoints_p = msg_p;
    RCLCPP_INFO_ONCE(this->get_logger(), "waypoints message recieved");

    std::vector<ignition::math::Vector2d> points;
    for (const auto &wp : waypoints_p->points) {
      points.emplace_back(wp.x, wp.y);
    }

    path_p = Path::fermat_smoothing(points, maximum_curvature);
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
      // const auto path_directions = path_p->sample_direction(num_samples);
      // std::vector<double> path_yaws(path_directions.size());
      // // std::transform(
      // //     path_directions.cbegin(), path_directions.cend(),
      // //     path_yaws.begin(),
      // //     [](const ignition::math::Vector2d& direction) {
      // //         return atan2(direction.Y(), direction.X());
      // //     }
      // // );

      //  path_marker_msg.header.stamp = this->get_clock()->now();
      //  path_marker_msg.header.frame_id = "map";

      //  //path_marker_msg.poses.clear();
      //  for (int i = 0; i < path_points.size(); i++)
      //  {
      //      const auto& pos = path_points[i];
      //      const auto& dir = path_directions[i];
      //      const auto& yaw = atan2(dir.Y(), dir.X());
      //
      //      if (std::isnan(yaw))
      //      {
      //          RCLCPP_WARN(
      //              this->get_logger(),
      //              "Direction " + std::to_string(dir.X()) +
      //              std::to_string(dir.Y()) +  " gave yaw " +
      //              std::to_string(yaw));
      //      }

      //      geometry_msgs::msg::Pose pose_msg;
      //      pose_msg.position.x = pos.X();
      //      pose_msg.position.y = pos.Y();
      //      pose_msg.position.z = 1.0;

      //      pose_msg.orientation.x = sin(yaw/2)*cos(0);
      //      pose_msg.orientation.y = sin(yaw/2)*cos(0);
      //      pose_msg.orientation.z = sin(yaw/2)*cos(1);
      //      pose_msg.orientation.w = cos(yaw/2);
      //
      //      path_marker_msg.poses.push_back(pose_msg);
      //  }

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

      // const auto closest = path_p->closest_point(pos);
      // geometry_msgs::msg::Point closest_geom;
      // closest_geom.x = closest.X();
      // closest_geom.y = closest.X();
      // closest_geom.z = 1.0;
      //

      // marker_closest_msg.header.frame_id = "map";
      // marker_closest_msg.header.stamp = this->get_clock()->now();
      //
      // marker_closest_msg.ns = "vehicle";
      // marker_closest_msg.id = 2;

      // // TODO: change this to nav_msgs::msg::Path
      // marker_closest_msg.type = visualization_msgs::msg::Marker::SPHERE;
      // marker_closest_msg.action = visualization_msgs::msg::Marker::ADD;
      // marker_closest_msg.points.push_back(closest_geom);
      // marker_closest_msg.scale.x = 0.2;
      // marker_closest_msg.scale.y = 0.2;
      // marker_closest_msg.scale.z = 0.2;

      // marker_closest_msg.color.r = 0.0;
      // marker_closest_msg.color.g = 1.0;
      // marker_closest_msg.color.b = 0.0;
      // marker_closest_msg.color.a = 1.0;
    }
  }
};

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(VehicleControllerNode)