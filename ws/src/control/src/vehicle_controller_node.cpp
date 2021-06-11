#include <algorithm>
#include <cmath>
#include <control/PID.hpp>
#include <control/clip.hpp>
#include <control/dynamics/no_slip_4wis.hpp>
#include <control/dynamics/singletrack_kinematic.hpp>
#include <control/iterative_lqr.hpp>
#include <control/ssa.hpp>
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
#include <rclcpp/rclcpp.hpp>
#include <vehicle_interface/msg/drive_mode.hpp>
#include <vehicle_interface/msg/guide.hpp>
#include <vehicle_interface/msg/vehicle_controller_info.hpp>
#include <vehicle_interface/msg/wheel_state.hpp>
#include <vehicle_interface/msg/yaw_reference.hpp>
#include <vehicle_interface/msg/yawrate_reference.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using control::clip;
using control::ssa;
using Eigen::VectorXd;
using ignition::math::Quaterniond;
using ignition::math::Vector2d;
using ignition::math::Vector3d;
using std::placeholders::_1;
using vehicle_interface::msg::VehicleControllerInfo;
using vehicle_interface::msg::YawReference;

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
    guidance_sub_p = this->create_subscription<vehicle_interface::msg::Guide>(
        "guide", 1,
        std::bind(&VehicleControllerNode::guidance_callback, this, _1));

    yaw_reference_sub_p = this->create_subscription<YawReference>(
        "yaw_reference", 10,
        std::bind(&VehicleControllerNode::yaw_reference_callback, this, _1));

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

    fl_sub_p = this->create_subscription<vehicle_interface::msg::WheelState>(
        "wheel_fl/state", 1,
        [this](vehicle_interface::msg::WheelState::SharedPtr msg) {
          this->fl_state_msg = *msg;
        });
    rl_sub_p = this->create_subscription<vehicle_interface::msg::WheelState>(
        "wheel_rl/state", 1,
        [this](vehicle_interface::msg::WheelState::SharedPtr msg) {
          this->rl_state_msg = *msg;
        });
    rr_sub_p = this->create_subscription<vehicle_interface::msg::WheelState>(
        "wheel_rr/state", 1,
        [this](vehicle_interface::msg::WheelState::SharedPtr msg) {
          this->rr_state_msg = *msg;
        });
    fr_sub_p = this->create_subscription<vehicle_interface::msg::WheelState>(
        "wheel_fr/state", 1,
        [this](vehicle_interface::msg::WheelState::SharedPtr msg) {
          this->fr_state_msg = *msg;
        });

    // this->declare_parameter<double>("update_rate", 20.0);
    this->declare_parameter("update_rate");
    this->get_parameter("update_rate", update_rate);

    timer_p = this->create_wall_timer(
        std::chrono::duration<double>(1 / update_rate),
        std::bind(&VehicleControllerNode::update_command, this));

    this->declare_parameter("pid_active");
    this->get_parameter<bool>("pid_active", pid_active);

    this->declare_parameter("disable_all");
    this->get_parameter<bool>("disable_all", disable_vehicle_controller);

    this->declare_parameter("yaw_source");
    this->get_parameter<std::string>("yaw_source", yaw_source);
    this->declare_parameter("P_yaw");
    this->get_parameter<double>("P_yaw", yaw_pid.P);
    this->declare_parameter("I_yaw");
    this->get_parameter<double>("I_yaw", yaw_pid.I);
    this->declare_parameter("D_yaw");
    this->get_parameter<double>("D_yaw", yaw_pid.D);

    this->declare_parameter("yawrate_ff");
    this->get_parameter<double>("yawrate_ff", yawrate_ff);
    this->declare_parameter("P_yawrate");
    this->get_parameter<double>("P_yawrate", yawrate_pid.P);
    this->declare_parameter("I_yawrate");
    this->get_parameter<double>("I_yawrate", yawrate_pid.I);
    this->declare_parameter("D_yawrate");
    this->get_parameter<double>("D_yawrate", yawrate_pid.D);
    this->declare_parameter("yawrate_sat");
    this->get_parameter<double>("yawrate_sat", yawrate_sat);

    this->declare_parameter("P_speed");
    this->get_parameter<double>("P_speed", speed_pid.P);
    this->declare_parameter("I_speed");
    this->get_parameter<double>("I_speed", speed_pid.I);
    this->declare_parameter("D_speed");
    this->get_parameter<double>("D_speed", speed_pid.D);

    this->declare_parameter("ilqr_4wis_active");
    this->get_parameter<bool>("ilqr_4wis_active", ilqr_4wis_active);

    this->declare_parameter("ilqr_singletrack_active");
    this->get_parameter<bool>("ilqr_singletrack_active",
                              ilqr_singletrack_active);

    double ilqr_cost_x, ilqr_cost_y, ilqr_cost_yaw, ilqr_cost_angular_vel,
        ilqr_cost_steering_angle;
    this->declare_parameter("ilqr_cost_x");
    this->get_parameter<double>("ilqr_cost_x", ilqr_cost_x);
    this->declare_parameter("ilqr_cost_y");
    this->get_parameter<double>("ilqr_cost_y", ilqr_cost_y);
    this->declare_parameter("ilqr_cost_yaw");
    this->get_parameter<double>("ilqr_cost_yaw", ilqr_cost_yaw);
    this->declare_parameter("ilqr_cost_angular_vel");
    this->get_parameter<double>("ilqr_cost_angular_vel", ilqr_cost_angular_vel);
    this->declare_parameter("ilqr_cost_steering_angle");
    this->get_parameter<double>("ilqr_cost_steering_angle",
                                ilqr_cost_steering_angle);
    int ilqr_trajectory_length;
    this->declare_parameter("ilqr_trajectory_length");
    this->get_parameter<int>("ilqr_trajectory_length", ilqr_trajectory_length);

    // iLQR for 4WIS no slip model
    dynsys_4wis_p = std::make_shared<control::dynamics::NoSlip4WISSystem>(
        cg_to_front, cg_to_rear, front_width, rear_width, wheel_radius, 3.0,
        3.0);
    VectorXd cost_states(dynsys_4wis_p->number_of_states());
    cost_states.fill(0.0);
    cost_states(0) = ilqr_cost_x;
    cost_states(1) = ilqr_cost_y;
    cost_states(2) = ilqr_cost_yaw;
    VectorXd cost_states_final = cost_states;
    VectorXd cost_inputs(dynsys_4wis_p->number_of_inputs());
    cost_inputs(0) = ilqr_cost_angular_vel;
    cost_inputs(1) = ilqr_cost_angular_vel;
    cost_inputs(2) = ilqr_cost_angular_vel;
    cost_inputs(3) = ilqr_cost_angular_vel;
    cost_inputs(4) = ilqr_cost_steering_angle;
    cost_inputs(5) = ilqr_cost_steering_angle;
    cost_inputs(6) = ilqr_cost_steering_angle;
    cost_inputs(7) = ilqr_cost_steering_angle;
    std::vector<VectorXd> input_sequence;
    input_sequence.resize(ilqr_trajectory_length);
    for (int i = 0; i < ilqr_trajectory_length; i++) {
      input_sequence[i].setRandom(dynsys_4wis_p->number_of_inputs());
    }
    const double stepsize = 1 / update_rate;
    ilqr_4wis = std::make_shared<IterativeLQR>(dynsys_4wis_p, cost_states,
                                               cost_states_final, cost_inputs,
                                               input_sequence, stepsize);

    // iLQR for singletrack model
    // dynsys_singletrack_p = std::make_shared<SingletrackKinematicSystem>(
    //     cg_to_front, cg_to_rear, wheel_radius, 3.0, 3.0);
    // cost_states.setZero(dynsys_singletrack_p->number_of_states());
    // cost_states(0) = ilqr_cost_x;
    // cost_states(1) = ilqr_cost_y;
    // cost_states(2) = ilqr_cost_yaw;
    // cost_states_final = cost_states;
    // cost_inputs.setZero(dynsys_singletrack_p->number_of_inputs());
    // cost_inputs(0) = ilqr_cost_angular_vel;
    // cost_inputs(1) = ilqr_cost_angular_vel;
    // cost_inputs(2) = ilqr_cost_steering_angle;
    // cost_inputs(3) = ilqr_cost_steering_angle;
    // input_sequence.resize(ilqr_trajectory_length);
    // for (int i = 0; i < ilqr_trajectory_length; i++) {
    //   input_sequence[i].setRandom(dynsys_singletrack_p->number_of_inputs());
    // }
    // ilqr_singletrack = std::make_shared<IterativeLQR>(
    //     dynsys_singletrack_p, cost_states, cost_states_final, cost_inputs,
    //     input_sequence, stepsize);
    //
  }

 private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_p;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_p;
  rclcpp::Subscription<vehicle_interface::msg::DriveMode>::SharedPtr
      reference_sub_p;
  rclcpp::Subscription<vehicle_interface::msg::Guide>::SharedPtr guidance_sub_p;
  rclcpp::Subscription<vehicle_interface::msg::YawReference>::SharedPtr
      yaw_reference_sub_p;

  rclcpp::Publisher<VehicleControllerInfo>::SharedPtr info_pub_p;
  VehicleControllerInfo info_msg;

  rclcpp::Publisher<vehicle_interface::msg::WheelState>::SharedPtr fl_pub_p,
      rl_pub_p, rr_pub_p, fr_pub_p;
  vehicle_interface::msg::WheelState fl_msg, rl_msg, rr_msg, fr_msg;

  rclcpp::Subscription<vehicle_interface::msg::WheelState>::SharedPtr fl_sub_p,
      rl_sub_p, rr_sub_p, fr_sub_p;
  vehicle_interface::msg::WheelState fl_state_msg, rl_state_msg, rr_state_msg,
      fr_state_msg;

  visualization_msgs::msg::MarkerArray controller_markers_msg;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      controller_markers_pub_p;

  rclcpp::TimerBase::SharedPtr timer_p;

  geometry_msgs::msg::PoseStamped::SharedPtr pose_p;
  geometry_msgs::msg::TwistStamped::SharedPtr twist_p;
  vehicle_interface::msg::DriveMode::SharedPtr reference_p;
  vehicle_interface::msg::Guide::SharedPtr guidance_p;
  vehicle_interface::msg::YawReference::SharedPtr yaw_reference_p;

  static constexpr double PI_HALF = 1.57079632679;
  static constexpr double cg_to_front = 0.2, cg_to_rear = 3.0,
                          front_width = 2.0, rear_width = 2.0;
  static constexpr double wheel_radius = 0.505;

  PID speed_pid;
  PID yaw_pid;
  PID yawrate_pid;
  double yawrate_ff;
  double yawrate_sat;

  // Parameters
  bool pid_active;
  bool ilqr_4wis_active;
  bool ilqr_singletrack_active;
  bool disable_vehicle_controller;
  std::string yaw_source;
  double update_rate;

  // Reference
  rclcpp::Time time_now;

  // Iterative LQR variables
  std::shared_ptr<control::dynamics::DynamicalSystem> dynsys_4wis_p;
  std::shared_ptr<IterativeLQR> ilqr_4wis;
  std::shared_ptr<control::dynamics::DynamicalSystem> dynsys_singletrack_p;
  std::shared_ptr<IterativeLQR> ilqr_singletrack;

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

  // void ilqr() {}
  Vector2d compute_icr(const Vector2d &velocity_body, double yawrate) {
    return Vector2d(-velocity_body.Y() / yawrate, velocity_body.X() / yawrate);
  }

  void yawrate_course_noslip_transformation() {
    info_msg.sideslip_reference = info_msg.course_reference - info_msg.yaw;

    const Vector3d pos_fl(cg_to_front, front_width / 2, 0);
    const Vector3d pos_rl(-cg_to_rear, rear_width / 2, 0);
    const Vector3d pos_rr(-cg_to_rear, -rear_width / 2, 0);
    const Vector3d pos_fr(cg_to_front, -front_width / 2, 0);

    // speed_error = (1 - braking_factor) * speed - speed;
    const double &sd = info_msg.speed_command;
    const double &betad = info_msg.sideslip_reference;
    const double &dpsid = info_msg.yawrate_command;

    const Vector3d velocity_body_desired(sd * cos(betad), sd * sin(betad), 0);

    const Vector3d z_axis(0, 0, 1);

    const Vector3d vel_fl =
        velocity_body_desired + dpsid * z_axis.Cross(pos_fl);
    const Vector3d vel_rl =
        velocity_body_desired + dpsid * z_axis.Cross(pos_rl);
    const Vector3d vel_rr =
        velocity_body_desired + dpsid * z_axis.Cross(pos_rr);
    const Vector3d vel_fr =
        velocity_body_desired + dpsid * z_axis.Cross(pos_fr);

    const double speed_fl = vel_fl.Length();
    const double speed_rl = vel_rl.Length();
    const double speed_rr = vel_rr.Length();
    const double speed_fr = vel_fr.Length();

    fl_msg.angular_velocity = speed_fl / wheel_radius;
    fl_msg.steering_angle = atan2(vel_fl.Y(), vel_fl.X());
    fl_msg.steering_angle_rate = 0.0;

    rl_msg.angular_velocity = speed_rl / wheel_radius;
    rl_msg.steering_angle = atan2(vel_rl.Y(), vel_rl.X());
    rl_msg.steering_angle_rate = 0.0;

    rr_msg.angular_velocity = speed_rr / wheel_radius;
    rr_msg.steering_angle = atan2(vel_rr.Y(), vel_rr.X());
    rr_msg.steering_angle_rate = 0.0;

    fr_msg.angular_velocity = speed_fr / wheel_radius;
    fr_msg.steering_angle = atan2(vel_fr.Y(), vel_fr.X());
    fr_msg.steering_angle_rate = 0.0;
  }

  void yaw_control() {
    if (yaw_source == YawReference::YAWRATE) {
      info_msg.yawrate_command =
          yawrate_ff * info_msg.yawrate_reference +
          yawrate_pid.update(info_msg.yawrate_error, time_now);
    }
    if (yaw_source != YawReference::YAWRATE) {
      info_msg.yawrate_reference =
          clip(yaw_pid.update(info_msg.yaw_error, time_now), -yawrate_sat,
               yawrate_sat);
      info_msg.yawrate_command = info_msg.yawrate_reference;
    }

    info_msg.yawrate_error = info_msg.yawrate_reference - info_msg.yawrate;
  }
  void speed_control() {
    info_msg.speed_command = info_msg.speed_reference +
                             speed_pid.update(info_msg.speed_error, time_now);
  }

  void update_command() {
    time_now = this->get_clock()->now();
    info_msg.header.stamp = time_now;
    //
    info_msg.guidance_ready = false;
    if (guidance_p && pose_p && twist_p) {
      info_msg.guidance_ready = true;
      info_msg.speed_error = info_msg.speed_reference - info_msg.speed;
      info_msg.course_error = ssa(info_msg.course_reference - info_msg.course);
      // info_msg.yawrate_error = info_msg.yawrate_reference - info_msg.yawrate;
    }

    info_msg.yaw_ready = false;
    if (yaw_reference_p && pose_p && twist_p) {
      info_msg.yaw_ready = true;
      info_msg.yaw_error = ssa(info_msg.yaw_reference - info_msg.yaw);
    }

    // Do control if either of the reference sources are ready
    if (info_msg.guidance_ready || info_msg.yaw_ready) {
      yaw_control();
      speed_control();

      yawrate_course_noslip_transformation();
    }
    // if (pose_p && twist_p) {
    //   using Marker = visualization_msgs::msg::Marker;
    //   controller_markers_msg.markers[0].header.frame_id = "map";
    //   controller_markers_msg.markers[0].header.stamp =
    //   this->get_clock()->now(); controller_markers_msg.markers[0].ns =
    //   "vehicle"; controller_markers_msg.markers[0].id = 0;
    //   controller_markers_msg.markers[0].type = Marker::ARROW;
    //   controller_markers_msg.markers[0].action = Marker::ADD;
    //   controller_markers_msg.markers[0].color.r = 1.0;
    //   controller_markers_msg.markers[0].color.g = 1.0;
    //   controller_markers_msg.markers[0].color.b = 1.0;
    //   controller_markers_msg.markers[0].color.a = 1.0;
    //   controller_markers_msg.markers[0].scale.x = 0.1;
    //   controller_markers_msg.markers[0].scale.y = 0.2;
    //   controller_markers_msg.markers[0].scale.z = 0.0;
    //   controller_markers_msg.markers[0].points.resize(2);
    //   controller_markers_msg.markers[0].points[0] = pose_p->pose.position;
    //   controller_markers_msg.markers[0].points[1].x = path_position.X();
    //   controller_markers_msg.markers[0].points[1].y = path_position.Y();
    //   controller_markers_msg.markers[0].points[1].z =
    //   pose_p->pose.position.z;

    //   controller_markers_pub_p->publish(controller_markers_msg);
    // }

    info_pub_p->publish(info_msg);

    if (!disable_vehicle_controller) {
      fl_pub_p->publish(fl_msg);
      rl_pub_p->publish(rl_msg);
      rr_pub_p->publish(rr_msg);
      fr_pub_p->publish(fr_msg);
    }
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
    const auto &q = pose_p->pose.orientation;
    const auto vehicle_quat = Quaterniond(q.w, q.x, q.y, q.z);
    info_msg.yaw = vehicle_quat.Yaw();
    // position = Vector2d(pose_p->pose.position.x, pose_p->pose.position.y);
    RCLCPP_INFO_ONCE(this->get_logger(), "pose message recieved");
  }

  void twist_callback(geometry_msgs::msg::TwistStamped::SharedPtr msg_p) {
    twist_p = msg_p;

    const auto velocity_body =
        Vector2d(twist_p->twist.linear.x, twist_p->twist.linear.y);

    info_msg.sideslip = atan2(velocity_body.Y(), velocity_body.X());
    info_msg.course = info_msg.yaw + info_msg.sideslip;
    info_msg.speed = velocity_body.Length();
    info_msg.yawrate = twist_p->twist.angular.z;

    // update icr
    const Vector2d icr = compute_icr(velocity_body, info_msg.yawrate);
    info_msg.icr.x = icr.X();
    info_msg.icr.y = icr.Y();
    const double icr_radius = icr.Length();
    info_msg.curvature = 1 / icr_radius;

    RCLCPP_INFO_ONCE(this->get_logger(), "twist message recieved");
  }

  void reference_callback(vehicle_interface::msg::DriveMode::SharedPtr msg_p) {
    reference_p = msg_p;
    RCLCPP_INFO_ONCE(this->get_logger(), "reference message recieved");
  }

  void guidance_callback(vehicle_interface::msg::Guide::SharedPtr msg_p) {
    guidance_p = msg_p;
    info_msg.course_reference = msg_p->course;
    info_msg.courserate_reference = msg_p->courserate;
    info_msg.speed_reference = msg_p->speed;
    info_msg.guidance_ready = true;
    RCLCPP_INFO_ONCE(this->get_logger(), "guidance message recieved");
  }

  void yaw_reference_callback(
      vehicle_interface::msg::YawReference::SharedPtr msg_p) {
    if (msg_p->source == yaw_source) {
      yaw_reference_p = msg_p;

      info_msg.yaw_reference = msg_p->yaw;
      info_msg.yawrate_reference = msg_p->yawrate;
      info_msg.yaw_ready = true;
    }
  }
};

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(VehicleControllerNode)