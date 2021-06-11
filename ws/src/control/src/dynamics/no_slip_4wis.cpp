#include <control/dynamics/no_slip_4wis.hpp>
#include <iostream>

using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace control {
namespace dynamics {

NoSlip4WISSystem::NoSlip4WISSystem(double length_to_front,
                                   double length_to_rear, double front_width,
                                   double rear_width, double wheel_radius,
                                   double time_constant_omega,
                                   double time_constant_delta)
    : wheel_radius(wheel_radius),
      time_constant_omega(time_constant_omega),
      time_constant_delta(time_constant_delta) {
  wheel_position_fl << length_to_front, front_width / 2;
  wheel_position_rl << -length_to_rear, rear_width / 2;
  wheel_position_rr << -length_to_rear, -rear_width / 2;
  wheel_position_fr << length_to_front, -front_width / 2;
  //
  MatrixXd A(8, 3);
  // Matrix<double, 8, 3> A;
  A.setZero();

  A.block<2, 2>(0, 0).setIdentity();
  A.block<2, 2>(2, 0).setIdentity();
  A.block<2, 2>(4, 0).setIdentity();
  A.block<2, 2>(6, 0).setIdentity();

  A(0, 2) = -wheel_position_fl(1);
  A(1, 2) = wheel_position_fl(0);
  A(2, 2) = -wheel_position_rl(1);
  A(3, 2) = wheel_position_rl(0);
  A(4, 2) = -wheel_position_rr(1);
  A(5, 2) = wheel_position_rr(0);
  A(6, 2) = -wheel_position_fr(1);
  A(7, 2) = wheel_position_fr(0);

  no_slip_matrix = (A.transpose() * A).inverse() * A.transpose();
}

NoSlip4WISSystem::~NoSlip4WISSystem() {}

Matrix2d NoSlip4WISSystem::rotation_matrix(double yaw) const {
  Matrix2d R;
  R(0, 0) = cos(yaw);
  R(0, 1) = -sin(yaw);
  R(1, 0) = sin(yaw);
  R(1, 1) = cos(yaw);
  return R;
}

void NoSlip4WISSystem::velocity_and_yawrate(const VectorXd& states,
                                            Vector2d& velocity,
                                            double& yawrate) const {
  // omega_fl 3
  // omega_rl 4
  // omega_rr 5
  // omega_fr 6
  // delta_fl 7
  // delta_rl 8
  // delta_rr 9
  // delta_fr 10
  VectorXd b(8);
  b(0) = wheel_radius * cos(states[7]) * states[3];
  b(1) = wheel_radius * sin(states[7]) * states[3];
  b(2) = wheel_radius * cos(states[8]) * states[4];
  b(3) = wheel_radius * sin(states[8]) * states[4];
  b(4) = wheel_radius * cos(states[9]) * states[5];
  b(5) = wheel_radius * sin(states[9]) * states[5];
  b(6) = wheel_radius * cos(states[10]) * states[6];
  b(7) = wheel_radius * sin(states[10]) * states[6];

  Vector3d v_and_dpsi = no_slip_matrix * b;
  velocity(0) = v_and_dpsi(0);
  velocity(1) = v_and_dpsi(1);
  yawrate = v_and_dpsi(2);
}

VectorXd NoSlip4WISSystem::derivatives(const VectorXd& states,
                                       const VectorXd& inputs) {
  // State indices
  // - pos idx 0,1
  // - yaw idx 2
  // - omega_fl 3
  // - omega_rl 4
  // - omega_rr 5
  // - omega_fr 6
  // - delta_fl 7
  // - delta_rl 8
  // - delta_rr 9
  // - delta_fr 10

  // Inputs indices
  // - omega_fl_c 0
  // - omega_rl_c 1
  // - omega_rr_c 2
  // - omega_fr_c 3
  // - delta_fl_c 4
  // - delta_rl_c 5
  // - delta_rr_c 6
  // - delta_fr_c 7

  assert(states.rows() == NX && states.cols() == 1);
  assert(inputs.rows() == NU && inputs.cols() == 1);

  VectorXd state_derivatives(NX);

  const double& yaw = states(2);
  Vector2d vel_body;
  double yawrate;
  velocity_and_yawrate(states, vel_body, yawrate);

  Matrix2d body_to_world = rotation_matrix(yaw);
  const Vector2d vel_world = body_to_world * vel_body;
  state_derivatives(0) = vel_world(0);
  state_derivatives(1) = vel_world(1);
  state_derivatives(2) = yawrate;

  state_derivatives(3) = (inputs(0) - states(3)) / time_constant_omega;
  state_derivatives(4) = (inputs(1) - states(4)) / time_constant_omega;
  state_derivatives(5) = (inputs(2) - states(5)) / time_constant_omega;
  state_derivatives(6) = (inputs(3) - states(6)) / time_constant_omega;

  state_derivatives(7) = (inputs(4) - states(7)) / time_constant_delta;
  state_derivatives(8) = (inputs(5) - states(8)) / time_constant_delta;
  state_derivatives(9) = (inputs(6) - states(9)) / time_constant_delta;
  state_derivatives(10) = (inputs(7) - states(10)) / time_constant_delta;

  return state_derivatives;
}

void NoSlip4WISSystem::linearize(const VectorXd& states, const VectorXd& inputs,
                                 MatrixXd& A, MatrixXd& B) {
  A.setZero(NX, NX);
  B.setZero(NX, NU);

  const double PI_HALF = atan2(1, 0);

  Vector2d vel_body;
  double yawrate;
  velocity_and_yawrate(states, vel_body, yawrate);

  Vector3d nu;
  nu(0) = vel_body(0);
  nu(1) = vel_body(1);
  nu(2) = yawrate;

  const double yaw = states(2);
  const Matrix2d body_to_world = rotation_matrix(yaw);
  const Matrix2d body_to_world_derivative =
      yawrate * rotation_matrix(yaw + PI_HALF);

  Matrix3d J;
  J.fill(0.0);
  J.block<2, 2>(0, 0) = body_to_world;
  J(2, 2) = 1;

  A.block<2, 1>(0, 2) = body_to_world_derivative * vel_body;

  VectorXd db(8);

  // omega_fl
  db.setZero();
  db(0) = wheel_radius * cos(states(7));
  db(1) = wheel_radius * sin(states(7));
  A.block<3, 1>(0, 3) = J * no_slip_matrix * db;

  // omega_rl
  db.setZero();
  db(2) = wheel_radius * cos(states(8));
  db(3) = wheel_radius * sin(states(8));
  A.block<3, 1>(0, 4) = J * no_slip_matrix * db;

  // omega_rr
  db.setZero();
  db(4) = wheel_radius * cos(states(9));
  db(5) = wheel_radius * sin(states(9));
  A.block<3, 1>(0, 6) = J * no_slip_matrix * db;

  // omega_fr
  db.setZero();
  db(6) = wheel_radius * cos(states(10));
  db(7) = wheel_radius * sin(states(10));
  A.block<3, 1>(0, 6) = J * no_slip_matrix * db;

  // delta_fl
  db.setZero();
  db(0) = -wheel_radius * sin(states(7)) * states(3);
  db(1) = wheel_radius * cos(states(7)) * states(3);
  A.block<3, 1>(0, 7) = J * no_slip_matrix * db;

  // delta_rl
  db.setZero();
  db(2) = -wheel_radius * sin(states(8)) * states(4);
  db(3) = wheel_radius * cos(states(8)) * states(4);
  A.block<3, 1>(0, 8) = J * no_slip_matrix * db;

  // delta_rr
  db.setZero();
  db(4) = -wheel_radius * sin(states(9)) * states(5);
  db(5) = wheel_radius * cos(states(9)) * states(5);
  A.block<3, 1>(0, 9) = J * no_slip_matrix * db;

  // delta_fr
  db.setZero();
  db(6) = -wheel_radius * sin(states(10)) * states(6);
  db(7) = wheel_radius * cos(states(10)) * states(6);
  A.block<3, 1>(0, 10) = J * no_slip_matrix * db;

  A(3, 3) = -1 / time_constant_omega;
  A(4, 4) = -1 / time_constant_omega;
  A(5, 5) = -1 / time_constant_omega;
  A(6, 6) = -1 / time_constant_omega;

  A(7, 7) = -1 / time_constant_delta;
  A(8, 8) = -1 / time_constant_delta;
  A(9, 9) = -1 / time_constant_delta;
  A(10, 10) = -1 / time_constant_delta;

  B(3, 0) = 1 / time_constant_omega;
  B(4, 1) = 1 / time_constant_omega;
  B(5, 2) = 1 / time_constant_omega;
  B(6, 3) = 1 / time_constant_omega;

  B(7, 4) = 1 / time_constant_delta;
  B(8, 5) = 1 / time_constant_delta;
  B(9, 6) = 1 / time_constant_delta;
  B(10, 7) = 1 / time_constant_delta;
}

int NoSlip4WISSystem::number_of_states() const { return NX; }
int NoSlip4WISSystem::number_of_inputs() const { return NU; }

}  // namespace dynamics
}  // namespace control