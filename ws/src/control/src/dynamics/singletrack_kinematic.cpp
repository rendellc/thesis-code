#include <control/dynamics/singletrack_kinematic.hpp>
#include <iostream>

using namespace Eigen;

namespace control {
namespace dynamics {

SingletrackKinematicSystem::SingletrackKinematicSystem(
    double length_to_front, double length_to_rear, double wheel_radius,
    double time_constant_omega, double time_constant_delta)
    : wheel_radius(wheel_radius),
      length_to_front(length_to_front),
      length_to_rear(length_to_rear),
      time_constant_omega(time_constant_omega),
      time_constant_delta(time_constant_delta) {
  //
  Matrix<double, 4, 3> A;

  A.block<2, 2>(0, 0).setIdentity();
  A.block<2, 2>(2, 0).setIdentity();

  A(0, 2) = 0;
  A(1, 2) = -length_to_front;
  A(2, 2) = 0;
  A(3, 2) = length_to_rear;

  no_slip_matrix = (A.transpose() * A).inverse() * A.transpose();
}

void SingletrackKinematicSystem::velocity_and_yawrate(const VectorXd& states,
                                                      Vector2d& velocity,
                                                      double& yawrate) const {
  // omega_f 3
  // omega_r 4
  // delta_f 5
  // delta_r 6
  Matrix<double, 4, 1> b;
  b(0) = wheel_radius * cos(states[5]) * states[3];
  b(1) = wheel_radius * sin(states[5]) * states[3];
  b(2) = wheel_radius * cos(states[6]) * states[4];
  b(3) = wheel_radius * sin(states[6]) * states[4];

  Vector3d v_and_dpsi = no_slip_matrix * b;
  velocity(0) = v_and_dpsi(0);
  velocity(1) = v_and_dpsi(1);
  yawrate = v_and_dpsi(2);
}

VectorXd SingletrackKinematicSystem::derivatives(const VectorXd& states,
                                                 const VectorXd& inputs) {
  // State indices
  // - pos idx 0,1
  // - yaw idx 2
  // - omega_f 3
  // - omega_r 4
  // - delta_f 5
  // - delta_r 6

  // Inputs indices
  // - omega_f_c 0
  // - omega_r_c 1
  // - delta_f_c 2
  // - delta_r_c 3

  assert(states.rows() == NX && states.cols() == 1);
  assert(inputs.rows() == NU && inputs.cols() == 1);

  Matrix<double, NX, 1> state_derivatives;

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

  state_derivatives(7) = (inputs(2) - states(5)) / time_constant_delta;
  state_derivatives(8) = (inputs(3) - states(6)) / time_constant_delta;

  return state_derivatives;
}

void SingletrackKinematicSystem::linearize(const VectorXd& states,
                                           const VectorXd& inputs, MatrixXd& A,
                                           MatrixXd& B) {
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

  Matrix<double, 4, 1> db;

  // omega_f
  db.setZero();
  db(0) = wheel_radius * cos(states(5));
  db(1) = wheel_radius * sin(states(5));
  A.block<3, 1>(0, 3) = J * no_slip_matrix * db;

  // omega_r
  db.setZero();
  db(2) = wheel_radius * cos(states(6));
  db(3) = wheel_radius * sin(states(6));
  A.block<3, 1>(0, 4) = J * no_slip_matrix * db;

  // delta_f
  db.setZero();
  db(0) = -wheel_radius * sin(states(5)) * states(3);
  db(1) = wheel_radius * cos(states(5)) * states(3);
  A.block<3, 1>(0, 5) = J * no_slip_matrix * db;

  // delta_r
  db.setZero();
  db(2) = -wheel_radius * sin(states(6)) * states(4);
  db(3) = wheel_radius * cos(states(6)) * states(4);
  A.block<3, 1>(0, 6) = J * no_slip_matrix * db;

  A(3, 3) = -1 / time_constant_omega;
  A(4, 4) = -1 / time_constant_omega;

  A(5, 5) = -1 / time_constant_delta;
  A(6, 6) = -1 / time_constant_delta;

  B(3, 0) = 1 / time_constant_omega;
  B(4, 1) = 1 / time_constant_omega;

  B(5, 2) = 1 / time_constant_delta;
  B(6, 3) = 1 / time_constant_delta;
}

int SingletrackKinematicSystem::number_of_states() const { return NX; }
int SingletrackKinematicSystem::number_of_inputs() const { return NU; }

}  // namespace dynamics
}  // namespace control