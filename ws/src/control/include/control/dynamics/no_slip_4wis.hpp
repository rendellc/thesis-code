#pragma once

#include <control/dynamics/dynamical_system.hpp>
#include <eigen3/Eigen/Dense>

namespace control {
namespace dynamics {

class NoSlip4WISSystem : public DynamicalSystem {
  void velocity_and_yawrate(const Eigen::VectorXd& states,
                            Eigen::Vector2d& velocity, double& yawrate) const;

  Eigen::Matrix2d rotation_matrix(double yaw) const;

  static constexpr int NX = 11;
  static constexpr int NU = 8;

  double wheel_radius;
  double time_constant_omega;
  double time_constant_delta;

  Eigen::Vector2d wheel_position_fl;
  Eigen::Vector2d wheel_position_rl;
  Eigen::Vector2d wheel_position_rr;
  Eigen::Vector2d wheel_position_fr;

  Eigen::Matrix<double, 3, 8> no_slip_matrix;

 public:
  using StateT = Eigen::Matrix<double, 11, 1>;
  using InputT = Eigen::Matrix<double, 8, 1>;

  NoSlip4WISSystem() = delete;
  NoSlip4WISSystem(double length_to_front, double length_to_rear,
                   double front_width, double rear_width, double wheel_radius,
                   double time_constant_omega, double time_constant_delta);
  virtual ~NoSlip4WISSystem();

  Eigen::VectorXd derivatives(const Eigen::VectorXd& states,
                              const Eigen::VectorXd& inputs) override;
  void linearize(const Eigen::VectorXd& states, const Eigen::VectorXd& inputs,
                 Eigen::MatrixXd& A, Eigen::MatrixXd& B) override;

  int number_of_states() const override;
  int number_of_inputs() const override;
};
}  // namespace dynamics
}  // namespace control