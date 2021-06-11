#pragma once

#include <control/dynamics/dynamical_system.hpp>
#include <eigen3/Eigen/Dense>

namespace control {
namespace dynamics {
class SingletrackKinematicSystem : public DynamicalSystem {
  void velocity_and_yawrate(const Eigen::VectorXd& states,
                            Eigen::Vector2d& velocity, double& yawrate) const;

  Eigen::Matrix2d rotation_matrix(double yaw) const;

  static constexpr int NX = 7;
  static constexpr int NU = 4;

  double wheel_radius;
  double length_to_front;
  double length_to_rear;
  double time_constant_omega;
  double time_constant_delta;

  Eigen::Matrix<double, 3, 4> no_slip_matrix;

 public:
  using StateT = Eigen::Matrix<double, NX, 1>;
  using InputT = Eigen::Matrix<double, NU, 1>;

  SingletrackKinematicSystem() = delete;
  SingletrackKinematicSystem(double length_to_front, double length_to_rear,
                             double wheel_radius, double time_constant_omega,
                             double time_constant_delta);
  virtual ~SingletrackKinematicSystem() = default;

  Eigen::VectorXd derivatives(const Eigen::VectorXd& states,
                              const Eigen::VectorXd& inputs) override;
  void linearize(const Eigen::VectorXd& states, const Eigen::VectorXd& inputs,
                 Eigen::MatrixXd& A, Eigen::MatrixXd& B) override;

  int number_of_states() const override;
  int number_of_inputs() const override;
};
}  // namespace dynamics
}  // namespace control