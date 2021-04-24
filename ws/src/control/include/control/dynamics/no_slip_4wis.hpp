#pragma once

#include <control/dynamics/dynamical_system.hpp>
#include <eigen3/Eigen/Dense>

class NoSlip4WISSystem : public DynamicalSystem {
  std::tuple<Eigen::Vector2d, double> velocity_and_yawrate(
      const Eigen::VectorXd& states) const;

  Eigen::Matrix2d rotation_matrix(double yaw) const;

  double wheel_radius;
  double time_constant_omega;
  double time_constant_delta;

  Eigen::Vector2d wheel_position_fl;
  Eigen::Vector2d wheel_position_rl;
  Eigen::Vector2d wheel_position_rr;
  Eigen::Vector2d wheel_position_fr;

  MatrixXd no_slip_matrix;

 public:
  NoSlip4WISSystem() = delete;
  NoSlip4WISSystem(double length_to_front, double length_to_rear,
                   double front_width, double rear_width, double wheel_radius,
                   double time_constant_omega, double time_constant_delta);
  virtual ~NoSlip4WISSystem();

  VectorXd derivatives(const Eigen::VectorXd& states,
                       const Eigen::VectorXd& inputs) override;
  void linearize(const Eigen::VectorXd& states, const Eigen::VectorXd& inputs,
                 Eigen::MatrixXd& A, Eigen::MatrixXd& B) override;
};