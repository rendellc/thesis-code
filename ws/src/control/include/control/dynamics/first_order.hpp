#pragma once

#include <control/dynamics/dynamical_system.hpp>
#include <eigen3/Eigen/Dense>

namespace control {
namespace dynamics {

class FirstOrder : public DynamicalSystem {
  static constexpr int NX = 2;
  static constexpr int NU = 1;

  const double gain;

 public:
  FirstOrder() = delete;
  FirstOrder(double gain);
  virtual ~FirstOrder() = default;

  Eigen::VectorXd derivatives(const Eigen::VectorXd& states,
                              const Eigen::VectorXd& inputs) override;
  void linearize(const Eigen::VectorXd& states, const Eigen::VectorXd& inputs,
                 Eigen::MatrixXd& A, Eigen::MatrixXd& B) override;

  int number_of_states() const override;
  int number_of_inputs() const override;
};
}  // namespace dynamics
}  // namespace control