#pragma once

#include <control/dynamics/dynamical_system.hpp>
#include <eigen3/Eigen/Dense>

namespace control {
namespace dynamics {
class InvertedPendulum : public DynamicalSystem {
  static constexpr int NX = 2;
  static constexpr int NU = 1;

  const double m;
  const double mu;
  const double l;
  const double g;

 public:
  InvertedPendulum() = delete;
  InvertedPendulum(double mass, double friction, double length, double gravity);
  virtual ~InvertedPendulum() = default;

  Eigen::VectorXd derivatives(const Eigen::VectorXd& states,
                              const Eigen::VectorXd& inputs) override;

  int number_of_states() const override;
  int number_of_inputs() const override;
};
}  // namespace dynamics
}  // namespace control