#pragma once

#include <eigen3/Eigen/Dense>

namespace control {
namespace dynamics {

class DynamicalSystem {
 public:
  DynamicalSystem() = default;
  virtual ~DynamicalSystem() = default;

  virtual Eigen::VectorXd derivatives(const Eigen::VectorXd& states,
                                      const Eigen::VectorXd& inputs) = 0;

  virtual void linearize(const Eigen::VectorXd& states,
                         const Eigen::VectorXd& inputs, Eigen::MatrixXd& A,
                         Eigen::MatrixXd& B);

  void discretize(const Eigen::VectorXd& states, const Eigen::VectorXd& inputs,
                  double stepsize, Eigen::MatrixXd& Ak, Eigen::MatrixXd& Bk);

  Eigen::VectorXd step(const Eigen::VectorXd& states,
                       const Eigen::VectorXd& inputs, double stepsize);

  virtual int number_of_states() const = 0;
  virtual int number_of_inputs() const = 0;
};
}  // namespace dynamics
}  // namespace control