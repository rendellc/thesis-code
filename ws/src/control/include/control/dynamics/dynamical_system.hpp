#pragma once

#include <eigen3/Eigen/Dense>

// using Eigen::MatrixXd;
// using Eigen::Vector2d;
//
using Eigen::MatrixXd;
using Eigen::VectorXd;

class DynamicalSystem {
 public:
  const int number_of_states;
  const int number_of_inputs;

  DynamicalSystem() = delete;
  DynamicalSystem(int number_of_states, int number_of_inputs);
  virtual ~DynamicalSystem();

  virtual VectorXd derivatives(const VectorXd& states,
                               const VectorXd& inputs) = 0;

  virtual void linearize(const VectorXd& states, const VectorXd& inputs,
                         MatrixXd& A, MatrixXd& B) = 0;

  void discretize(const VectorXd& states, const VectorXd& inputs,
                  double stepsize, MatrixXd& Ak, MatrixXd& Bk);

  VectorXd step(const VectorXd& states, const VectorXd& inputs,
                double stepsize);
};