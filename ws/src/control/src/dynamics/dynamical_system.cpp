#include <control/dynamics/dynamical_system.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

using Eigen::MatrixXd;
using Eigen::VectorXd;

#include <iostream>

void DynamicalSystem::discretize(const VectorXd& states, const VectorXd& inputs,
                                 double stepsize, MatrixXd& Ak, MatrixXd& Bk) {
  const int nx = number_of_states();
  const int nu = number_of_inputs();
  MatrixXd A(nx, nx);
  MatrixXd B(nx, nu);
  linearize(states, inputs, A, B);
  MatrixXd F(nx + nu, nx + nu);
  F.setZero();
  F.block(0, 0, nx, nx) = A * stepsize;
  F.block(0, nx, nx, nu) = B * stepsize;
  const MatrixXd V = F.exp();

  Ak = V.block(0, 0, nx, nx);
  Bk = V.block(0, nx, nx, nu);

  assert(Ak.rows() == nx && Ak.cols() == nx);
  assert(Bk.rows() == nx && Bk.cols() == nu);
}

VectorXd DynamicalSystem::step(const VectorXd& states, const VectorXd& inputs,
                               double stepsize) {
  // Runge-Kutta 4
  const VectorXd& u = inputs;
  const VectorXd& x = states;
  const double& h = stepsize;

  const VectorXd k1 = derivatives(x, u);
  const VectorXd k2 = derivatives(x + h / 2 * k1, u);
  const VectorXd k3 = derivatives(x + h / 2 * k2, u);
  const VectorXd k4 = derivatives(x + h * k3, u);

  return x + (h / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
}

void DynamicalSystem::linearize(const Eigen::VectorXd& states,
                                const Eigen::VectorXd& inputs,
                                Eigen::MatrixXd& A, Eigen::MatrixXd& B) {
  // Numerical linearization

  const int NX = number_of_states();
  const int NU = number_of_inputs();
  A.setZero(NX, NX);
  B.setZero(NX, NU);

  const double eps = 0.001;
  for (int col = 0; col < NX; col++) {
    VectorXd step(NX);
    step.setZero();
    step(col) = eps;
    VectorXd dstates = (derivatives(states + step, inputs) -
                        derivatives(states - step, inputs)) /
                       (2 * eps);
    A.block(0, col, NX, 1) = dstates;
  }

  for (int col = 0; col < NU; col++) {
    VectorXd step(NU);
    step.setZero();
    step(col) = eps;
    VectorXd dinputs = (derivatives(states, inputs + step) -
                        derivatives(states, inputs - step)) /
                       (2 * eps);
    B.block(0, col, NX, 1) = dinputs;
  }
}