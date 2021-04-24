#include <control/dynamics/dynamical_system.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

DynamicalSystem::DynamicalSystem(int number_of_states, int number_of_inputs)
    : number_of_states(number_of_states), number_of_inputs(number_of_inputs) {}

void DynamicalSystem::discretize(const VectorXd& states, const VectorXd& inputs,
                                 double stepsize, MatrixXd& Ak, MatrixXd& Bk) {
  const int& nx = number_of_states;
  const int& nu = number_of_inputs;
  MatrixXd A(nx, nx);
  MatrixXd B(nx, nu);
  linearize(states, inputs, A, B);
  MatrixXd F = MatrixXd::Zero(nx + nu, nx + nu);
  F.block(0, 0, nx, nx) = A * stepsize;
  F.block(0, nx, nx, nu) = (B * stepsize);
  const MatrixXd V = F.exp();

  Ak = V.block(0, 0, nx, nx);
  Bk = V.block(0, nx, nx, nu);
}

VectorXd DynamicalSystem::step(const VectorXd& states, const VectorXd& inputs,
                               double stepsize) {
  // Runge-Kutta 4
  const auto& u = inputs;
  const auto& x = states;
  const double& h = stepsize;

  const auto k1 = derivatives(x, u);
  const auto k2 = derivatives(x + h / 2 * k1, u);
  const auto k3 = derivatives(x + h / 2 * k2, u);
  const auto k4 = derivatives(x + h * k3, u);

  return x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
}

DynamicalSystem::~DynamicalSystem() {}