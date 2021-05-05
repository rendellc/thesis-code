#include <control/dynamics/first_order.hpp>

using namespace Eigen;

FirstOrder::FirstOrder(double gain) : gain(gain) {}

Eigen::VectorXd FirstOrder::derivatives(const VectorXd& states,
                                        const VectorXd& inputs) {
  VectorXd state_derivatives(NX);
  state_derivatives(0) = (gain * states(0)) + inputs(0);
  state_derivatives(1) = states(1);

  return state_derivatives;
}

void FirstOrder::linearize(const VectorXd& states, const VectorXd& inputs,
                           MatrixXd& A, MatrixXd& B) {
  A.setZero(NX, NX);
  A(0, 0) = gain;
  A(1, 0) = 1;

  B.setZero(NX, NU);
  B(0, 0) = 1.0;
}

int FirstOrder::number_of_states() const { return NX; }
int FirstOrder::number_of_inputs() const { return NU; }