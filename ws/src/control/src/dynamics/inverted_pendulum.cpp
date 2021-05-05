#include <control/dynamics/inverted_pendulum.hpp>

InvertedPendulum::InvertedPendulum(double mass, double friction, double length,
                                   double gravity)
    : m(mass), mu(friction), l(length), g(gravity) {}

Eigen::VectorXd InvertedPendulum::derivatives(const Eigen::VectorXd& states,
                                              const Eigen::VectorXd& inputs) {
  // State indices:
  // 0, x , position of cart
  // 1, xdot, velocity of cart
  // 2, theta, angle of pendulum (down = 0)
  // 3, thetadot, angular velocity of pendulum
  Eigen::Matrix<double, NX, 1> state_derivatives;
  state_derivatives(0) = states(1);
  state_derivatives(1) = g / l * sin(states(0)) - mu / (m * l * l) * states(1) +
                         1 / (m * l * l) * inputs(0);

  return state_derivatives;
}

int InvertedPendulum::number_of_states() const { return NX; }
int InvertedPendulum::number_of_inputs() const { return NU; }