#include <control/reference_model.hpp>

namespace control {
namespace reference_model {
FirstOrder::FirstOrder() : FirstOrder(1, 0) {}
FirstOrder::FirstOrder(double time_constant, double initial_value)
    : time_constant(time_constant), reference(initial_value) {}

void FirstOrder::update(double new_reference, double dt) {
  reference += dt / time_constant * (new_reference - reference);
}
double FirstOrder::getReference() const { return reference; }

void FirstOrder::setTimeConstant(double new_time_constant) {
  time_constant = new_time_constant;
}

SecondOrder::SecondOrder() : SecondOrder(0.9, 2.0, 0, 0) {}
SecondOrder::SecondOrder(double damping_ratio, double natural_frequency,
                         double initial_value, double initial_derivative) {
  A(0, 0) = 0;
  A(0, 1) = 1;
  A(1, 0) = -natural_frequency * natural_frequency;
  A(1, 1) = -2 * damping_ratio * natural_frequency;
  b(1) = natural_frequency * natural_frequency;
  x << initial_value, initial_derivative;
}

void SecondOrder::update(double reference, double dt) {
  const auto I = Eigen::Matrix2d::Identity(2, 2);
  x = (I + dt * A) * x + dt * b * reference;
}
double SecondOrder::getReference() const { return x(0); }
double SecondOrder::getReferenceDerivative() const { return x(1); }

void SecondOrder::setParameters(double damping_ratio,
                                double natural_frequency) {
  A(0, 0) = 0;
  A(0, 1) = 1;
  A(1, 0) = -natural_frequency * natural_frequency;
  A(1, 1) = -2 * damping_ratio * natural_frequency;
  b(1) = natural_frequency * natural_frequency;
}
}  // namespace reference_model
}  // namespace control
