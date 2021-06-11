#pragma once
#include <eigen3/Eigen/Dense>

namespace control {
namespace reference_model {
class FirstOrder {
 public:
  FirstOrder();
  FirstOrder(double time_constant, double initial_value);

  void update(double reference, double dt);
  double getReference() const;

  void setTimeConstant(double time_constant);

 private:
  double time_constant;
  double reference;
};

class SecondOrder {
 public:
  SecondOrder();
  SecondOrder(double damping_ratio, double natural_frequency,
              double initial_value, double initial_derivative);

  void update(double reference, double dt);
  double getReference() const;
  double getReferenceDerivative() const;

  void setParameters(double damping_ratio, double natural_frequency);

 private:
  Eigen::Matrix2d A;
  Eigen::Vector2d b;
  Eigen::Vector2d x;
};
}  // namespace reference_model
}  // namespace control
