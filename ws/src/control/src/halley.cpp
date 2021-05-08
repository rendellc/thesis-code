#include <cmath>
#include <control/halley.hpp>

namespace control {
double halleys_method(std::function<double(double)> f,
                      std::function<double(double)> df,
                      std::function<double(double)> ddf, double initial_value,
                      double threshold, int max_iterations) {
  int iteration = 0;
  double x = initial_value;
  while (fabs(f(x)) > threshold && iteration < max_iterations) {
    const double y = f(x);
    const double dy = df(x);
    const double ddy = ddf(x);

    x = x - 2 * y * dy / (2 * dy * dy - y * ddy);
    iteration++;
  }

  return x;
}

}  // namespace control