#pragma once
#include <functional>

namespace control {
double halleys_method(std::function<double(double)> f,
                      std::function<double(double)> df,
                      std::function<double(double)> ddf, double initial_value,
                      double threshold, int max_iterations);
}  // namespace control