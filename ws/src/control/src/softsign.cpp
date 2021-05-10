#include <cmath>
#include <control/softsign.hpp>

namespace control {
double softsign(double value, double softregion) {
  if (fabs(value) < softregion) {
    return value / softregion;
  } else {
    return value / fabs(value);  // sign(x)
  }
}
}  // namespace control