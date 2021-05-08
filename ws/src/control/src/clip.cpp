#include <control/clip.hpp>

namespace control {
double clip(double value, double lower_limit, double upper_limit) {
  if (value < lower_limit) {
    return lower_limit;
  } else if (value > upper_limit) {
    return upper_limit;
  } else {
    return value;
  }
}

}  // namespace control