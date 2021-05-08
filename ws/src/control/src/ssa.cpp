#include <cmath>
#include <control/ssa.hpp>

namespace control {
double ssa(double angle) { return atan2(sin(angle), cos(angle)); }

}  // namespace control