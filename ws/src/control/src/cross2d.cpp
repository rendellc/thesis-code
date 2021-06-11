#include <control/cross2d.hpp>

namespace control {
double cross2d(const ignition::math::Vector2d& v1,
               const ignition::math::Vector2d& v2) {
  return v1.X() * v2.Y() - v1.Y() * v2.X();
}
}  // namespace control