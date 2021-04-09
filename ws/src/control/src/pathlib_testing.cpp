// #include <control/path/path.hpp>
#include <control/path/path_line.hpp>
#include <control/path/path_spiral.hpp>
#include <ignition/math/Vector2.hh>
#include <memory>

using Vec2 = ignition::math::Vector2d;

int main() {
  std::vector<Vec2> waypoints = {Vec2(0, 0), Vec2(7, 3), Vec2(10, 10),
                                 Vec2(5, 15), Vec2(0, 10)};

  constexpr double minimum_turn_radius = 1.5;
  [[maybe_unused]] constexpr double maximum_curvature = 1 / minimum_turn_radius;
  // TODO: implement Path interface for [Path*]
  // const auto path = Path::fermat_smoothing(waypoints, maximum_curvature);
  const auto path = std::make_shared<PathSpiral>(Vec2(0, 0), 0, 30, 0, 2);

  // const auto path = Path::straight_line_path(waypoints);
  const auto points = path->sample(50);
  for (const auto& p : points) {
    std::cout << p.X() << "," << p.Y() << "\n";
  }

  return 0;
}
