#include <control/path/path.hpp>
#include <control/path/path_collection.hpp>
#include <control/path/path_line.hpp>
#include <control/path/path_spiral.hpp>

std::shared_ptr<Path> Path::straight_line_path(
    const std::vector<ignition::math::Vector2d>& waypoints) {
  std::vector<std::shared_ptr<Path>> subpaths;

  for (int i = 0; i < waypoints.size() - 1; i++) {
    subpaths.push_back(
        std::make_shared<PathLine>(waypoints[i], waypoints[i + 1]));
  }

  return std::make_shared<PathCollection>(subpaths);
}

std::shared_ptr<Path> Path::fermat_smoothing(
    const std::vector<ignition::math::Vector2d>& waypoints,
    double maximum_curvature) {
  std::vector<std::shared_ptr<Path>> subpaths;

  const double PI = atan2(+0.0, -1.0);
  const auto ssa = [](double angle) { return atan2(sin(angle), cos(angle)); };
  for (int i = 1; i < waypoints.size() - 1; i++) {
    const auto prev =
        (subpaths.empty()) ? waypoints[i - 1] : (*subpaths.crbegin())->getEnd();
    const auto& curr = waypoints[i];

    const auto& next = waypoints[i + 1];

    const auto direction_in = (curr - prev).Normalized();
    const auto direction_out = (next - curr).Normalized();
    const double course_in = atan2(direction_in.Y(), direction_in.X());
    const double course_out = atan2(direction_out.Y(), direction_out.X());
    const double course_change = fabs(ssa(course_out - course_in));
    const int turn_direction = ssa(course_out - course_in) > 0 ? 1 : -1;

    // Halleys method to determine theta_end
    double theta_end = 0.0;
    constexpr double threshold = 0.001;
    const auto f = [=](double theta_end) {
      // f(theta_end) = 0 defines theta_end
      return theta_end + atan(2 * theta_end) - course_change / 2;
    };
    const auto df = [=](double theta_end) {
      return 1.0 + 2 / (1 + 4 * pow(theta_end, 2));
    };
    const auto ddf = [=](double theta_end) {
      return -16 * theta_end / pow(1 + 4 * pow(theta_end, 2), 2);
    };
    while (fabs(f(theta_end)) > threshold) {
      const double y = f(theta_end);
      const double dy = df(theta_end);
      const double ddy = ddf(theta_end);

      theta_end = theta_end - 2 * y * dy / (2 * pow(dy, 2) - y * ddy);
    }

    const double theta_max_curvature =
        std::min(theta_end, sqrt(sqrt(7) / 2 - 1.25));
    const double scale = 1 / maximum_curvature * 2 * sqrt(theta_max_curvature) *
                         (3 + 4 * pow(theta_max_curvature, 2)) /
                         sqrt(pow(1 + 4 * pow(theta_max_curvature, 2), 3));

    const double alpha = (PI - course_change) / 2;
    const double h = scale * sqrt(theta_end) * sin(theta_end);
    const double l1 = scale * sqrt(theta_end) * cos(theta_end);
    const double l2 = h / tan(alpha);
    const double l = l1 + l2;

    const auto spiral_begin = curr - l * direction_in;
    const auto spiral_end = curr + l * direction_out;

    subpaths.push_back(std::make_shared<PathLine>(prev, spiral_begin));
    // if (theta_end >= 0) {
    std::cout << "theta_end " << theta_end << std::endl;
    if (turn_direction == 1) {
      subpaths.push_back(std::make_shared<PathSpiral>(spiral_begin, course_in,
                                                      scale, 0, theta_end));
      subpaths.push_back(std::make_shared<PathSpiral>(spiral_end, course_out,
                                                      scale, -theta_end, 0));
    } else {
      subpaths.push_back(std::make_shared<PathSpiral>(
          spiral_begin, course_in + PI, scale, 0, -theta_end));
      subpaths.push_back(std::make_shared<PathSpiral>(
          spiral_end, course_out + PI, scale, theta_end, 0));
    }
    // } else {
    //   subpaths.push_back(std::make_shared<PathSpiral>(
    //       spiral_begin, course_in + PI, scale, 0, theta_end));
    //   subpaths.push_back(std::make_shared<PathSpiral>(
    //       spiral_end, course_out + PI, scale, -theta_end, 0));
    // }
  }

  // Add straight line to final waypoint
  const auto secondtolast =
      (subpaths.empty()) ? waypoints[0] : (*subpaths.crbegin())->getEnd();
  subpaths.push_back(
      std::make_shared<PathLine>(secondtolast, *waypoints.crbegin()));

  return std::make_shared<PathCollection>(subpaths);
}
