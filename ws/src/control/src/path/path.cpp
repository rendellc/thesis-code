#include <control/clip.hpp>
#include <control/halley.hpp>
#include <control/path/path.hpp>
#include <control/path/path_circle.hpp>
#include <control/path/path_collection.hpp>
#include <control/path/path_line.hpp>
#include <control/path/path_spiral.hpp>
#include <control/ssa.hpp>

using control::halleys_method;
using control::ssa;
using ignition::math::Vector2d;

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

  for (int i = 1; i < waypoints.size() - 1; i++) {
    const auto prev =
        (subpaths.empty()) ? waypoints[i - 1] : (*subpaths.crbegin())->getEnd();
    const auto& curr = waypoints[i];

    const auto& next = waypoints[i + 1];

    const auto direction_in = (curr - prev).Normalized();
    const auto direction_out = (next - curr).Normalized();
    const double course_in = atan2(direction_in.Y(), direction_in.X());
    const double course_out = atan2(direction_out.Y(), direction_out.X());
    const double course_change = ssa(course_out - course_in);
    const double course_change_mag = fabs(course_change);
    const int turn_direction = ssa(course_out - course_in) > 0 ? 1 : -1;

    // Halleys method to determine theta_end
    double theta_end = 0.0;
    constexpr double threshold = 0.001;
    const auto f = [=](double theta_end) {
      // f(theta_end) = 0 defines theta_end
      return theta_end + atan(2 * theta_end) - course_change_mag / 2;
    };
    const auto df = [=](double theta_end) {
      return 1.0 + 2 / (1 + 4 * pow(theta_end, 2));
    };
    const auto ddf = [=](double theta_end) {
      return -16 * theta_end / pow(1 + 4 * pow(theta_end, 2), 2);
    };
    theta_end = halleys_method(f, df, ddf, theta_end, threshold, 1000);

    const double theta_max_curvature_mag =
        std::min(fabs(theta_end), sqrt(sqrt(7) / 2 - 1.25));

    // const double theta_max_curvature =
    //     clip(theta_end, -sqrt(sqrt(7) / 2 - 1.25), sqrt(sqrt(7) / 2 - 1.25));

    const double scale = 1 / maximum_curvature * 2 *
                         sqrt(theta_max_curvature_mag) *
                         (3 + 4 * pow(theta_max_curvature_mag, 2)) /
                         sqrt(pow(1 + 4 * pow(theta_max_curvature_mag, 2), 3));

    const double alpha = (PI - course_change_mag) / 2;
    const double h = scale * sqrt(theta_end) * sin(theta_end);
    const double l1 = scale * sqrt(theta_end) * cos(theta_end);
    const double l2 = h / tan(alpha);
    const double l = l1 + l2;

    const auto spiral_begin = curr - l * direction_in;
    const auto spiral_end = curr + l * direction_out;

    subpaths.push_back(std::make_shared<PathLine>(prev, spiral_begin));
    // std::cout << "theta_end " << theta_end << std::endl;
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
  }

  // Add straight line to final waypoint
  const auto secondtolast =
      (subpaths.empty()) ? waypoints[0] : (*subpaths.crbegin())->getEnd();
  subpaths.push_back(
      std::make_shared<PathLine>(secondtolast, *waypoints.crbegin()));

  return std::make_shared<PathCollection>(subpaths);
}

std::shared_ptr<Path> Path::circular_smoothing(
    const std::vector<ignition::math::Vector2d>& waypoints, double radius) {
  std::vector<std::shared_ptr<Path>> subpaths;

  const double PI = atan2(+0.0, -1.0);
  for (int i = 1; i < waypoints.size() - 1; i++) {
    const auto prev =
        (subpaths.empty()) ? waypoints[i - 1] : (*subpaths.crbegin())->getEnd();
    const auto& curr = waypoints[i];
    const auto& next = waypoints[i + 1];

    const auto v0 = curr - prev;
    const auto v1 = next - curr;

    const auto cross_2d = [](const Vector2d& v0, const Vector2d& v1) {
      return v0.X() * v1.Y() - v1.X() * v0.Y();
    };

    const int turn_direction = cross_2d(v0, v1) > 0 ? 1 : -1;
    const auto& s = turn_direction;

    // Normals
    const auto n0 = Vector2d(-v0.Y(), v0.X()).Normalized();
    const auto n1 = Vector2d(-v1.Y(), v1.X()).Normalized();

    const auto p0c = prev + radius * s * n0;
    const auto p1c = curr + radius * s * n1;

    // find intersection between lines
    const double s0 = -1 / cross_2d(v0, v1) * cross_2d(v1, p1c - p0c);
    const double s1 = -1 / cross_2d(v0, v1) * cross_2d(v0, p1c - p0c);

    // circle center
    const auto center = p0c + s0 * v0;

    // tangent points
    const auto t0 = prev + s0 * v0;
    const auto t1 = curr + s1 * v1;

    const double angle_begin = atan2(t0.Y() - center.Y(), t0.X() - center.X());
    double angle_end = atan2(t1.Y() - center.Y(), t1.X() - center.X());
    angle_end = angle_begin + ssa(angle_end - angle_begin);

    subpaths.push_back(std::make_shared<PathLine>(prev, t0));
    subpaths.push_back(
        std::make_shared<PathCircle>(center, radius, angle_begin, angle_end));
  }

  const auto secondtolast =
      (subpaths.empty()) ? waypoints[0] : (*subpaths.crbegin())->getEnd();
  subpaths.push_back(
      std::make_shared<PathLine>(secondtolast, *waypoints.crbegin()));

  return std::make_shared<PathCollection>(subpaths);
}