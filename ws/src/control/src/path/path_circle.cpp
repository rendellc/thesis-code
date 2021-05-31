#include <cassert>
#include <control/path/path_circle.hpp>
#include <control/ssa.hpp>
#include <ignition/math/Vector3.hh>

using ignition::math::Vector2d;

PathCircle::PathCircle(const ignition::math::Vector2d& center, double radius,
                       double angle_begin, double angle_end)
    : Path(Path::Type::CIRCLE),
      center(center),
      radius(radius),
      angle_begin(angle_begin),
      angle_end(angle_end) {
  domain_lower = angle_begin < angle_end ? angle_begin : angle_end;
  domain_upper = angle_begin < angle_end ? angle_end : angle_begin;
}

double PathCircle::distance(const ignition::math::Vector2d& pos) {
  const double angle = closest_angle(pos);
  const auto circle_pos = circle_position(angle);
  return circle_pos.Distance(pos);
}

ignition::math::Vector2d PathCircle::circle_position(double angle) const {
  return center + radius * Vector2d(cos(angle), sin(angle));
}

ignition::math::Vector2d PathCircle::circle_velocity(double angle) const {
  // NOTE: assumes angle rate is 0
  return radius * Vector2d(-sin(angle), cos(angle));
}

std::vector<ignition::math::Vector2d> PathCircle::sample(
    int number_of_samples) {
  std::vector<ignition::math::Vector2d> points;
  points.resize(number_of_samples);

  for (int i = 0; i < number_of_samples; i++) {
    const double n = static_cast<double>(number_of_samples);
    // i = 0      => interpolation = 0
    // i = n - 1  => interpolation = 1
    const double interpolation = n / (n - 1) * (i / n);

    const double angle =
        angle_begin + interpolation * (angle_end - angle_begin);
    points[i] = circle_position(angle);
  }

  return points;
}

std::vector<ignition::math::Vector2d> PathCircle::sample_direction(
    int number_of_samples) {
  std::vector<ignition::math::Vector2d> directions(number_of_samples);

  for (int i = 0; i < number_of_samples; i++) {
    const double n = static_cast<double>(number_of_samples);
    // i = 0      => interpolation = 0
    // i = n - 1  => interpolation = 1
    const double interpolation = n / (n - 1) * (i / n);
    const double angle =
        angle_begin + interpolation * (angle_end - angle_begin);

    directions[i] = circle_velocity(angle);
  }

  return directions;
}

double PathCircle::closest_angle(const ignition::math::Vector2d& pos) {
  const auto circle_to_pos = (pos - center).Normalized();

  // with a full circle segment this is the closest point
  const auto closest_point_full = center + radius * circle_to_pos;
  double closest_point_full_angle =
      atan2(closest_point_full.Y(), closest_point_full.X());

  const double PI = 2 * atan2(+1.0, 0.0);

  while (closest_point_full_angle < domain_lower) {
    closest_point_full_angle += 2 * PI;
  }
  if (closest_point_full_angle < domain_upper) {
    return closest_point_full_angle;
  }

  // It's either begin or end
  const auto begin = getBegin();
  const auto end = getEnd();
  if (begin.Distance(pos) < end.Distance(pos)) {
    return angle_begin;
  } else {
    return angle_end;
  }
}

ignition::math::Vector2d PathCircle::closest_point(
    const ignition::math::Vector2d& pos) {
  const double angle = closest_angle(pos);
  return circle_position(angle);
}

ignition::math::Vector2d PathCircle::closest_direction(
    const ignition::math::Vector2d& pos) {
  const double angle = closest_angle(pos);
  return circle_velocity(angle).Normalized();
}

double PathCircle::closest_courserate(const ignition::math::Vector2d& pos) {
  // const double angle = closest_angle(pos);
  // NOTE: not implemented
  return 0.0;
}

ignition::math::Vector2d PathCircle::getBegin() const {
  return circle_position(angle_begin);
}

ignition::math::Vector2d PathCircle::getEnd() const {
  return circle_position(angle_end);
}
