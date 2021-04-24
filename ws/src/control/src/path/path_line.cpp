#include <control/path/path_line.hpp>

PathLine::PathLine(const ignition::math::Vector2d& begin,
                   const ignition::math::Vector2d& end)
    : Path(Path::Type::LINE), pos_begin(begin), pos_end(end) {}

double PathLine::distance(const ignition::math::Vector2d& pos) {
  return closest_point(pos).Distance(pos);
}

std::vector<ignition::math::Vector2d> PathLine::sample(int number_of_samples) {
  std::vector<ignition::math::Vector2d> points(number_of_samples);
  for (int i = 0; i < number_of_samples; i++) {
    const double interpolation_variable =
        i / static_cast<double>(number_of_samples);
    points[i] = pos_begin + interpolation_variable * (pos_end - pos_begin);
  }

  points[number_of_samples - 1] = pos_end;

  return points;
}

std::vector<ignition::math::Vector2d> PathLine::sample_direction(
    int number_of_samples) {
  const auto direction = (pos_end - pos_begin).Normalized();
  std::vector<ignition::math::Vector2d> directions(number_of_samples);
  for (int i = 0; i < number_of_samples; i++) {
    directions[i] = direction;
  }

  return directions;
}

ignition::math::Vector2d PathLine::closest_point(
    const ignition::math::Vector2d& pos) {
  const auto step = pos_end - pos_begin;
  const double t = (pos - pos_begin).Dot(step) / step.Dot(step);
  if (t < 0) {
    // pos is behind begin
    return pos_begin;
  } else if (t > 1) {
    // pos is ahead of end
    return pos_end;
  } else {
    // pos is perpendicular to line
    const auto pos_line = pos_begin + t * (pos_end - pos_begin);
    return pos_line;
  }
}

// ignition::math::Vector2d PathLine::closest_point_step_ahead(
//     const ignition::math::Vector2d& pos, double velocity, double time) {
//   const auto pos_line = closest_point(pos);
//   const auto direction = closest_direction(pos);
//   const double distance_to_end = (pos_line - pos_end).Length();
//   const double time_on_path = std::min(time, distance_to_end / velocity);
//
//   const auto pos_ahead = pos_line + time_on_path * velocity * direction;
//   if (time_remaining) {
//     *time_remaining = time - time_on_path;
//   }
//
//   return pos_ahead;
// }

ignition::math::Vector2d PathLine::closest_direction(
    const ignition::math::Vector2d& pos) {
  return (pos_end - pos_begin).Normalized();
}

double PathLine::closest_courserate(const ignition::math::Vector2d& pos) {
  return 0.0;
}

ignition::math::Vector2d PathLine::getBegin() const { return pos_begin; }

ignition::math::Vector2d PathLine::getEnd() const { return pos_end; }
