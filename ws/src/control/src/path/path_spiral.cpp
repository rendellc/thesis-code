#include <cassert>
#include <control/path/path_spiral.hpp>
#include <ignition/math/Vector3.hh>

PathSpiral::PathSpiral(const ignition::math::Vector2d& origin,
                       double orientation, double scale, double theta_begin,
                       double theta_end)
    : Path(Path::Type::SPIRAL),
      origin(origin),
      orientation(orientation),
      scale(scale),
      theta_begin(theta_begin),
      theta_end(theta_end) {}

double PathSpiral::distance(const ignition::math::Vector2d& pos) {
  const double theta = find_closest_theta(pos);
  return pos.Distance(spiral(theta));
}

double PathSpiral::distance(const ignition::math::Vector2d& pos,
                            double theta) const {
  return pos.Distance(spiral(theta));
}

double PathSpiral::theta_to_u(double theta) const {
  return (theta > 0 ? 1 : -1) * sqrt(fabs(theta));
}

double PathSpiral::u_to_theta(double u) const {
  return (u > 0 ? 1 : -1) * u * u;
}

ignition::math::Vector2d PathSpiral::spiral(double theta) const {
  const auto sign = theta > 0 ? 1 : -1;
  return origin + sign * scale * sqrt(fabs(theta)) *
                      ignition::math::Vector2d(cos(theta + orientation),
                                               sin(theta + orientation));
}

ignition::math::Vector2d PathSpiral::spiral_u(double u) const {
  const auto sign = u > 0 ? 1 : -1;
  return origin + scale * u *
                      ignition::math::Vector2d(cos(sign * u * u + orientation),
                                               sin(sign * u * u + orientation));
}

double PathSpiral::u_derivative(double u, double desired_speed) const {
  return desired_speed / (scale * sqrt(2 * (1 + 4 * u * u * u * u)));
}

double PathSpiral::u_double_derivative(double u, double desired_speed) const {
  return -desired_speed / (scale * sqrt(2)) * 8 * u * u * u /
         pow(1 + 4 * u * u * u * u, 1.5);
}

ignition::math::Vector2d PathSpiral::spiral_u_derivative(double u) const {
  const auto sign = u > 0 ? 1 : -1;
  const auto& k = scale;
  const auto c = cos(sign * u * u + orientation);
  const auto s = sin(sign * u * u + orientation);
  return ignition::math::Vector2d(k * c - 2 * k * sign * u * u * s,
                                  k * s + 2 * k * sign * u * u * c);
}

ignition::math::Vector2d PathSpiral::spiral_u_double_derivative(
    double u) const {
  const auto sign = u > 0 ? 1 : -1;
  const auto& k = scale;
  const auto c = cos(sign * u * u + orientation);
  const auto s = sin(sign * u * u + orientation);
  return ignition::math::Vector2d(
      6 * k * sign * u * (-s) + 4 * k * pow(u, 3) * (-c),
      6 * k * sign * u * (c) + 4 * k * pow(u, 3) * (-s));
}

std::vector<ignition::math::Vector2d> PathSpiral::sample(
    int number_of_samples) {
  std::vector<ignition::math::Vector2d> points;
  points.resize(number_of_samples);

  for (int i = 0; i < number_of_samples; i++) {
    const double n = static_cast<double>(number_of_samples);
    // i = 0      => interpolation = 0
    // i = n - 1  => interpolation = 1
    const double interpolation = n / (n - 1) * (i / n);

    const double theta =
        theta_begin + interpolation * (theta_end - theta_begin);
    points[i] = spiral(theta);
  }

  return points;
}

std::vector<ignition::math::Vector2d> PathSpiral::sample_direction(
    int number_of_samples) {
  std::vector<ignition::math::Vector2d> directions(number_of_samples);

  for (int i = 0; i < number_of_samples; i++) {
    const double n = static_cast<double>(number_of_samples);
    // i = 0      => interpolation = 0
    // i = n - 1  => interpolation = 1
    const double interpolation = n / (n - 1) * (i / n);
    const double theta =
        theta_begin + interpolation * (theta_end - theta_begin);

    // derivative is not defined in theta-space, so convert to u
    const double u = theta_to_u(theta);
    directions[i] = spiral_u_derivative(u).Normalized();
  }

  return directions;
}

ignition::math::Vector2d PathSpiral::closest_point(
    const ignition::math::Vector2d& pos) {
  return spiral(find_closest_theta(pos));
}

ignition::math::Vector2d PathSpiral::closest_direction(
    const ignition::math::Vector2d& pos) {
  const auto sign = theta_end > theta_begin ? 1 : -1;
  const double theta = find_closest_theta(pos);
  const double u = theta_to_u(theta);
  return sign * spiral_u_derivative(u).Normalized();
}

double PathSpiral::closest_courserate(const ignition::math::Vector2d& pos,
                                      const ignition::math::Vector2d& vel) {
  const auto sign = theta_end > theta_begin ? 1 : -1;
  const double theta = find_closest_theta(pos);
  const double u = theta_to_u(theta);

  const double speed = vel.Length();
  const double u_dot = u_derivative(u, speed);
  const double u_ddot = u_double_derivative(u, speed);

  using ignition::math::Vector2d;
  using ignition::math::Vector3d;

  const auto dpdu = spiral_u_derivative(u);
  const auto dvdu = spiral_u_double_derivative(u);

  const Vector2d v2 = dpdu * u_dot;
  const Vector2d vdot2 = dvdu * u_dot + dpdu * u_ddot;

  const Vector3d v(v2.X(), v2.Y(), 0);
  const Vector3d vdot(vdot2.X(), vdot2.Y(), 0);
  const Vector3d z_axis(0, 0, 1);

  const double courserate = z_axis.Cross(v).Dot(vdot) / v.SquaredLength();
  return courserate;
}

ignition::math::Vector2d PathSpiral::getBegin() const {
  return spiral(theta_begin);
}

ignition::math::Vector2d PathSpiral::getEnd() const {
  return spiral(theta_end);
}

double PathSpiral::find_closest_theta(const ignition::math::Vector2d& pos) {
  const auto cache_it = closest_cache.find(pos);
  if (cache_it != closest_cache.cend()) {
    return cache_it->second;
  }

  // value was not cached, compute it
  // transform variable since theta space has singularity

  // Sample distances to begin close to local optimum
  constexpr int num_samples = 10;
  const auto sample_positions = sample(num_samples);
  int sample_index_best = -1;
  double distance_sample_best = std::numeric_limits<double>::infinity();
  for (int i = 0; i < sample_positions.size(); i++) {
    const auto& sample_position = sample_positions[i];
    const double distance = sample_position.Distance(pos);
    if (distance < distance_sample_best) {
      sample_index_best = i;
      distance_sample_best = distance;
    }
  }
  const double n = static_cast<double>(num_samples);
  const double interpolation = n / (n - 1) * sample_index_best / n;
  double theta_sample_best =
      theta_begin + interpolation * (theta_end - theta_begin);

  // double theta_initial = (theta_begin + theta_end) / 2;
  // if (!closest_cache.empty()) {
  //   theta_initial = closest_cache.begin()->second;
  // }

  double u = theta_to_u(theta_sample_best);
  constexpr int max_iterations = 100;
  constexpr double eps = 0.001;
  double cost_derivative = 2 * (spiral_u(u) - pos).Dot(spiral_u_derivative(u));
  int iterations = 0;

  // Newtons method to find where d(distance)/dtheta = 0
  while (iterations < max_iterations &&  // u_begin <= u && u <= u_end &&
         fabs(cost_derivative) >= eps) {
    const auto pos_u = spiral_u(u);
    const auto vel_u = spiral_u_derivative(u);
    const auto acc_u = spiral_u_double_derivative(u);
    cost_derivative = 2 * (pos_u - pos).Dot(vel_u);
    const double cost_double_derivative =
        2 * (vel_u.Dot(vel_u)) + 2 * (pos_u - pos).Dot(acc_u);

    u = u - cost_derivative / cost_double_derivative;
    iterations++;
  }

  // TODO(rendellc): double check that cost_double_derivative > 0 at solution

  double theta_best = u_to_theta(u);
  double distance_best = spiral(theta_best).Distance(pos);
  const bool inside_domain =
      (theta_begin <= theta_best && theta_best <= theta_end) ||
      (theta_end <= theta_best && theta_best <= theta_begin);
  if (!inside_domain) {
    theta_best = theta_sample_best;
    distance_best = distance_sample_best;
  }

  if (iterations == max_iterations) {
    // std::cerr << "max iterations reached in Newtons method" << std::endl;
  }

  // (*const_cast<typeof(closest_cache)*>(&closest_cache))[pos] = theta;
  // we will in practise never look up the same position again in the future
  // so clear cache to avoid memory leak
  // TODO(rendellc): use a better data structure for this cache
  closest_cache.clear();
  closest_cache[pos] = theta_best;
  return theta_best;
}
