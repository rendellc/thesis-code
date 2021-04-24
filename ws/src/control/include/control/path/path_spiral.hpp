#pragma once

#include <control/path/path.hpp>
#include <control/path/path_line.hpp>
#include <ignition/math/Vector2.hh>
#include <map>
#include <vector>

class PathSpiral : public Path {
 public:
  PathSpiral(const ignition::math::Vector2d& origin, double orientation,
             double scale, double theta_begin, double theta_end);

  virtual ~PathSpiral() = default;

  double distance(const ignition::math::Vector2d& pos) override;

  std::vector<ignition::math::Vector2d> sample(int number_of_samples) override;
  std::vector<ignition::math::Vector2d> sample_direction(
      int number_of_samples) override;

  ignition::math::Vector2d closest_point(
      const ignition::math::Vector2d& pos) override;
  ignition::math::Vector2d closest_direction(
      const ignition::math::Vector2d& pos) override;
  double closest_courserate(const ignition::math::Vector2d& pos) override;
  // ignition::math::Vector2d closest_point_step_ahead(
  //     const ignition::math::Vector2d& pos, double velocity, double time,
  //     double* time_left) override;

  ignition::math::Vector2d getBegin() const override;
  ignition::math::Vector2d getEnd() const override;

 private:
  const ignition::math::Vector2d origin;
  const double orientation;
  const double scale;
  const double theta_begin, theta_end;

  double theta_to_u(double theta) const;
  double u_to_theta(double u) const;

  ignition::math::Vector2d spiral(double theta) const;
  ignition::math::Vector2d spiral_u(double u) const;
  ignition::math::Vector2d spiral_u_derivative(double u) const;
  ignition::math::Vector2d spiral_u_double_derivative(double u) const;

  std::map<ignition::math::Vector2d, double> closest_cache;
  double find_closest_theta(const ignition::math::Vector2d& pos);

  double distance(const ignition::math::Vector2d& pos, double theta) const;
};
